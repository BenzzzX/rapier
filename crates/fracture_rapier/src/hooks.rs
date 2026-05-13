use std::collections::BTreeMap;
use std::sync::{Arc, Mutex, RwLock};

use rapier2d::prelude::*;

use crate::collider_sync::{DestructibleActorRef, VoxelContact};
use crate::contact_map::{
    ContactPairMapping, ContactPairSide, PreSolverContactKey, PreSolverContactMapping,
    collider_key, contact_pair_key,
};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactMaterialProperties {
    pub friction: f32,
    pub restitution: f32,
}

impl Default for ContactMaterialProperties {
    fn default() -> Self {
        Self {
            friction: 0.5,
            restitution: 0.0,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct HookObservation {
    pub collider1: ColliderHandle,
    pub collider2: ColliderHandle,
    pub before_solver_contacts: usize,
    pub after_solver_contacts: usize,
    pub cache_pair_key: ((u32, u32), (u32, u32)),
    pub cache_sequence: u64,
    pub side: ContactPairSide,
    pub destructible_collider: ColliderHandle,
    pub other_collider: ColliderHandle,
    pub subshape: u32,
    pub friction: f32,
    pub restitution: f32,
    pub material: u16,
    pub voxel: Option<VoxelContact>,
    pub used_fallback: bool,
}

#[derive(Clone, Debug, Default)]
pub(crate) struct ContactMaterialRegistry {
    pub collider_actors: BTreeMap<(u32, u32), DestructibleActorRef>,
    pub collider_voxels: BTreeMap<(u32, u32), Vec<VoxelContact>>,
    pub material_properties: BTreeMap<u16, ContactMaterialProperties>,
}

#[derive(Clone, Debug)]
pub struct FxContactHooks {
    registry: Arc<RwLock<ContactMaterialRegistry>>,
    pre_solver_cache: Arc<Mutex<Vec<PreSolverContactMapping>>>,
    observations: Arc<Mutex<Vec<HookObservation>>>,
}

impl Default for FxContactHooks {
    fn default() -> Self {
        Self::new()
    }
}

impl FxContactHooks {
    pub fn new() -> Self {
        Self {
            registry: Arc::new(RwLock::new(ContactMaterialRegistry::default())),
            pre_solver_cache: Arc::new(Mutex::new(Vec::new())),
            observations: Arc::new(Mutex::new(Vec::new())),
        }
    }

    pub(crate) fn registry(&self) -> Arc<RwLock<ContactMaterialRegistry>> {
        self.registry.clone()
    }

    pub fn set_material_properties(&self, material: u16, properties: ContactMaterialProperties) {
        self.registry
            .write()
            .expect("contact material registry poisoned")
            .material_properties
            .insert(material, properties);
    }

    pub fn drain_observations(&self) -> Vec<HookObservation> {
        std::mem::take(
            &mut *self
                .observations
                .lock()
                .expect("contact hook observations poisoned"),
        )
    }

    pub(crate) fn clear_pre_solver_contact_cache(&self) {
        self.pre_solver_cache
            .lock()
            .expect("pre-solver contact cache poisoned")
            .clear();
    }

    pub(crate) fn pre_solver_contact_cache_snapshot(&self) -> Vec<PreSolverContactMapping> {
        self.pre_solver_cache
            .lock()
            .expect("pre-solver contact cache poisoned")
            .clone()
    }
}

impl PhysicsHooks for FxContactHooks {
    fn modify_solver_contacts(&self, context: &mut ContactModificationContext) {
        let registry = self
            .registry
            .read()
            .expect("contact material registry poisoned");
        let mappings = contact_context_mappings(context, &registry);
        let Some(primary) = mappings.first().copied() else {
            return;
        };
        let primary_voxel = contact_context_voxel(primary, context, &registry);
        let material = primary_voxel
            .map(|voxel| voxel.contact_material)
            .unwrap_or_default();
        let properties = registry
            .material_properties
            .get(&material)
            .copied()
            .unwrap_or_default();
        let before = context.solver_contacts.len();
        for contact in context.solver_contacts.iter_mut() {
            contact.friction = properties.friction;
            contact.restitution = properties.restitution;
        }
        let after = context.solver_contacts.len();
        let solver_contact_ids = context
            .solver_contacts
            .iter()
            .map(|contact| contact.contact_id[0])
            .collect::<Vec<_>>();
        let pair_key = contact_pair_key(context.collider1, context.collider2);
        let mut cache = self
            .pre_solver_cache
            .lock()
            .expect("pre-solver contact cache poisoned");
        let mut cached_mappings = Vec::new();
        for mapping in mappings {
            let voxel = contact_context_voxel(mapping, context, &registry);
            let (destructible_subshape, other_subshape) =
                contact_context_subshapes(mapping.side, context);
            let stable_key = PreSolverContactKey {
                pair: pair_key,
                sequence: cache.len() as u64,
                side: mapping.side,
                destructible_subshape,
                other_subshape,
            };
            let material = voxel
                .map(|voxel| voxel.contact_material)
                .unwrap_or_default();
            let cached = PreSolverContactMapping {
                stable_key,
                pair: mapping,
                destructible_subshape,
                other_subshape,
                material,
                voxel,
                node: voxel.map(|voxel| voxel.node),
                used_fallback: voxel.is_none(),
                solver_contact_count: after,
                solver_contact_ids: solver_contact_ids.clone(),
            };
            cache.push(cached.clone());
            cached_mappings.push(cached);
        }
        drop(registry);
        drop(cache);

        let mut observations = self
            .observations
            .lock()
            .expect("contact hook observations poisoned");
        for cached in cached_mappings {
            observations.push(HookObservation {
                collider1: context.collider1,
                collider2: context.collider2,
                before_solver_contacts: before,
                after_solver_contacts: after,
                cache_pair_key: cached.stable_key.pair,
                cache_sequence: cached.stable_key.sequence,
                side: cached.pair.side,
                destructible_collider: cached.pair.destructible_collider,
                other_collider: cached.pair.other_collider,
                subshape: cached.destructible_subshape,
                friction: properties.friction,
                restitution: properties.restitution,
                material: cached.material,
                voxel: cached.voxel,
                used_fallback: cached.used_fallback,
            });
        }
    }
}

fn contact_context_mappings(
    context: &ContactModificationContext,
    registry: &ContactMaterialRegistry,
) -> Vec<ContactPairMapping> {
    let mut out = Vec::new();
    if let Some(destructible) = registry
        .collider_actors
        .get(&collider_key(context.collider1))
        .copied()
    {
        out.push(ContactPairMapping {
            destructible,
            destructible_collider: context.collider1,
            other_collider: context.collider2,
            side: ContactPairSide::Collider1,
        });
    }
    if let Some(destructible) = registry
        .collider_actors
        .get(&collider_key(context.collider2))
        .copied()
    {
        out.push(ContactPairMapping {
            destructible,
            destructible_collider: context.collider2,
            other_collider: context.collider1,
            side: ContactPairSide::Collider2,
        });
    }
    out
}

fn contact_context_voxel(
    mapping: ContactPairMapping,
    context: &ContactModificationContext,
    registry: &ContactMaterialRegistry,
) -> Option<VoxelContact> {
    let (destructible_subshape, _) = contact_context_subshapes(mapping.side, context);
    registry
        .collider_voxels
        .get(&collider_key(mapping.destructible_collider))
        .and_then(|voxels| {
            voxels
                .iter()
                .find(|voxel| voxel.subshape == destructible_subshape)
        })
        .copied()
}

fn contact_context_subshapes(
    side: ContactPairSide,
    context: &ContactModificationContext,
) -> (u32, u32) {
    match side {
        ContactPairSide::Collider1 => (context.manifold.subshape1, context.manifold.subshape2),
        ContactPairSide::Collider2 => (context.manifold.subshape2, context.manifold.subshape1),
    }
}
