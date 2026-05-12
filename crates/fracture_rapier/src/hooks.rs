use std::collections::BTreeMap;
use std::sync::{Arc, Mutex, RwLock};

use rapier2d::prelude::*;

use crate::collider_sync::{DestructibleActorRef, VoxelContact};
use crate::contact_map::collider_key;

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
}

impl PhysicsHooks for FxContactHooks {
    fn modify_solver_contacts(&self, context: &mut ContactModificationContext) {
        let registry = self
            .registry
            .read()
            .expect("contact material registry poisoned");
        let side = if registry
            .collider_actors
            .contains_key(&collider_key(context.collider1))
        {
            Some((context.collider1, context.manifold.subshape1))
        } else if registry
            .collider_actors
            .contains_key(&collider_key(context.collider2))
        {
            Some((context.collider2, context.manifold.subshape2))
        } else {
            None
        };
        let Some((collider, subshape)) = side else {
            return;
        };
        let voxel = registry
            .collider_voxels
            .get(&collider_key(collider))
            .and_then(|voxels| voxels.iter().find(|voxel| voxel.subshape == subshape))
            .copied();
        let material = voxel
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
        drop(registry);

        self.observations
            .lock()
            .expect("contact hook observations poisoned")
            .push(HookObservation {
                collider1: context.collider1,
                collider2: context.collider2,
                before_solver_contacts: before,
                after_solver_contacts: after,
                friction: properties.friction,
                restitution: properties.restitution,
                material,
                voxel,
                used_fallback: voxel.is_none(),
            });
    }
}
