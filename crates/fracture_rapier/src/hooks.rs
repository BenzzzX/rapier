use std::collections::BTreeMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, RwLock};

use rapier2d::prelude::*;

use crate::collider_sync::{DestructibleActorRef, VoxelContact};
use crate::contact_map::{
    ContactPairMapping, ContactPairSide, DestructibleActorContactMetadata, PreSolverContactKey,
    PreSolverContactMapping, QuickImpactAction, QuickImpactEstimate, collider_key,
    contact_pair_key,
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
    pub actor_metadata: Option<DestructibleActorContactMetadata>,
    pub quick_impact: Option<QuickImpactEstimate>,
    pub used_fallback: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QuickImpactSettings {
    pub enabled: bool,
    pub soften_enabled: bool,
    pub suppress_enabled: bool,
    pub static_soften_impulse_threshold: f32,
    pub static_suppress_impulse_threshold: f32,
    pub dynamic_soften_impulse_threshold: f32,
    pub dynamic_suppress_impulse_threshold: f32,
    pub penetration_impulse_scale: f32,
    pub stress_force_scale: f32,
    pub softened_friction_scale: f32,
    pub softened_restitution_scale: f32,
    pub suppress_tunnel_window_frames: u16,
}

impl Default for QuickImpactSettings {
    fn default() -> Self {
        Self {
            enabled: false,
            soften_enabled: true,
            suppress_enabled: true,
            static_soften_impulse_threshold: 2.0,
            static_suppress_impulse_threshold: 6.0,
            dynamic_soften_impulse_threshold: 8.0,
            dynamic_suppress_impulse_threshold: 24.0,
            penetration_impulse_scale: 1.0,
            stress_force_scale: 1.0,
            softened_friction_scale: 0.25,
            softened_restitution_scale: 0.0,
            suppress_tunnel_window_frames: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct SuppressTunnelWindowKey {
    projectile_collider: (u32, u32),
    family: fracture_core::FxFamilyId,
}

#[derive(Clone, Debug, Default)]
pub(crate) struct ContactMaterialRegistry {
    pub collider_actors: BTreeMap<(u32, u32), DestructibleActorRef>,
    pub collider_voxels: BTreeMap<(u32, u32), Vec<VoxelContact>>,
    pub actor_metadata: BTreeMap<DestructibleActorRef, DestructibleActorContactMetadata>,
    pub material_properties: BTreeMap<u16, ContactMaterialProperties>,
    pub material_hardness: BTreeMap<u16, f32>,
    pub quick_impact_settings: QuickImpactSettings,
}

#[derive(Clone, Debug)]
pub struct FxContactHooks {
    registry: Arc<RwLock<ContactMaterialRegistry>>,
    pre_solver_cache: Arc<Mutex<Vec<PreSolverContactMapping>>>,
    observations: Arc<Mutex<Vec<HookObservation>>>,
    current_tick: Arc<AtomicU64>,
    suppress_tunnel_windows: Arc<Mutex<BTreeMap<SuppressTunnelWindowKey, u64>>>,
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
            current_tick: Arc::new(AtomicU64::new(0)),
            suppress_tunnel_windows: Arc::new(Mutex::new(BTreeMap::new())),
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

    pub fn set_quick_impact_settings(&self, settings: QuickImpactSettings) {
        self.registry
            .write()
            .expect("contact material registry poisoned")
            .quick_impact_settings = sanitize_quick_impact_settings(settings);
    }

    pub fn quick_impact_settings(&self) -> QuickImpactSettings {
        self.registry
            .read()
            .expect("contact material registry poisoned")
            .quick_impact_settings
    }

    pub fn set_material_impact_hardness(&self, material: u16, hardness: f32) {
        self.registry
            .write()
            .expect("contact material registry poisoned")
            .material_hardness
            .insert(material, hardness.max(0.0));
    }

    pub fn material_impact_hardness(&self, material: u16) -> f32 {
        self.registry
            .read()
            .expect("contact material registry poisoned")
            .material_hardness
            .get(&material)
            .copied()
            .unwrap_or(1.0)
            .max(0.0)
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

    pub(crate) fn begin_step(&self, tick: u64) {
        self.current_tick.store(tick, Ordering::Relaxed);
        self.suppress_tunnel_windows
            .lock()
            .expect("suppress tunnel windows poisoned")
            .retain(|_, expires_at| *expires_at >= tick);
    }

    #[cfg(test)]
    pub(crate) fn suppress_tunnel_window_active(
        &self,
        projectile_collider: ColliderHandle,
        family: fracture_core::FxFamilyId,
    ) -> bool {
        let tick = self.current_tick.load(Ordering::Relaxed);
        self.suppress_tunnel_windows
            .lock()
            .expect("suppress tunnel windows poisoned")
            .get(&SuppressTunnelWindowKey {
                projectile_collider: collider_key(projectile_collider),
                family,
            })
            .is_some_and(|expires_at| *expires_at >= tick)
    }

    pub(crate) fn pre_solver_contact_cache_snapshot(&self) -> Vec<PreSolverContactMapping> {
        self.pre_solver_cache
            .lock()
            .expect("pre-solver contact cache poisoned")
            .clone()
    }
}

impl PhysicsHooks for FxContactHooks {
    fn filter_contact_pair(&self, context: &PairFilterContext) -> Option<SolverFlags> {
        let registry = self
            .registry
            .read()
            .expect("contact material registry poisoned");
        let family1 = registry
            .collider_actors
            .get(&collider_key(context.collider1))
            .map(|actor| actor.family);
        let family2 = registry
            .collider_actors
            .get(&collider_key(context.collider2))
            .map(|actor| actor.family);
        drop(registry);

        let tick = self.current_tick.load(Ordering::Relaxed);
        let windows = self
            .suppress_tunnel_windows
            .lock()
            .expect("suppress tunnel windows poisoned");
        let suppress = match (family1, family2) {
            (Some(family), None) => windows.get(&SuppressTunnelWindowKey {
                projectile_collider: collider_key(context.collider2),
                family,
            }),
            (None, Some(family)) => windows.get(&SuppressTunnelWindowKey {
                projectile_collider: collider_key(context.collider1),
                family,
            }),
            _ => None,
        }
        .is_some_and(|expires_at| *expires_at >= tick);

        if suppress {
            Some(SolverFlags::empty())
        } else {
            Some(SolverFlags::COMPUTE_IMPULSES)
        }
    }

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
        let pair_contains_small_debris = mappings
            .iter()
            .copied()
            .any(|mapping| mapping_is_small_debris(mapping, &registry));
        let pair_is_destructible_vs_destructible = mappings.len() > 1;
        let quick_impacts = if pair_contains_small_debris || pair_is_destructible_vs_destructible {
            vec![None; mappings.len()]
        } else {
            mappings
                .iter()
                .copied()
                .map(|mapping| quick_impact_for_mapping(mapping, context, &registry))
                .collect::<Vec<_>>()
        };
        let strongest_action = quick_impacts
            .iter()
            .filter_map(|impact| impact.map(|impact| impact.action))
            .max();
        for contact in context.solver_contacts.iter_mut() {
            contact.friction = properties.friction;
            contact.restitution = properties.restitution;
        }
        if strongest_action == Some(QuickImpactAction::Soften) {
            let settings = registry.quick_impact_settings;
            for contact in context.solver_contacts.iter_mut() {
                contact.friction *= settings.softened_friction_scale;
                contact.restitution *= settings.softened_restitution_scale;
            }
        } else if strongest_action == Some(QuickImpactAction::Suppress) {
            register_suppress_tunnel_windows(
                context,
                &registry,
                &mappings,
                &quick_impacts,
                &self.current_tick,
                &self.suppress_tunnel_windows,
            );
            context.solver_contacts.clear();
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
        for (mapping, quick_impact) in mappings.into_iter().zip(quick_impacts) {
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
                actor_metadata: registry.actor_metadata.get(&mapping.destructible).copied(),
                quick_impact,
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
                actor_metadata: cached.actor_metadata,
                quick_impact: cached.quick_impact,
                used_fallback: cached.used_fallback,
            });
        }
    }
}

fn mapping_is_small_debris(
    mapping: ContactPairMapping,
    registry: &ContactMaterialRegistry,
) -> bool {
    registry
        .actor_metadata
        .get(&mapping.destructible)
        .is_some_and(|metadata| metadata.small_debris)
}

fn register_suppress_tunnel_windows(
    context: &ContactModificationContext,
    registry: &ContactMaterialRegistry,
    mappings: &[ContactPairMapping],
    quick_impacts: &[Option<QuickImpactEstimate>],
    current_tick: &AtomicU64,
    suppress_tunnel_windows: &Mutex<BTreeMap<SuppressTunnelWindowKey, u64>>,
) {
    let settings = registry.quick_impact_settings;
    let window_frames = u64::from(settings.suppress_tunnel_window_frames);
    if window_frames == 0 {
        return;
    }

    let tick = current_tick.load(Ordering::Relaxed);
    let expires_at = tick.saturating_add(window_frames);
    let mut windows = suppress_tunnel_windows
        .lock()
        .expect("suppress tunnel windows poisoned");
    for (mapping, quick_impact) in mappings.iter().copied().zip(quick_impacts.iter().copied()) {
        let Some(quick_impact) = quick_impact else {
            continue;
        };
        if quick_impact.action != QuickImpactAction::Suppress {
            continue;
        }
        let Some(destructible_body) = destructible_body_for_mapping(mapping, context) else {
            continue;
        };
        if destructible_body.is_dynamic()
            || registry
                .collider_actors
                .contains_key(&collider_key(mapping.other_collider))
        {
            continue;
        }
        windows.insert(
            SuppressTunnelWindowKey {
                projectile_collider: collider_key(mapping.other_collider),
                family: mapping.destructible.family,
            },
            expires_at,
        );
    }
}

fn sanitize_quick_impact_settings(mut settings: QuickImpactSettings) -> QuickImpactSettings {
    settings.static_soften_impulse_threshold = settings.static_soften_impulse_threshold.max(0.0);
    settings.static_suppress_impulse_threshold = settings
        .static_suppress_impulse_threshold
        .max(settings.static_soften_impulse_threshold);
    settings.dynamic_soften_impulse_threshold = settings.dynamic_soften_impulse_threshold.max(0.0);
    settings.dynamic_suppress_impulse_threshold = settings
        .dynamic_suppress_impulse_threshold
        .max(settings.dynamic_soften_impulse_threshold);
    settings.penetration_impulse_scale = settings.penetration_impulse_scale.max(0.0);
    settings.stress_force_scale = settings.stress_force_scale.max(0.0);
    settings.softened_friction_scale = settings.softened_friction_scale.clamp(0.0, 1.0);
    settings.softened_restitution_scale = settings.softened_restitution_scale.clamp(0.0, 1.0);
    settings
}

fn destructible_body_for_mapping<'a>(
    mapping: ContactPairMapping,
    context: &'a ContactModificationContext<'a>,
) -> Option<&'a RigidBody> {
    match mapping.side {
        ContactPairSide::Collider1 => context
            .rigid_body1
            .and_then(|handle| context.bodies.get(handle)),
        ContactPairSide::Collider2 => context
            .rigid_body2
            .and_then(|handle| context.bodies.get(handle)),
    }
}

fn quick_impact_for_mapping(
    mapping: ContactPairMapping,
    context: &ContactModificationContext,
    registry: &ContactMaterialRegistry,
) -> Option<QuickImpactEstimate> {
    let settings = registry.quick_impact_settings;
    if !settings.enabled {
        return None;
    }
    let metadata = registry
        .actor_metadata
        .get(&mapping.destructible)
        .copied()?;
    if metadata.small_debris {
        return None;
    }
    let voxel = contact_context_voxel(mapping, context, registry);
    let material = voxel
        .map(|voxel| voxel.contact_material)
        .unwrap_or_default();
    let hardness = registry
        .material_hardness
        .get(&material)
        .copied()
        .unwrap_or(1.0)
        .max(0.0);
    if hardness == 0.0 {
        return None;
    }
    let (body1, body2) = (
        context
            .rigid_body1
            .and_then(|handle| context.bodies.get(handle)),
        context
            .rigid_body2
            .and_then(|handle| context.bodies.get(handle)),
    );
    let (destructible_body, other_body) = match mapping.side {
        ContactPairSide::Collider1 => (body1, body2),
        ContactPairSide::Collider2 => (body2, body1),
    };
    let destructible_body = destructible_body?;
    let normal = *context.normal;
    let normal_sign = match mapping.side {
        ContactPairSide::Collider1 => 1.0,
        ContactPairSide::Collider2 => -1.0,
    };
    if destructible_body.is_dynamic() {
        return None;
    }

    let dynamic_opponent = other_body.is_some_and(RigidBody::is_dynamic);
    let (soften_threshold, suppress_threshold) = (
        settings.static_soften_impulse_threshold,
        settings.static_suppress_impulse_threshold,
    );

    let destructible_mass = destructible_body.mass().max(0.0);
    let other_mass = other_body
        .filter(|body| body.is_dynamic())
        .map(|body| body.mass().max(0.0));
    let effective_mass = match other_mass {
        Some(other_mass) if destructible_mass > 0.0 && other_mass > 0.0 => {
            1.0 / (1.0 / destructible_mass + 1.0 / other_mass)
        }
        Some(other_mass) if other_mass > 0.0 => other_mass,
        _ => destructible_mass,
    };
    if !effective_mass.is_finite() || effective_mass <= 0.0 {
        return None;
    }

    let mut best = None;
    for contact in context.solver_contacts.iter() {
        let vel1 = body1
            .map(|body| body.velocity_at_point(contact.point))
            .unwrap_or_default();
        let vel2 = body2
            .map(|body| body.velocity_at_point(contact.point))
            .unwrap_or_default();
        let relative_normal_speed = (-(vel2 - vel1).dot(normal)).max(0.0);
        let penetration_speed = (-contact.dist).max(0.0) * settings.penetration_impulse_scale;
        let raw_impulse = (relative_normal_speed + penetration_speed) * effective_mass;
        let scaled_impulse = raw_impulse * hardness;
        let action = if settings.suppress_enabled && scaled_impulse >= suppress_threshold {
            QuickImpactAction::Suppress
        } else if settings.soften_enabled && scaled_impulse >= soften_threshold {
            QuickImpactAction::Soften
        } else {
            continue;
        };
        let estimate = QuickImpactEstimate {
            action,
            point: contact.point,
            normal,
            impulse: normal * (raw_impulse * normal_sign),
            relative_normal_speed,
            effective_mass,
            scaled_impulse,
            threshold: if action == QuickImpactAction::Suppress {
                suppress_threshold
            } else {
                soften_threshold
            },
            hardness,
            contact_id: contact.contact_id[0],
            dynamic_opponent,
        };
        if best.is_none_or(|best: QuickImpactEstimate| {
            estimate.action > best.action
                || (estimate.action == best.action && estimate.scaled_impulse > best.scaled_impulse)
        }) {
            best = Some(estimate);
        }
    }
    best
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
