use std::collections::{BTreeMap, BTreeSet};

use fracture_core::{
    CommandId, DamageSource, DeterministicOrderKey, FxFamily, FxFamilyId, StressInput, Vec2,
};
use rapier2d::prelude::*;

use crate::contact_map::{
    ContactPairKey, ContactPairMapping, ContactPairSide, PreSolverContactMapping,
    QuickImpactEstimate, collider_key, map_contact_pair_destructibles, stable_pair_key,
};
use crate::hooks::ContactMaterialRegistry;

#[derive(Clone, Debug, PartialEq)]
pub struct TrackedContactImpulse {
    pub pre_solver_pair_key: ((u32, u32), (u32, u32)),
    pub pre_solver_sequence: u64,
    pub mapping: ContactPairMapping,
    pub voxel: Option<crate::collider_sync::VoxelContact>,
    pub material: u16,
    pub used_fallback: bool,
    pub manifold_index: usize,
    pub contact_index: usize,
    pub normal_impulse: f32,
    pub tangent_impulse: f32,
    pub source_pre_solver_cache: bool,
    pub source_tracked_geometric_contact: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ContactImpulseInput {
    pub impulse: TrackedContactImpulse,
    pub stress: StressInput,
}

#[derive(Clone, Debug, PartialEq)]
pub struct TrackedQuickImpact {
    pub pre_solver_pair_key: ((u32, u32), (u32, u32)),
    pub pre_solver_sequence: u64,
    pub mapping: ContactPairMapping,
    pub voxel: Option<crate::collider_sync::VoxelContact>,
    pub material: u16,
    pub estimate: QuickImpactEstimate,
}

#[derive(Clone, Debug, PartialEq)]
pub struct QuickImpactInput {
    pub impact: TrackedQuickImpact,
    pub stress: StressInput,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ContactImpulseReadbackMiss {
    pub pair_key: ContactPairKey,
    pub mapping: ContactPairMapping,
    pub destructible_subshape: u32,
    pub other_subshape: u32,
    pub manifold_index: usize,
    pub contact_index: usize,
    pub normal_impulse: f32,
    pub tangent_impulse: f32,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct ContactImpulseReadback {
    pub inputs: Vec<ContactImpulseInput>,
    pub cache_misses: Vec<ContactImpulseReadbackMiss>,
}

pub(crate) fn collect_contact_impulse_inputs(
    tick: u64,
    dt: f32,
    narrow_phase: &NarrowPhase,
    families: &[(FxFamilyId, &FxFamily)],
    registry: &ContactMaterialRegistry,
    pre_solver_cache: &[PreSolverContactMapping],
) -> ContactImpulseReadback {
    let cache = pre_solver_cache_index(pre_solver_cache);
    let mut cache_cursors = BTreeMap::<CacheMatchKey, usize>::new();
    let mut destructible_colliders = registry.collider_actors.keys().copied().collect::<Vec<_>>();
    destructible_colliders.sort_unstable();

    let mut seen = BTreeSet::new();
    let mut out = Vec::new();
    let mut misses = Vec::new();
    let mut command_id = 0u32;
    for key in destructible_colliders {
        let collider = ColliderHandle::from_raw_parts(key.0, key.1);
        for pair in narrow_phase.contact_pairs_with(collider) {
            if !seen.insert(stable_pair_key(pair)) {
                continue;
            }
            let mappings = map_contact_pair_destructibles(pair, registry);
            for (manifold_index, manifold) in pair.manifolds.iter().enumerate() {
                for mapping in mappings.iter().copied() {
                    let Some((_, family)) = families
                        .iter()
                        .find(|(id, _)| *id == mapping.destructible.family)
                    else {
                        continue;
                    };
                    let Some(actor) = family.actor(mapping.destructible.actor) else {
                        continue;
                    };
                    let (destructible_subshape, other_subshape) =
                        manifold_subshapes(mapping.side, manifold);
                    let match_key = CacheMatchKey {
                        pair: stable_pair_key(pair),
                        side: mapping.side,
                        destructible_collider: collider_key(mapping.destructible_collider),
                        other_collider: collider_key(mapping.other_collider),
                        destructible_subshape,
                        other_subshape,
                    };
                    let cached = consume_cache_entry(&cache, &mut cache_cursors, match_key);
                    if cached.is_none() {
                        collect_cache_misses(pair, manifold, mapping, manifold_index, &mut misses);
                        continue;
                    }
                    let cached = cached.expect("checked above");
                    if cached.quick_impact.is_some() {
                        continue;
                    }
                    let fallback_node = actor.owned_nodes.first().copied();
                    let Some(node) = cached.node.or(fallback_node) else {
                        continue;
                    };
                    let solver_contact_ids = cached
                        .solver_contact_ids
                        .iter()
                        .copied()
                        .collect::<BTreeSet<_>>();
                    if solver_contact_ids.is_empty() {
                        continue;
                    }
                    let normal = manifold.data.normal;
                    let normal_sign = match mapping.side {
                        ContactPairSide::Collider1 => 1.0,
                        ContactPairSide::Collider2 => -1.0,
                    };
                    let tangent = Vector::new(-normal.y, normal.x);
                    for (contact_index, contact) in manifold.points.iter().enumerate() {
                        if !solver_contact_ids.contains(&(contact_index as u32)) {
                            continue;
                        }
                        let normal_impulse = contact.data.impulse;
                        let tangent_impulse = contact.data.tangent_impulse[0];
                        if normal_impulse <= 0.0 && tangent_impulse == 0.0 {
                            continue;
                        }
                        let force = (normal * (normal_impulse * normal_sign)
                            + tangent * (tangent_impulse * normal_sign))
                            / dt.max(f32::EPSILON);
                        let stress = StressInput {
                            order_key: DeterministicOrderKey::new(
                                tick,
                                10,
                                mapping.destructible.family,
                                mapping.destructible.actor,
                                CommandId(command_id),
                            ),
                            actor: mapping.destructible.actor,
                            node,
                            force: Vec2::new(force.x, force.y),
                            source: DamageSource::ContactImpulse,
                        };
                        command_id += 1;
                        out.push(ContactImpulseInput {
                            impulse: TrackedContactImpulse {
                                pre_solver_pair_key: cached.stable_key.pair,
                                pre_solver_sequence: cached.stable_key.sequence,
                                mapping,
                                voxel: cached.voxel,
                                material: cached.material,
                                used_fallback: cached.used_fallback,
                                manifold_index,
                                contact_index,
                                normal_impulse,
                                tangent_impulse,
                                source_pre_solver_cache: true,
                                source_tracked_geometric_contact: true,
                            },
                            stress,
                        });
                    }
                }
            }
        }
    }
    out.sort_by_key(|input| {
        (
            input.stress.order_key,
            collider_key(input.impulse.mapping.destructible_collider),
            input.impulse.manifold_index,
            input.impulse.contact_index,
        )
    });
    ContactImpulseReadback {
        inputs: out,
        cache_misses: misses,
    }
}

pub(crate) fn collect_quick_impact_inputs(
    tick: u64,
    dt: f32,
    families: &[(FxFamilyId, &FxFamily)],
    pre_solver_cache: &[PreSolverContactMapping],
    stress_force_scale: f32,
) -> Vec<QuickImpactInput> {
    let mut out = Vec::new();
    let mut command_id = 0u32;
    let mut sorted = pre_solver_cache.iter().collect::<Vec<_>>();
    sorted.sort_by_key(|mapping| mapping.stable_key);
    for cached in sorted {
        let Some(estimate) = cached.quick_impact else {
            continue;
        };
        let Some((_, family)) = families
            .iter()
            .find(|(id, _)| *id == cached.pair.destructible.family)
        else {
            continue;
        };
        let Some(actor) = family.actor(cached.pair.destructible.actor) else {
            continue;
        };
        let fallback_node = actor.owned_nodes.first().copied();
        let Some(node) = cached.node.or(fallback_node) else {
            continue;
        };
        let force = estimate.impulse * (stress_force_scale / dt.max(f32::EPSILON));
        let stress = StressInput {
            order_key: DeterministicOrderKey::new(
                tick,
                5,
                cached.pair.destructible.family,
                cached.pair.destructible.actor,
                CommandId(command_id),
            ),
            actor: cached.pair.destructible.actor,
            node,
            force: Vec2::new(force.x, force.y),
            source: DamageSource::ContactImpulse,
        };
        command_id += 1;
        out.push(QuickImpactInput {
            impact: TrackedQuickImpact {
                pre_solver_pair_key: cached.stable_key.pair,
                pre_solver_sequence: cached.stable_key.sequence,
                mapping: cached.pair,
                voxel: cached.voxel,
                material: cached.material,
                estimate,
            },
            stress,
        });
    }
    out.sort_by_key(|input| {
        (
            input.stress.order_key,
            collider_key(input.impact.mapping.destructible_collider),
        )
    });
    out
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct CacheMatchKey {
    pair: ContactPairKey,
    side: ContactPairSide,
    destructible_collider: (u32, u32),
    other_collider: (u32, u32),
    destructible_subshape: u32,
    other_subshape: u32,
}

fn pre_solver_cache_index(
    pre_solver_cache: &[PreSolverContactMapping],
) -> BTreeMap<CacheMatchKey, Vec<&PreSolverContactMapping>> {
    let mut cache = BTreeMap::<CacheMatchKey, Vec<&PreSolverContactMapping>>::new();
    let mut sorted = pre_solver_cache.iter().collect::<Vec<_>>();
    sorted.sort_by_key(|mapping| mapping.stable_key);
    for mapping in sorted {
        cache
            .entry(CacheMatchKey {
                pair: mapping.stable_key.pair,
                side: mapping.pair.side,
                destructible_collider: collider_key(mapping.pair.destructible_collider),
                other_collider: collider_key(mapping.pair.other_collider),
                destructible_subshape: mapping.destructible_subshape,
                other_subshape: mapping.other_subshape,
            })
            .or_default()
            .push(mapping);
    }
    cache
}

fn consume_cache_entry<'a>(
    cache: &BTreeMap<CacheMatchKey, Vec<&'a PreSolverContactMapping>>,
    cursors: &mut BTreeMap<CacheMatchKey, usize>,
    key: CacheMatchKey,
) -> Option<&'a PreSolverContactMapping> {
    let cursor = cursors.entry(key).or_default();
    let entry = cache
        .get(&key)
        .and_then(|entries| entries.get(*cursor).copied());
    if entry.is_some() {
        *cursor += 1;
    }
    entry
}

fn manifold_subshapes(side: ContactPairSide, manifold: &ContactManifold) -> (u32, u32) {
    match side {
        ContactPairSide::Collider1 => (manifold.subshape1, manifold.subshape2),
        ContactPairSide::Collider2 => (manifold.subshape2, manifold.subshape1),
    }
}

fn collect_cache_misses(
    pair: &ContactPair,
    manifold: &ContactManifold,
    mapping: ContactPairMapping,
    manifold_index: usize,
    misses: &mut Vec<ContactImpulseReadbackMiss>,
) {
    let (destructible_subshape, other_subshape) = manifold_subshapes(mapping.side, manifold);
    for (contact_index, contact) in manifold.points.iter().enumerate() {
        let normal_impulse = contact.data.impulse;
        let tangent_impulse = contact.data.tangent_impulse[0];
        if normal_impulse <= 0.0 && tangent_impulse == 0.0 {
            continue;
        }
        misses.push(ContactImpulseReadbackMiss {
            pair_key: stable_pair_key(pair),
            mapping,
            destructible_subshape,
            other_subshape,
            manifold_index,
            contact_index,
            normal_impulse,
            tangent_impulse,
        });
    }
}
