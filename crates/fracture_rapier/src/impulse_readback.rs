use std::collections::BTreeSet;

use fracture_core::{
    CommandId, DamageSource, DeterministicOrderKey, FxFamily, FxFamilyId, StressInput, Vec2,
};
use rapier2d::prelude::*;

use crate::contact_map::{
    ContactPairMapping, ContactPairSide, collider_key, map_contact_pair_destructibles,
    map_contact_voxel, stable_pair_key,
};
use crate::hooks::ContactMaterialRegistry;

#[derive(Clone, Debug, PartialEq)]
pub struct TrackedContactImpulse {
    pub mapping: ContactPairMapping,
    pub voxel: Option<crate::collider_sync::VoxelContact>,
    pub used_fallback: bool,
    pub manifold_index: usize,
    pub contact_index: usize,
    pub normal_impulse: f32,
    pub tangent_impulse: f32,
    pub source_tracked_geometric_contact: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ContactImpulseInput {
    pub impulse: TrackedContactImpulse,
    pub stress: StressInput,
}

pub(crate) fn collect_contact_impulse_inputs(
    tick: u64,
    dt: f32,
    narrow_phase: &NarrowPhase,
    families: &[(FxFamilyId, &FxFamily)],
    registry: &ContactMaterialRegistry,
) -> Vec<ContactImpulseInput> {
    let mut destructible_colliders = registry.collider_actors.keys().copied().collect::<Vec<_>>();
    destructible_colliders.sort_unstable();

    let mut seen = BTreeSet::new();
    let mut out = Vec::new();
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
                    let voxel_mapping = map_contact_voxel(mapping, manifold, registry);
                    let fallback_node = actor.owned_nodes.first().copied();
                    let Some(node) = voxel_mapping
                        .voxel
                        .map(|voxel| voxel.node)
                        .or(fallback_node)
                    else {
                        continue;
                    };
                    let normal = manifold.data.normal;
                    let normal_sign = match mapping.side {
                        ContactPairSide::Collider1 => 1.0,
                        ContactPairSide::Collider2 => -1.0,
                    };
                    let tangent = Vector::new(-normal.y, normal.x);
                    for (contact_index, contact) in manifold.points.iter().enumerate() {
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
                                mapping,
                                voxel: voxel_mapping.voxel,
                                used_fallback: voxel_mapping.used_fallback,
                                manifold_index,
                                contact_index,
                                normal_impulse,
                                tangent_impulse,
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
    out
}
