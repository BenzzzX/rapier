use rapier2d::prelude::*;

use crate::collider_sync::{DestructibleActorRef, VoxelContact};
use crate::hooks::ContactMaterialRegistry;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ContactPairSide {
    Collider1,
    Collider2,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ContactPairMapping {
    pub destructible: DestructibleActorRef,
    pub destructible_collider: ColliderHandle,
    pub other_collider: ColliderHandle,
    pub side: ContactPairSide,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct ContactVoxelMapping {
    pub pair: ContactPairMapping,
    pub voxel: Option<VoxelContact>,
    pub used_fallback: bool,
}

pub fn collider_key(handle: ColliderHandle) -> (u32, u32) {
    handle.into_raw_parts()
}

pub fn rigid_body_key(handle: RigidBodyHandle) -> (u32, u32) {
    handle.into_raw_parts()
}

#[cfg(test)]
pub(crate) fn map_contact_pair(
    pair: &ContactPair,
    registry: &ContactMaterialRegistry,
) -> Option<ContactPairMapping> {
    map_contact_pair_destructibles(pair, registry)
        .into_iter()
        .next()
}

pub(crate) fn map_contact_pair_destructibles(
    pair: &ContactPair,
    registry: &ContactMaterialRegistry,
) -> Vec<ContactPairMapping> {
    let c1 = registry
        .collider_actors
        .get(&collider_key(pair.collider1))
        .copied();
    let c2 = registry
        .collider_actors
        .get(&collider_key(pair.collider2))
        .copied();
    let mut out = Vec::new();
    if let Some(destructible) = c1 {
        out.push(ContactPairMapping {
            destructible,
            destructible_collider: pair.collider1,
            other_collider: pair.collider2,
            side: ContactPairSide::Collider1,
        });
    }
    if let Some(destructible) = c2 {
        out.push(ContactPairMapping {
            destructible,
            destructible_collider: pair.collider2,
            other_collider: pair.collider1,
            side: ContactPairSide::Collider2,
        });
    }
    out
}

pub(crate) fn map_contact_voxel(
    mapping: ContactPairMapping,
    manifold: &ContactManifold,
    registry: &ContactMaterialRegistry,
) -> ContactVoxelMapping {
    let subshape = match mapping.side {
        ContactPairSide::Collider1 => manifold.subshape1,
        ContactPairSide::Collider2 => manifold.subshape2,
    };
    let voxel = registry
        .collider_voxels
        .get(&collider_key(mapping.destructible_collider))
        .and_then(|voxels| voxels.iter().find(|voxel| voxel.subshape == subshape))
        .copied();
    ContactVoxelMapping {
        pair: mapping,
        voxel,
        used_fallback: voxel.is_none(),
    }
}

pub(crate) fn stable_pair_key(pair: &ContactPair) -> ((u32, u32), (u32, u32)) {
    let a = collider_key(pair.collider1);
    let b = collider_key(pair.collider2);
    if a <= b { (a, b) } else { (b, a) }
}
