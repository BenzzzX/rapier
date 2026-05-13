use rapier2d::prelude::*;

use crate::collider_sync::{DestructibleActorRef, VoxelContact};
use crate::hooks::ContactMaterialRegistry;

pub type ContactPairKey = ((u32, u32), (u32, u32));

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
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

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct PreSolverContactKey {
    pub pair: ContactPairKey,
    pub sequence: u64,
    pub side: ContactPairSide,
    pub destructible_subshape: u32,
    pub other_subshape: u32,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PreSolverContactMapping {
    pub stable_key: PreSolverContactKey,
    pub pair: ContactPairMapping,
    pub destructible_subshape: u32,
    pub other_subshape: u32,
    pub material: u16,
    pub voxel: Option<VoxelContact>,
    pub node: Option<fracture_core::SupportNodeId>,
    pub used_fallback: bool,
    pub solver_contact_count: usize,
    pub solver_contact_ids: Vec<u32>,
}

pub fn collider_key(handle: ColliderHandle) -> (u32, u32) {
    handle.into_raw_parts()
}

pub fn rigid_body_key(handle: RigidBodyHandle) -> (u32, u32) {
    handle.into_raw_parts()
}

pub(crate) fn contact_pair_key(
    collider1: ColliderHandle,
    collider2: ColliderHandle,
) -> ContactPairKey {
    let a = collider_key(collider1);
    let b = collider_key(collider2);
    if a <= b { (a, b) } else { (b, a) }
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

pub(crate) fn stable_pair_key(pair: &ContactPair) -> ContactPairKey {
    contact_pair_key(pair.collider1, pair.collider2)
}
