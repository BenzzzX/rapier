use fracture_core::{FxActor, FxActorId, FxFamilyId, GridCoord, SupportNodeId, Vec2};
use fracture_voxel::AuthoredVoxelAsset;
use rapier2d::prelude::*;

use crate::hooks::ContactMaterialProperties;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct DestructibleActorRef {
    pub family: FxFamilyId,
    pub actor: FxActorId,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ActorPhysicsHandles {
    pub body: RigidBodyHandle,
    pub collider: ColliderHandle,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ImpulseJointHandleReplacement {
    pub old: ImpulseJointHandle,
    pub new: ImpulseJointHandle,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VoxelContact {
    pub coord: GridCoord,
    pub node: SupportNodeId,
    pub contact_material: u16,
    pub subshape: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ColliderLodSettings {
    pub enabled: bool,
    pub small_debris_max_voxels: usize,
    pub small_debris_max_nodes: usize,
}

impl Default for ColliderLodSettings {
    fn default() -> Self {
        Self {
            enabled: true,
            small_debris_max_voxels: 4,
            small_debris_max_nodes: 1,
        }
    }
}

impl ColliderLodSettings {
    pub fn disabled() -> Self {
        Self {
            enabled: false,
            ..Self::default()
        }
    }

    pub fn small_debris_box(max_voxels: usize, max_nodes: usize) -> Self {
        Self {
            enabled: true,
            small_debris_max_voxels: max_voxels,
            small_debris_max_nodes: max_nodes,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ActorColliderBuildKind {
    VoxelCompound,
    SmallDebrisPrimitive,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct FxPhysicsSyncReport {
    pub created_actor_bodies: usize,
    pub removed_actor_bodies: usize,
    pub rebuilt_colliders: usize,
    pub untouched_actor_count: usize,
    pub primitive_lod_replacements: usize,
    pub impulse_joint_handle_replacements: Vec<ImpulseJointHandleReplacement>,
}

impl FxPhysicsSyncReport {
    pub(crate) fn absorb(&mut self, other: Self) {
        self.created_actor_bodies += other.created_actor_bodies;
        self.removed_actor_bodies += other.removed_actor_bodies;
        self.rebuilt_colliders += other.rebuilt_colliders;
        self.untouched_actor_count += other.untouched_actor_count;
        self.primitive_lod_replacements += other.primitive_lod_replacements;
        self.impulse_joint_handle_replacements
            .extend(other.impulse_joint_handle_replacements);
    }

    pub(crate) fn record_kind(&mut self, kind: ActorColliderBuildKind) {
        if kind == ActorColliderBuildKind::SmallDebrisPrimitive {
            self.primitive_lod_replacements += 1;
        }
    }
}

#[derive(Clone, Debug)]
pub(crate) struct ActorColliderBuild {
    pub builder: ColliderBuilder,
    pub voxels: Vec<VoxelContact>,
    pub kind: ActorColliderBuildKind,
}

pub(crate) fn actor_collider_build(
    actor: &FxActor,
    asset: &AuthoredVoxelAsset,
    local_origin: Vec2,
    material: ContactMaterialProperties,
    lod: ColliderLodSettings,
) -> Option<ActorColliderBuild> {
    let voxel_size = asset.core().voxel_size();
    let mut occupied = Vec::new();
    for node_id in &actor.owned_nodes {
        let Some(node) = asset.core().node(*node_id) else {
            continue;
        };
        for coord in &node.voxels {
            let metadata = asset.voxel_metadata(*coord).ok()?;
            occupied.push(VoxelContact {
                coord: *coord,
                node: metadata.node?,
                contact_material: metadata.contact_material,
                subshape: 0,
            });
        }
    }

    if occupied.is_empty() {
        return None;
    }

    if is_small_debris_primitive(actor, occupied.len(), lod) {
        let first = occupied
            .iter()
            .min_by_key(|voxel| voxel.coord)
            .copied()
            .expect("occupied checked");
        let (min_x, max_x, min_y, max_y) = occupied_bounds(&occupied);
        let center = Vec2::new(
            ((min_x + max_x + 1) as f32 * 0.5) * voxel_size,
            ((min_y + max_y + 1) as f32 * 0.5) * voxel_size,
        );
        let half_extents = Vector::new(
            ((max_x - min_x + 1) as f32 * voxel_size) * 0.5,
            ((max_y - min_y + 1) as f32 * voxel_size) * 0.5,
        );
        let offset = Vector::new(center.x - local_origin.x, center.y - local_origin.y);
        return Some(ActorColliderBuild {
            builder: ColliderBuilder::cuboid(half_extents.x, half_extents.y)
                .translation(offset)
                .density(1.0)
                .friction(material.friction)
                .restitution(material.restitution)
                .active_hooks(
                    ActiveHooks::MODIFY_SOLVER_CONTACTS | ActiveHooks::FILTER_CONTACT_PAIRS,
                ),
            voxels: vec![first],
            kind: ActorColliderBuildKind::SmallDebrisPrimitive,
        });
    }

    let mut shapes = Vec::new();
    let mut voxels = Vec::new();
    for (subshape, mut voxel) in occupied.into_iter().enumerate() {
        let center = voxel.coord.center(voxel_size);
        let offset = Vector::new(center.x - local_origin.x, center.y - local_origin.y);
        shapes.push((
            Pose::from_translation(offset),
            SharedShape::cuboid(voxel_size * 0.5, voxel_size * 0.5),
        ));
        voxel.subshape = subshape as u32;
        voxels.push(voxel);
    }

    Some(ActorColliderBuild {
        builder: ColliderBuilder::compound(shapes)
            .density(1.0)
            .friction(material.friction)
            .restitution(material.restitution)
            .active_hooks(ActiveHooks::MODIFY_SOLVER_CONTACTS | ActiveHooks::FILTER_CONTACT_PAIRS),
        voxels,
        kind: ActorColliderBuildKind::VoxelCompound,
    })
}

fn is_small_debris_primitive(
    actor: &FxActor,
    occupied_voxels: usize,
    lod: ColliderLodSettings,
) -> bool {
    lod.enabled
        && lod.small_debris_max_nodes == 1
        && actor.owned_nodes.len() == 1
        && occupied_voxels <= lod.small_debris_max_voxels
}

fn occupied_bounds(voxels: &[VoxelContact]) -> (u32, u32, u32, u32) {
    let mut min_x = u32::MAX;
    let mut max_x = 0;
    let mut min_y = u32::MAX;
    let mut max_y = 0;
    for voxel in voxels {
        min_x = min_x.min(voxel.coord.x);
        max_x = max_x.max(voxel.coord.x);
        min_y = min_y.min(voxel.coord.y);
        max_y = max_y.max(voxel.coord.y);
    }
    (min_x, max_x, min_y, max_y)
}

pub(crate) fn actor_body_builder_at(actor: &FxActor, translation: Vector) -> RigidBodyBuilder {
    RigidBodyBuilder::dynamic()
        .translation(translation)
        .additional_mass(actor.mass.max(0.001))
}

pub(crate) fn actor_default_contact_material(
    actor: &FxActor,
    asset: &AuthoredVoxelAsset,
) -> Option<u16> {
    actor
        .owned_nodes
        .iter()
        .filter_map(|node| node_contact_material(*node, asset))
        .min()
}

fn node_contact_material(node: SupportNodeId, asset: &AuthoredVoxelAsset) -> Option<u16> {
    asset
        .node_summaries()
        .iter()
        .find(|summary| summary.node_id == node)
        .map(|summary| summary.contact_material_summary)
}
