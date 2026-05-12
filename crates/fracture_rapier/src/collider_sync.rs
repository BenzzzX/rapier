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
pub struct VoxelContact {
    pub coord: GridCoord,
    pub node: SupportNodeId,
    pub contact_material: u16,
    pub subshape: u32,
}

#[derive(Clone, Debug)]
pub(crate) struct ActorColliderBuild {
    pub builder: ColliderBuilder,
    pub voxels: Vec<VoxelContact>,
}

pub(crate) fn actor_collider_build(
    actor: &FxActor,
    asset: &AuthoredVoxelAsset,
    local_origin: Vec2,
    material: ContactMaterialProperties,
) -> Option<ActorColliderBuild> {
    let voxel_size = asset.core().voxel_size();
    let mut shapes = Vec::new();
    let mut voxels = Vec::new();

    for node_id in &actor.owned_nodes {
        let Some(node) = asset.core().node(*node_id) else {
            continue;
        };
        for coord in &node.voxels {
            let metadata = asset.voxel_metadata(*coord).ok()?;
            let center = coord.center(voxel_size);
            let offset = Vector::new(center.x - local_origin.x, center.y - local_origin.y);
            let subshape = shapes.len() as u32;
            shapes.push((
                Pose::from_translation(offset),
                SharedShape::cuboid(voxel_size * 0.5, voxel_size * 0.5),
            ));
            voxels.push(VoxelContact {
                coord: *coord,
                node: metadata.node?,
                contact_material: metadata.contact_material,
                subshape,
            });
        }
    }

    if shapes.is_empty() {
        return None;
    }

    Some(ActorColliderBuild {
        builder: ColliderBuilder::compound(shapes)
            .density(1.0)
            .friction(material.friction)
            .restitution(material.restitution)
            .active_hooks(ActiveHooks::MODIFY_SOLVER_CONTACTS),
        voxels,
    })
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
