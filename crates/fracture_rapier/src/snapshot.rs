use fracture_core::{
    BondId, CompressionDamageMode2D, ConnectionId, DamageSource, ExternalBondId, FxActorId,
    FxFamilyId, GridCoord, StressBaselineTarget2D, StressSettings, SupportNodeId,
    snapshot::SnapshotMode,
};
use rapier2d::prelude::*;
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::collider_sync::{DestructibleActorRef, VoxelContact};
use crate::connect_api::StaticAnchorBodyPolicy;
use crate::world::{FractureField2D, FractureFieldMode};

pub use fracture_core::snapshot::SnapshotMode as SnapshotReplayMode;

const MAGIC: [u8; 8] = *b"RFXSR2\0\0";
const VERSION: u16 = 7;
const HEADER_LEN: usize = 34;

#[derive(Error, Debug, Clone, PartialEq)]
pub enum FxRapierSnapshotError {
    #[error("snapshot magic is invalid")]
    InvalidMagic,
    #[error("snapshot version {0} is unsupported")]
    UnsupportedVersion(u16),
    #[error("snapshot mode {0} is unsupported")]
    UnsupportedMode(u8),
    #[error("snapshot flags {0:#x} contain unsupported bits")]
    UnsupportedFlags(u32),
    #[error("snapshot payload length mismatch")]
    PayloadLengthMismatch,
    #[error("snapshot payload checksum mismatch")]
    PayloadChecksumMismatch,
    #[error("snapshot ended while reading {0}")]
    UnexpectedEof(&'static str),
    #[error("snapshot has trailing bytes")]
    TrailingBytes,
    #[error("snapshot value {0} is invalid or non-finite")]
    InvalidValue(&'static str),
    #[error("snapshot state is inconsistent: {0}")]
    StateMismatch(&'static str),
    #[error(
        "deterministic replay snapshot mode requires fracture_rapier feature `deterministic-replay`"
    )]
    DeterministicReplayFeatureRequired,
    #[error("family {0:?} restored core asset does not match authored voxel asset")]
    AssetCoreMismatch(FxFamilyId),
    #[error(transparent)]
    Core(#[from] fracture_core::FxCoreSnapshotError),
    #[error(transparent)]
    Voxel(#[from] fracture_voxel::VoxelSnapshotError),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BodyTypeSnapshot {
    Dynamic,
    Fixed,
    KinematicPositionBased,
    KinematicVelocityBased,
}

impl BodyTypeSnapshot {
    pub(crate) fn from_body_type(value: RigidBodyType) -> Self {
        match value {
            RigidBodyType::Dynamic => Self::Dynamic,
            RigidBodyType::Fixed => Self::Fixed,
            RigidBodyType::KinematicPositionBased => Self::KinematicPositionBased,
            RigidBodyType::KinematicVelocityBased => Self::KinematicVelocityBased,
        }
    }

    pub(crate) fn to_body_type(self) -> RigidBodyType {
        match self {
            Self::Dynamic => RigidBodyType::Dynamic,
            Self::Fixed => RigidBodyType::Fixed,
            Self::KinematicPositionBased => RigidBodyType::KinematicPositionBased,
            Self::KinematicVelocityBased => RigidBodyType::KinematicVelocityBased,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct FxRapierFamilySnapshot {
    pub family: FxFamilyId,
    pub asset: fracture_voxel::AuthoredVoxelAssetSnapshot,
    pub core_family: Vec<u8>,
}

#[derive(Serialize)]
struct RapierOwnedStateRef<'a> {
    bodies: &'a RigidBodySet,
    colliders: &'a ColliderSet,
    impulse_joints: &'a ImpulseJointSet,
    multibody_joints: &'a MultibodyJointSet,
    islands: &'a IslandManager,
    broad_phase: &'a BroadPhaseBvh,
    narrow_phase: &'a NarrowPhase,
    ccd_solver: &'a CCDSolver,
}

#[derive(Deserialize)]
pub struct RapierOwnedState {
    pub bodies: RigidBodySet,
    pub colliders: ColliderSet,
    pub impulse_joints: ImpulseJointSet,
    pub multibody_joints: MultibodyJointSet,
    pub islands: IslandManager,
    pub broad_phase: BroadPhaseBvh,
    pub narrow_phase: NarrowPhase,
    pub ccd_solver: CCDSolver,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ActorPhysicsSnapshot {
    pub family: FxFamilyId,
    pub actor: FxActorId,
    pub body_handle: (u32, u32),
    pub collider_handle: (u32, u32),
    pub body_local_origin_in_asset: [f32; 2],
}

#[derive(Clone, Debug, PartialEq)]
pub struct BodyActorSnapshot {
    pub body_handle: (u32, u32),
    pub actor: DestructibleActorRef,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ColliderActorSnapshot {
    pub collider_handle: (u32, u32),
    pub actor: DestructibleActorRef,
}

#[derive(Clone, Debug, PartialEq)]
pub struct VoxelContactSnapshot {
    pub coord: GridCoord,
    pub node: SupportNodeId,
    pub contact_material: u16,
    pub subshape: u32,
}

impl From<VoxelContact> for VoxelContactSnapshot {
    fn from(value: VoxelContact) -> Self {
        Self {
            coord: value.coord,
            node: value.node,
            contact_material: value.contact_material,
            subshape: value.subshape,
        }
    }
}

impl From<VoxelContactSnapshot> for VoxelContact {
    fn from(value: VoxelContactSnapshot) -> Self {
        Self {
            coord: value.coord,
            node: value.node,
            contact_material: value.contact_material,
            subshape: value.subshape,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ColliderVoxelSnapshot {
    pub collider_handle: (u32, u32),
    pub voxels: Vec<VoxelContactSnapshot>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct StaticAnchorPolicySnapshot {
    pub family: FxFamilyId,
    pub bond: fracture_core::ExternalBondId,
    pub policy: StaticAnchorBodyPolicy,
}

#[derive(Clone, Debug, PartialEq)]
pub struct AppliedStaticAnchorPolicySnapshot {
    pub family: FxFamilyId,
    pub actor: FxActorId,
    pub policy: StaticAnchorBodyPolicy,
}

#[derive(Clone, Debug, PartialEq)]
pub struct BodyBaselineSnapshot {
    pub family: FxFamilyId,
    pub actor: FxActorId,
    pub body_type: BodyTypeSnapshot,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum PrestressBaselineTargetSnapshot {
    Bond(BondId),
    ExternalBond(ExternalBondId),
    Connection(ConnectionId),
}

impl From<StressBaselineTarget2D> for PrestressBaselineTargetSnapshot {
    fn from(value: StressBaselineTarget2D) -> Self {
        match value {
            StressBaselineTarget2D::Bond(id) => Self::Bond(id),
            StressBaselineTarget2D::ExternalBond(id) => Self::ExternalBond(id),
            StressBaselineTarget2D::Connection(id) => Self::Connection(id),
        }
    }
}

impl From<PrestressBaselineTargetSnapshot> for StressBaselineTarget2D {
    fn from(value: PrestressBaselineTargetSnapshot) -> Self {
        match value {
            PrestressBaselineTargetSnapshot::Bond(id) => Self::Bond(id),
            PrestressBaselineTargetSnapshot::ExternalBond(id) => Self::ExternalBond(id),
            PrestressBaselineTargetSnapshot::Connection(id) => Self::Connection(id),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PrestressBaselineEdgeSnapshot {
    pub target: PrestressBaselineTargetSnapshot,
    pub node_a_force: [f32; 2],
    pub node_b_force: [f32; 2],
}

#[derive(Clone, Debug, PartialEq)]
pub struct PrestressBaselineSnapshot {
    pub family: FxFamilyId,
    pub topology_signature: u64,
    pub gravity: [f32; 2],
    pub loads: Vec<PrestressBaselineEdgeSnapshot>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct FxRapierWorldSnapshot {
    pub mode: SnapshotMode,
    pub tick: u64,
    pub gravity: [f32; 2],
    pub integration: IntegrationSnapshot,
    pub stress: StressSettingsSnapshot,
    pub quick_impact_settings: crate::hooks::QuickImpactSettings,
    pub queued_fracture_fields: Vec<FractureField2D>,
    pub rapier_owned_state: Vec<u8>,
    pub contact_materials: Vec<(u16, crate::hooks::ContactMaterialProperties)>,
    pub material_impact_hardness: Vec<(u16, f32)>,
    pub families: Vec<FxRapierFamilySnapshot>,
    pub actor_physics: Vec<ActorPhysicsSnapshot>,
    pub body_actors: Vec<BodyActorSnapshot>,
    pub collider_actors: Vec<ColliderActorSnapshot>,
    pub collider_voxels: Vec<ColliderVoxelSnapshot>,
    pub static_anchor_policies: Vec<StaticAnchorPolicySnapshot>,
    pub applied_static_anchor_policies: Vec<AppliedStaticAnchorPolicySnapshot>,
    pub static_anchor_body_baselines: Vec<BodyBaselineSnapshot>,
    pub prestress_baselines: Vec<PrestressBaselineSnapshot>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IntegrationSnapshot {
    pub dt: f32,
    pub min_ccd_dt: f32,
    pub contact_softness_natural_frequency: f32,
    pub contact_softness_damping_ratio: f32,
    pub warmstart_coefficient: f32,
    pub length_unit: f32,
    pub normalized_allowed_linear_error: f32,
    pub normalized_max_corrective_velocity: f32,
    pub normalized_prediction_distance: f32,
    pub num_solver_iterations: usize,
    pub num_internal_pgs_iterations: usize,
    pub num_internal_stabilization_iterations: usize,
    pub min_island_size: usize,
    pub max_ccd_substeps: usize,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StressSettingsSnapshot {
    pub tension_limit_scale: f32,
    pub shear_limit_scale: f32,
    pub compression_limit_scale: f32,
    pub compression_damage_mode: CompressionDamageMode2D,
    pub damage_per_overload: f32,
    pub fracture_energy_budget: f32,
    pub beam_bending_moment_scale: f32,
    pub section_aggregation_max_bonds: u16,
    pub section_axis_dot_min: f32,
    pub max_fractures_per_frame: u16,
    pub max_iterations: u16,
    pub convergence_epsilon: f32,
    pub enable_gravity: bool,
}

impl From<StressSettings> for StressSettingsSnapshot {
    fn from(value: StressSettings) -> Self {
        Self {
            tension_limit_scale: value.tension_limit_scale,
            shear_limit_scale: value.shear_limit_scale,
            compression_limit_scale: value.compression_limit_scale,
            compression_damage_mode: value.compression_damage_mode,
            damage_per_overload: value.damage_per_overload,
            fracture_energy_budget: value.fracture_energy_budget,
            beam_bending_moment_scale: value.beam_bending_moment_scale,
            section_aggregation_max_bonds: value.section_aggregation_max_bonds,
            section_axis_dot_min: value.section_axis_dot_min,
            max_fractures_per_frame: value.max_fractures_per_frame,
            max_iterations: value.max_iterations,
            convergence_epsilon: value.convergence_epsilon,
            enable_gravity: value.enable_gravity,
        }
    }
}

impl From<StressSettingsSnapshot> for StressSettings {
    fn from(value: StressSettingsSnapshot) -> Self {
        Self {
            tension_limit_scale: value.tension_limit_scale,
            shear_limit_scale: value.shear_limit_scale,
            compression_limit_scale: value.compression_limit_scale,
            compression_damage_mode: value.compression_damage_mode,
            damage_per_overload: value.damage_per_overload,
            fracture_energy_budget: value.fracture_energy_budget,
            beam_bending_moment_scale: value.beam_bending_moment_scale,
            section_aggregation_max_bonds: value.section_aggregation_max_bonds,
            section_axis_dot_min: value.section_axis_dot_min,
            max_fractures_per_frame: value.max_fractures_per_frame,
            max_iterations: value.max_iterations,
            convergence_epsilon: value.convergence_epsilon,
            enable_gravity: value.enable_gravity,
        }
    }
}

pub fn encode_rapier_owned_state(
    bodies: &RigidBodySet,
    colliders: &ColliderSet,
    impulse_joints: &ImpulseJointSet,
    multibody_joints: &MultibodyJointSet,
    islands: &IslandManager,
    broad_phase: &BroadPhaseBvh,
    narrow_phase: &NarrowPhase,
    ccd_solver: &CCDSolver,
) -> Result<Vec<u8>, FxRapierSnapshotError> {
    bincode::serialize(&RapierOwnedStateRef {
        bodies,
        colliders,
        impulse_joints,
        multibody_joints,
        islands,
        broad_phase,
        narrow_phase,
        ccd_solver,
    })
    .map_err(|_| FxRapierSnapshotError::StateMismatch("rapier state serialization failed"))
}

pub fn decode_rapier_owned_state(bytes: &[u8]) -> Result<RapierOwnedState, FxRapierSnapshotError> {
    bincode::deserialize(bytes)
        .map_err(|_| FxRapierSnapshotError::StateMismatch("rapier state deserialization failed"))
}

pub fn encode_world_snapshot(snapshot: &FxRapierWorldSnapshot) -> Vec<u8> {
    let mut writer = Writer::new();
    writer.u64(snapshot.tick);
    write_vec2(&mut writer, snapshot.gravity).expect("finite gravity");
    write_integration(&mut writer, snapshot.integration).expect("finite integration");
    write_stress(&mut writer, snapshot.stress).expect("finite stress");
    write_quick_impact_settings(&mut writer, snapshot.quick_impact_settings)
        .expect("finite quick impact settings");
    writer
        .len(snapshot.queued_fracture_fields.len())
        .expect("fracture field count");
    for field in &snapshot.queued_fracture_fields {
        write_fracture_field(&mut writer, *field).expect("finite fracture field");
    }
    writer
        .bytes(&snapshot.rapier_owned_state)
        .expect("rapier state bytes");

    writer.len(snapshot.families.len()).expect("family count");
    for family in &snapshot.families {
        writer.u32(family.family.0);
        writer.bytes(&family.asset.bytes).expect("asset bytes");
        writer.bytes(&family.core_family).expect("family bytes");
    }

    writer
        .len(snapshot.actor_physics.len())
        .expect("actor physics count");
    for item in &snapshot.actor_physics {
        write_actor_ref(
            &mut writer,
            DestructibleActorRef {
                family: item.family,
                actor: item.actor,
            },
        );
        write_handle(&mut writer, item.body_handle);
        write_handle(&mut writer, item.collider_handle);
        write_vec2(&mut writer, item.body_local_origin_in_asset).expect("finite local origin");
    }

    writer
        .len(snapshot.body_actors.len())
        .expect("body actor count");
    for item in &snapshot.body_actors {
        write_handle(&mut writer, item.body_handle);
        write_actor_ref(&mut writer, item.actor);
    }

    writer
        .len(snapshot.collider_actors.len())
        .expect("collider actor count");
    for item in &snapshot.collider_actors {
        write_handle(&mut writer, item.collider_handle);
        write_actor_ref(&mut writer, item.actor);
    }

    writer
        .len(snapshot.collider_voxels.len())
        .expect("collider voxel count");
    for item in &snapshot.collider_voxels {
        write_handle(&mut writer, item.collider_handle);
        writer.len(item.voxels.len()).expect("voxel count");
        for voxel in &item.voxels {
            writer.u32(voxel.coord.x);
            writer.u32(voxel.coord.y);
            writer.u32(voxel.node.0);
            writer.u16(voxel.contact_material);
            writer.u32(voxel.subshape);
        }
    }

    writer
        .len(snapshot.static_anchor_policies.len())
        .expect("policy count");
    for item in &snapshot.static_anchor_policies {
        writer.u32(item.family.0);
        writer.u32(item.bond.0);
        write_static_policy(&mut writer, item.policy);
    }

    writer
        .len(snapshot.applied_static_anchor_policies.len())
        .expect("policy count");
    for item in &snapshot.applied_static_anchor_policies {
        writer.u32(item.family.0);
        writer.u32(item.actor.0);
        write_static_policy(&mut writer, item.policy);
    }

    writer
        .len(snapshot.static_anchor_body_baselines.len())
        .expect("baseline count");
    for item in &snapshot.static_anchor_body_baselines {
        writer.u32(item.family.0);
        writer.u32(item.actor.0);
        write_body_type(&mut writer, item.body_type);
    }

    writer
        .len(snapshot.prestress_baselines.len())
        .expect("prestress baseline count");
    for item in &snapshot.prestress_baselines {
        writer.u32(item.family.0);
        writer.u64(item.topology_signature);
        write_vec2(&mut writer, item.gravity).expect("finite prestress gravity");
        writer.len(item.loads.len()).expect("prestress load count");
        for load in &item.loads {
            write_prestress_target(&mut writer, load.target);
            write_vec2(&mut writer, load.node_a_force).expect("finite prestress load");
            write_vec2(&mut writer, load.node_b_force).expect("finite prestress load");
        }
    }

    writer
        .len(snapshot.contact_materials.len())
        .expect("material count");
    for (material, properties) in &snapshot.contact_materials {
        writer.u16(*material);
        writer.f32(properties.friction).expect("finite friction");
        writer
            .f32(properties.restitution)
            .expect("finite restitution");
    }

    writer
        .len(snapshot.material_impact_hardness.len())
        .expect("material hardness count");
    for (material, hardness) in &snapshot.material_impact_hardness {
        writer.u16(*material);
        writer.f32(*hardness).expect("finite material hardness");
    }

    wrap(snapshot.mode, writer.bytes)
}

pub fn decode_world_snapshot(
    bytes: &[u8],
) -> Result<(SnapshotMode, FxRapierWorldSnapshot), FxRapierSnapshotError> {
    let (mode, payload) = unwrap(bytes)?;
    let mut reader = Reader::new(payload);
    let tick = reader.u64("tick")?;
    let gravity = reader.vec2("gravity")?;
    let integration = read_integration(&mut reader)?;
    let stress = read_stress(&mut reader)?;
    let quick_impact_settings = read_quick_impact_settings(&mut reader)?;
    let fracture_field_count = reader.len("queued_fracture_fields")?;
    let mut queued_fracture_fields = Vec::with_capacity(fracture_field_count);
    for _ in 0..fracture_field_count {
        queued_fracture_fields.push(read_fracture_field(&mut reader)?);
    }
    let rapier_owned_state = reader.bytes("rapier_owned_state")?.to_vec();

    let family_count = reader.len("families")?;
    let mut families = Vec::with_capacity(family_count);
    for _ in 0..family_count {
        families.push(FxRapierFamilySnapshot {
            family: FxFamilyId(reader.u32("family.id")?),
            asset: fracture_voxel::AuthoredVoxelAssetSnapshot {
                bytes: reader.bytes("family.asset_bytes")?.to_vec(),
            },
            core_family: reader.bytes("family.family_bytes")?.to_vec(),
        });
    }

    let actor_physics_count = reader.len("actor_physics")?;
    let mut actor_physics = Vec::with_capacity(actor_physics_count);
    for _ in 0..actor_physics_count {
        let actor = read_actor_ref(&mut reader)?;
        actor_physics.push(ActorPhysicsSnapshot {
            family: actor.family,
            actor: actor.actor,
            body_handle: read_handle(&mut reader, "actor_physics.body")?,
            collider_handle: read_handle(&mut reader, "actor_physics.collider")?,
            body_local_origin_in_asset: reader.vec2("actor_physics.local_origin")?,
        });
    }

    let body_actor_count = reader.len("body_actors")?;
    let mut body_actors = Vec::with_capacity(body_actor_count);
    for _ in 0..body_actor_count {
        body_actors.push(BodyActorSnapshot {
            body_handle: read_handle(&mut reader, "body_actor.handle")?,
            actor: read_actor_ref(&mut reader)?,
        });
    }

    let collider_actor_count = reader.len("collider_actors")?;
    let mut collider_actors = Vec::with_capacity(collider_actor_count);
    for _ in 0..collider_actor_count {
        collider_actors.push(ColliderActorSnapshot {
            collider_handle: read_handle(&mut reader, "collider_actor.handle")?,
            actor: read_actor_ref(&mut reader)?,
        });
    }

    let collider_voxel_count = reader.len("collider_voxels")?;
    let mut collider_voxels = Vec::with_capacity(collider_voxel_count);
    for _ in 0..collider_voxel_count {
        let collider_handle = read_handle(&mut reader, "collider_voxel.handle")?;
        let voxel_count = reader.len("collider_voxel.voxels")?;
        let mut voxels = Vec::with_capacity(voxel_count);
        for _ in 0..voxel_count {
            voxels.push(VoxelContactSnapshot {
                coord: GridCoord {
                    x: reader.u32("voxel.coord.x")?,
                    y: reader.u32("voxel.coord.y")?,
                },
                node: SupportNodeId(reader.u32("voxel.node")?),
                contact_material: reader.u16("voxel.contact_material")?,
                subshape: reader.u32("voxel.subshape")?,
            });
        }
        collider_voxels.push(ColliderVoxelSnapshot {
            collider_handle,
            voxels,
        });
    }

    let static_policy_count = reader.len("static_anchor_policies")?;
    let mut static_anchor_policies = Vec::with_capacity(static_policy_count);
    for _ in 0..static_policy_count {
        static_anchor_policies.push(StaticAnchorPolicySnapshot {
            family: FxFamilyId(reader.u32("static_policy.family")?),
            bond: fracture_core::ExternalBondId(reader.u32("static_policy.bond")?),
            policy: read_static_policy(&mut reader)?,
        });
    }

    let applied_policy_count = reader.len("applied_static_anchor_policies")?;
    let mut applied_static_anchor_policies = Vec::with_capacity(applied_policy_count);
    for _ in 0..applied_policy_count {
        applied_static_anchor_policies.push(AppliedStaticAnchorPolicySnapshot {
            family: FxFamilyId(reader.u32("applied_policy.family")?),
            actor: FxActorId(reader.u32("applied_policy.actor")?),
            policy: read_static_policy(&mut reader)?,
        });
    }

    let baseline_count = reader.len("static_anchor_body_baselines")?;
    let mut static_anchor_body_baselines = Vec::with_capacity(baseline_count);
    for _ in 0..baseline_count {
        static_anchor_body_baselines.push(BodyBaselineSnapshot {
            family: FxFamilyId(reader.u32("baseline.family")?),
            actor: FxActorId(reader.u32("baseline.actor")?),
            body_type: read_body_type(&mut reader)?,
        });
    }

    let prestress_baseline_count = reader.len("prestress_baselines")?;
    let mut prestress_baselines = Vec::with_capacity(prestress_baseline_count);
    for _ in 0..prestress_baseline_count {
        let family = FxFamilyId(reader.u32("prestress_baseline.family")?);
        let topology_signature = reader.u64("prestress_baseline.topology_signature")?;
        let gravity = reader.vec2("prestress_baseline.gravity")?;
        let load_count = reader.len("prestress_baseline.loads")?;
        let mut loads = Vec::with_capacity(load_count);
        for _ in 0..load_count {
            loads.push(PrestressBaselineEdgeSnapshot {
                target: read_prestress_target(&mut reader)?,
                node_a_force: reader.vec2("prestress_baseline.node_a_force")?,
                node_b_force: reader.vec2("prestress_baseline.node_b_force")?,
            });
        }
        prestress_baselines.push(PrestressBaselineSnapshot {
            family,
            topology_signature,
            gravity,
            loads,
        });
    }

    let contact_material_count = reader.len("contact_material_properties")?;
    let mut contact_materials = Vec::with_capacity(contact_material_count);
    for _ in 0..contact_material_count {
        contact_materials.push((
            reader.u16("material.id")?,
            crate::hooks::ContactMaterialProperties {
                friction: reader.f32("material.friction")?,
                restitution: reader.f32("material.restitution")?,
            },
        ));
    }

    let material_hardness_count = reader.len("material_impact_hardness")?;
    let mut material_impact_hardness = Vec::with_capacity(material_hardness_count);
    for _ in 0..material_hardness_count {
        material_impact_hardness.push((
            reader.u16("material_hardness.id")?,
            reader.f32("material_hardness.value")?,
        ));
    }

    reader.finish()?;
    Ok((
        mode,
        FxRapierWorldSnapshot {
            mode,
            tick,
            gravity,
            integration,
            stress,
            quick_impact_settings,
            queued_fracture_fields,
            rapier_owned_state,
            contact_materials,
            material_impact_hardness,
            families,
            actor_physics,
            body_actors,
            collider_actors,
            collider_voxels,
            static_anchor_policies,
            applied_static_anchor_policies,
            static_anchor_body_baselines,
            prestress_baselines,
        },
    ))
}

fn write_integration(
    writer: &mut Writer,
    params: IntegrationSnapshot,
) -> Result<(), FxRapierSnapshotError> {
    writer.f32(params.dt)?;
    writer.f32(params.min_ccd_dt)?;
    writer.f32(params.contact_softness_natural_frequency)?;
    writer.f32(params.contact_softness_damping_ratio)?;
    writer.f32(params.warmstart_coefficient)?;
    writer.f32(params.length_unit)?;
    writer.f32(params.normalized_allowed_linear_error)?;
    writer.f32(params.normalized_max_corrective_velocity)?;
    writer.f32(params.normalized_prediction_distance)?;
    writer.len(params.num_solver_iterations)?;
    writer.len(params.num_internal_pgs_iterations)?;
    writer.len(params.num_internal_stabilization_iterations)?;
    writer.len(params.min_island_size)?;
    writer.len(params.max_ccd_substeps)?;
    Ok(())
}

fn read_integration(reader: &mut Reader<'_>) -> Result<IntegrationSnapshot, FxRapierSnapshotError> {
    Ok(IntegrationSnapshot {
        dt: reader.f32("integration.dt")?,
        min_ccd_dt: reader.f32("integration.min_ccd_dt")?,
        contact_softness_natural_frequency: reader
            .f32("integration.contact_softness_natural_frequency")?,
        contact_softness_damping_ratio: reader.f32("integration.contact_softness_damping_ratio")?,
        warmstart_coefficient: reader.f32("integration.warmstart_coefficient")?,
        length_unit: reader.f32("integration.length_unit")?,
        normalized_allowed_linear_error: reader
            .f32("integration.normalized_allowed_linear_error")?,
        normalized_max_corrective_velocity: reader
            .f32("integration.normalized_max_corrective_velocity")?,
        normalized_prediction_distance: reader.f32("integration.normalized_prediction_distance")?,
        num_solver_iterations: reader.len("integration.num_solver_iterations")?,
        num_internal_pgs_iterations: reader.len("integration.num_internal_pgs_iterations")?,
        num_internal_stabilization_iterations: reader
            .len("integration.num_internal_stabilization_iterations")?,
        min_island_size: reader.len("integration.min_island_size")?,
        max_ccd_substeps: reader.len("integration.max_ccd_substeps")?,
    })
}

fn write_stress(
    writer: &mut Writer,
    settings: StressSettingsSnapshot,
) -> Result<(), FxRapierSnapshotError> {
    writer.f32(settings.tension_limit_scale)?;
    writer.f32(settings.shear_limit_scale)?;
    writer.f32(settings.compression_limit_scale)?;
    write_compression_damage_mode(writer, settings.compression_damage_mode);
    writer.f32(settings.damage_per_overload)?;
    writer.f32(settings.fracture_energy_budget)?;
    writer.f32(settings.beam_bending_moment_scale)?;
    writer.u16(settings.section_aggregation_max_bonds);
    writer.f32(settings.section_axis_dot_min)?;
    writer.u16(settings.max_fractures_per_frame);
    writer.u16(settings.max_iterations);
    writer.f32(settings.convergence_epsilon)?;
    writer.u8(u8::from(settings.enable_gravity));
    Ok(())
}

fn read_stress(reader: &mut Reader<'_>) -> Result<StressSettingsSnapshot, FxRapierSnapshotError> {
    let settings = StressSettingsSnapshot {
        tension_limit_scale: reader.f32("stress.tension_limit_scale")?,
        shear_limit_scale: reader.f32("stress.shear_limit_scale")?,
        compression_limit_scale: reader.f32("stress.compression_limit_scale")?,
        compression_damage_mode: read_compression_damage_mode(reader)?,
        damage_per_overload: reader.f32("stress.damage_per_overload")?,
        fracture_energy_budget: reader.f32("stress.fracture_energy_budget")?,
        beam_bending_moment_scale: reader.f32("stress.beam_bending_moment_scale")?,
        section_aggregation_max_bonds: reader.u16("stress.section_aggregation_max_bonds")?,
        section_axis_dot_min: reader.f32("stress.section_axis_dot_min")?,
        max_fractures_per_frame: reader.u16("stress.max_fractures_per_frame")?,
        max_iterations: reader.u16("stress.max_iterations")?,
        convergence_epsilon: reader.f32("stress.convergence_epsilon")?,
        enable_gravity: match reader.u8("stress.enable_gravity")? {
            0 => false,
            1 => true,
            _ => return Err(FxRapierSnapshotError::InvalidValue("stress.enable_gravity")),
        },
    };
    Ok(settings)
}

fn write_quick_impact_settings(
    writer: &mut Writer,
    settings: crate::hooks::QuickImpactSettings,
) -> Result<(), FxRapierSnapshotError> {
    writer.u8(u8::from(settings.enabled));
    writer.u8(u8::from(settings.soften_enabled));
    writer.u8(u8::from(settings.suppress_enabled));
    writer.f32(settings.static_soften_impulse_threshold)?;
    writer.f32(settings.static_suppress_impulse_threshold)?;
    writer.f32(settings.dynamic_soften_impulse_threshold)?;
    writer.f32(settings.dynamic_suppress_impulse_threshold)?;
    writer.f32(settings.penetration_impulse_scale)?;
    writer.f32(settings.stress_force_scale)?;
    writer.f32(settings.softened_friction_scale)?;
    writer.f32(settings.softened_restitution_scale)?;
    writer.u16(settings.suppress_tunnel_window_frames);
    Ok(())
}

fn read_quick_impact_settings(
    reader: &mut Reader<'_>,
) -> Result<crate::hooks::QuickImpactSettings, FxRapierSnapshotError> {
    Ok(crate::hooks::QuickImpactSettings {
        enabled: read_bool(reader, "quick_impact.enabled")?,
        soften_enabled: read_bool(reader, "quick_impact.soften_enabled")?,
        suppress_enabled: read_bool(reader, "quick_impact.suppress_enabled")?,
        static_soften_impulse_threshold: reader.f32("quick_impact.static_soften_threshold")?,
        static_suppress_impulse_threshold: reader.f32("quick_impact.static_suppress_threshold")?,
        dynamic_soften_impulse_threshold: reader.f32("quick_impact.dynamic_soften_threshold")?,
        dynamic_suppress_impulse_threshold: reader
            .f32("quick_impact.dynamic_suppress_threshold")?,
        penetration_impulse_scale: reader.f32("quick_impact.penetration_impulse_scale")?,
        stress_force_scale: reader.f32("quick_impact.stress_force_scale")?,
        softened_friction_scale: reader.f32("quick_impact.softened_friction_scale")?,
        softened_restitution_scale: reader.f32("quick_impact.softened_restitution_scale")?,
        suppress_tunnel_window_frames: reader.u16("quick_impact.suppress_tunnel_window_frames")?,
    })
}

fn write_fracture_field(
    writer: &mut Writer,
    field: FractureField2D,
) -> Result<(), FxRapierSnapshotError> {
    match field.family {
        Some(family) => {
            writer.u8(1);
            writer.u32(family.0);
        }
        None => writer.u8(0),
    }
    writer.f32(field.center.x)?;
    writer.f32(field.center.y)?;
    writer.f32(field.radius)?;
    writer.f32(field.force.x)?;
    writer.f32(field.force.y)?;
    writer.f32(field.health_loss)?;
    writer.f32(field.effective_length_loss)?;
    write_fracture_field_mode(writer, field.mode);
    write_damage_source(writer, field.source);
    Ok(())
}

fn read_fracture_field(reader: &mut Reader<'_>) -> Result<FractureField2D, FxRapierSnapshotError> {
    let family = match reader.u8("queued_fracture_field.family_present")? {
        0 => None,
        1 => Some(FxFamilyId(reader.u32("queued_fracture_field.family")?)),
        _ => {
            return Err(FxRapierSnapshotError::InvalidValue(
                "queued fracture field family",
            ));
        }
    };
    Ok(FractureField2D {
        family,
        center: fracture_core::Vec2::new(
            reader.f32("queued_fracture_field.center.x")?,
            reader.f32("queued_fracture_field.center.y")?,
        ),
        radius: reader.f32("queued_fracture_field.radius")?,
        force: fracture_core::Vec2::new(
            reader.f32("queued_fracture_field.force.x")?,
            reader.f32("queued_fracture_field.force.y")?,
        ),
        health_loss: reader.f32("queued_fracture_field.health_loss")?,
        effective_length_loss: reader.f32("queued_fracture_field.effective_length_loss")?,
        mode: read_fracture_field_mode(reader)?,
        source: read_damage_source(reader)?,
    })
}

fn write_fracture_field_mode(writer: &mut Writer, mode: FractureFieldMode) {
    writer.u8(match mode {
        FractureFieldMode::Stress => 0,
        FractureFieldMode::DirectDamage => 1,
    });
}

fn read_fracture_field_mode(
    reader: &mut Reader<'_>,
) -> Result<FractureFieldMode, FxRapierSnapshotError> {
    match reader.u8("queued_fracture_field.mode")? {
        0 => Ok(FractureFieldMode::Stress),
        1 => Ok(FractureFieldMode::DirectDamage),
        _ => Err(FxRapierSnapshotError::InvalidValue(
            "queued fracture field mode",
        )),
    }
}

fn write_damage_source(writer: &mut Writer, source: DamageSource) {
    writer.u8(match source {
        DamageSource::Script => 0,
        DamageSource::ContactImpulse => 1,
        DamageSource::JointFeedback => 2,
        DamageSource::Stress => 3,
    });
}

fn read_damage_source(reader: &mut Reader<'_>) -> Result<DamageSource, FxRapierSnapshotError> {
    match reader.u8("queued_fracture_field.source")? {
        0 => Ok(DamageSource::Script),
        1 => Ok(DamageSource::ContactImpulse),
        2 => Ok(DamageSource::JointFeedback),
        3 => Ok(DamageSource::Stress),
        _ => Err(FxRapierSnapshotError::InvalidValue(
            "queued fracture field source",
        )),
    }
}

fn read_bool(reader: &mut Reader<'_>, field: &'static str) -> Result<bool, FxRapierSnapshotError> {
    match reader.u8(field)? {
        0 => Ok(false),
        1 => Ok(true),
        _ => Err(FxRapierSnapshotError::InvalidValue(field)),
    }
}

fn write_compression_damage_mode(writer: &mut Writer, mode: CompressionDamageMode2D) {
    writer.u8(match mode {
        CompressionDamageMode2D::Ignore => 0,
        CompressionDamageMode2D::DamageOnly => 1,
        CompressionDamageMode2D::Break => 2,
    });
}

fn read_compression_damage_mode(
    reader: &mut Reader<'_>,
) -> Result<CompressionDamageMode2D, FxRapierSnapshotError> {
    match reader.u8("stress.compression_damage_mode")? {
        0 => Ok(CompressionDamageMode2D::Ignore),
        1 => Ok(CompressionDamageMode2D::DamageOnly),
        2 => Ok(CompressionDamageMode2D::Break),
        _ => Err(FxRapierSnapshotError::InvalidValue(
            "stress.compression_damage_mode",
        )),
    }
}

fn write_handle(writer: &mut Writer, handle: (u32, u32)) {
    writer.u32(handle.0);
    writer.u32(handle.1);
}

fn read_handle(
    reader: &mut Reader<'_>,
    field: &'static str,
) -> Result<(u32, u32), FxRapierSnapshotError> {
    Ok((reader.u32(field)?, reader.u32(field)?))
}

fn write_actor_ref(writer: &mut Writer, actor: DestructibleActorRef) {
    writer.u32(actor.family.0);
    writer.u32(actor.actor.0);
}

fn read_actor_ref(reader: &mut Reader<'_>) -> Result<DestructibleActorRef, FxRapierSnapshotError> {
    Ok(DestructibleActorRef {
        family: FxFamilyId(reader.u32("actor_ref.family")?),
        actor: FxActorId(reader.u32("actor_ref.actor")?),
    })
}

fn write_static_policy(writer: &mut Writer, policy: StaticAnchorBodyPolicy) {
    writer.u8(match policy {
        StaticAnchorBodyPolicy::Preserve => 0,
        StaticAnchorBodyPolicy::Fixed => 1,
        StaticAnchorBodyPolicy::KinematicVelocityBased => 2,
    });
}

fn read_static_policy(
    reader: &mut Reader<'_>,
) -> Result<StaticAnchorBodyPolicy, FxRapierSnapshotError> {
    match reader.u8("static_anchor_policy")? {
        0 => Ok(StaticAnchorBodyPolicy::Preserve),
        1 => Ok(StaticAnchorBodyPolicy::Fixed),
        2 => Ok(StaticAnchorBodyPolicy::KinematicVelocityBased),
        _ => Err(FxRapierSnapshotError::InvalidValue("static_anchor_policy")),
    }
}

fn write_body_type(writer: &mut Writer, body_type: BodyTypeSnapshot) {
    writer.u8(match body_type {
        BodyTypeSnapshot::Dynamic => 0,
        BodyTypeSnapshot::Fixed => 1,
        BodyTypeSnapshot::KinematicPositionBased => 2,
        BodyTypeSnapshot::KinematicVelocityBased => 3,
    });
}

fn read_body_type(reader: &mut Reader<'_>) -> Result<BodyTypeSnapshot, FxRapierSnapshotError> {
    match reader.u8("body_type")? {
        0 => Ok(BodyTypeSnapshot::Dynamic),
        1 => Ok(BodyTypeSnapshot::Fixed),
        2 => Ok(BodyTypeSnapshot::KinematicPositionBased),
        3 => Ok(BodyTypeSnapshot::KinematicVelocityBased),
        _ => Err(FxRapierSnapshotError::InvalidValue("body_type")),
    }
}

fn write_prestress_target(writer: &mut Writer, target: PrestressBaselineTargetSnapshot) {
    match target {
        PrestressBaselineTargetSnapshot::Bond(id) => {
            writer.u8(0);
            writer.u32(id.0);
        }
        PrestressBaselineTargetSnapshot::ExternalBond(id) => {
            writer.u8(1);
            writer.u32(id.0);
        }
        PrestressBaselineTargetSnapshot::Connection(id) => {
            writer.u8(2);
            writer.u32(id.0);
        }
    }
}

fn read_prestress_target(
    reader: &mut Reader<'_>,
) -> Result<PrestressBaselineTargetSnapshot, FxRapierSnapshotError> {
    let kind = reader.u8("prestress_baseline.target_kind")?;
    let id = reader.u32("prestress_baseline.target_id")?;
    match kind {
        0 => Ok(PrestressBaselineTargetSnapshot::Bond(BondId(id))),
        1 => Ok(PrestressBaselineTargetSnapshot::ExternalBond(
            ExternalBondId(id),
        )),
        2 => Ok(PrestressBaselineTargetSnapshot::Connection(ConnectionId(
            id,
        ))),
        _ => Err(FxRapierSnapshotError::InvalidValue(
            "prestress_baseline.target_kind",
        )),
    }
}

fn write_vec2(writer: &mut Writer, value: [f32; 2]) -> Result<(), FxRapierSnapshotError> {
    writer.f32(value[0])?;
    writer.f32(value[1])?;
    Ok(())
}

fn wrap(mode: SnapshotMode, payload: Vec<u8>) -> Vec<u8> {
    let mut out = Vec::with_capacity(HEADER_LEN + payload.len());
    out.extend_from_slice(&MAGIC);
    out.extend_from_slice(&VERSION.to_le_bytes());
    out.push(2);
    out.push(4);
    out.push(mode as u8);
    out.push(0);
    out.extend_from_slice(&0u32.to_le_bytes());
    out.extend_from_slice(&(payload.len() as u64).to_le_bytes());
    out.extend_from_slice(&checksum(&payload).to_le_bytes());
    out.extend_from_slice(&payload);
    out
}

fn unwrap(bytes: &[u8]) -> Result<(SnapshotMode, &[u8]), FxRapierSnapshotError> {
    if bytes.len() < HEADER_LEN {
        return Err(FxRapierSnapshotError::UnexpectedEof("header"));
    }
    if bytes[0..8] != MAGIC {
        return Err(FxRapierSnapshotError::InvalidMagic);
    }
    let version = u16::from_le_bytes([bytes[8], bytes[9]]);
    if version != VERSION {
        return Err(FxRapierSnapshotError::UnsupportedVersion(version));
    }
    if bytes[10] != 2 || bytes[11] != 4 {
        return Err(FxRapierSnapshotError::InvalidValue("header"));
    }
    let mode = match bytes[12] {
        0 => SnapshotMode::Normal,
        1 => SnapshotMode::Deterministic,
        other => return Err(FxRapierSnapshotError::UnsupportedMode(other)),
    };
    if bytes[13] != 0 {
        return Err(FxRapierSnapshotError::UnsupportedFlags(bytes[13] as u32));
    }
    let flags = u32::from_le_bytes([bytes[14], bytes[15], bytes[16], bytes[17]]);
    if flags != 0 {
        return Err(FxRapierSnapshotError::UnsupportedFlags(flags));
    }
    let len_u64 = u64::from_le_bytes([
        bytes[18], bytes[19], bytes[20], bytes[21], bytes[22], bytes[23], bytes[24], bytes[25],
    ]);
    let len = usize::try_from(len_u64).map_err(|_| FxRapierSnapshotError::PayloadLengthMismatch)?;
    let total = HEADER_LEN
        .checked_add(len)
        .ok_or(FxRapierSnapshotError::PayloadLengthMismatch)?;
    if bytes.len() != total {
        return Err(FxRapierSnapshotError::PayloadLengthMismatch);
    }
    let expected = u64::from_le_bytes([
        bytes[26], bytes[27], bytes[28], bytes[29], bytes[30], bytes[31], bytes[32], bytes[33],
    ]);
    let payload = &bytes[HEADER_LEN..];
    if checksum(payload) != expected {
        return Err(FxRapierSnapshotError::PayloadChecksumMismatch);
    }
    Ok((mode, payload))
}

struct Writer {
    bytes: Vec<u8>,
}

impl Writer {
    fn new() -> Self {
        Self { bytes: Vec::new() }
    }

    fn u8(&mut self, value: u8) {
        self.bytes.push(value);
    }

    fn u16(&mut self, value: u16) {
        self.bytes.extend_from_slice(&value.to_le_bytes());
    }

    fn u32(&mut self, value: u32) {
        self.bytes.extend_from_slice(&value.to_le_bytes());
    }

    fn u64(&mut self, value: u64) {
        self.bytes.extend_from_slice(&value.to_le_bytes());
    }

    fn f32(&mut self, value: f32) -> Result<(), FxRapierSnapshotError> {
        if !value.is_finite() {
            return Err(FxRapierSnapshotError::InvalidValue("f32"));
        }
        self.u32(value.to_bits());
        Ok(())
    }

    fn len(&mut self, len: usize) -> Result<(), FxRapierSnapshotError> {
        let len = u32::try_from(len).map_err(|_| FxRapierSnapshotError::InvalidValue("length"))?;
        self.u32(len);
        Ok(())
    }

    fn bytes(&mut self, bytes: &[u8]) -> Result<(), FxRapierSnapshotError> {
        self.len(bytes.len())?;
        self.bytes.extend_from_slice(bytes);
        Ok(())
    }
}

struct Reader<'a> {
    bytes: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, pos: 0 }
    }

    fn finish(&self) -> Result<(), FxRapierSnapshotError> {
        if self.pos == self.bytes.len() {
            Ok(())
        } else {
            Err(FxRapierSnapshotError::TrailingBytes)
        }
    }

    fn u8(&mut self, field: &'static str) -> Result<u8, FxRapierSnapshotError> {
        Ok(self.take(1, field)?[0])
    }

    fn u16(&mut self, field: &'static str) -> Result<u16, FxRapierSnapshotError> {
        let bytes = self.take(2, field)?;
        Ok(u16::from_le_bytes([bytes[0], bytes[1]]))
    }

    fn u32(&mut self, field: &'static str) -> Result<u32, FxRapierSnapshotError> {
        let bytes = self.take(4, field)?;
        Ok(u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
    }

    fn u64(&mut self, field: &'static str) -> Result<u64, FxRapierSnapshotError> {
        let bytes = self.take(8, field)?;
        Ok(u64::from_le_bytes([
            bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7],
        ]))
    }

    fn f32(&mut self, field: &'static str) -> Result<f32, FxRapierSnapshotError> {
        let value = f32::from_bits(self.u32(field)?);
        if value.is_finite() {
            Ok(value)
        } else {
            Err(FxRapierSnapshotError::InvalidValue(field))
        }
    }

    fn vec2(&mut self, field: &'static str) -> Result<[f32; 2], FxRapierSnapshotError> {
        Ok([self.f32(field)?, self.f32(field)?])
    }

    fn len(&mut self, field: &'static str) -> Result<usize, FxRapierSnapshotError> {
        Ok(self.u32(field)? as usize)
    }

    fn bytes(&mut self, field: &'static str) -> Result<&'a [u8], FxRapierSnapshotError> {
        let len = self.len(field)?;
        self.take(len, field)
    }

    fn take(&mut self, len: usize, field: &'static str) -> Result<&'a [u8], FxRapierSnapshotError> {
        let end = self
            .pos
            .checked_add(len)
            .ok_or(FxRapierSnapshotError::UnexpectedEof(field))?;
        if end > self.bytes.len() {
            return Err(FxRapierSnapshotError::UnexpectedEof(field));
        }
        let out = &self.bytes[self.pos..end];
        self.pos = end;
        Ok(out)
    }
}

fn checksum(bytes: &[u8]) -> u64 {
    let mut hash = 0xcbf2_9ce4_8422_2325u64;
    for byte in bytes {
        hash ^= u64::from(*byte);
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    }
    hash
}
