use fracture_core::{
    DamageSource, FxActorId, FxFamilyId, GridCoord, SplitEvent, Vec2, snapshot::SnapshotMode,
};
use fracture_rapier::{
    FractureField2D, FxActorBodyType, FxFamilyBodyMode, FxFamilyDeltaKind, FxRapierError,
    FxRapierWorld2D, FxStepWithDiagnostics,
};
use fracture_voxel::{
    AuthoredVoxelAsset, RuntimeEdit, VoxelAuthoringInput, VoxelRuntime, author_voxel_asset,
};
use rapier2d::parry::query::ShapeCastOptions;
use rapier2d::prelude::*;
use std::collections::{HashMap, HashSet};
use std::os::raw::{c_char, c_void};
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::ptr;
use std::slice;

const QUERY_SOURCE_TERRAIN: u32 = 1;
const QUERY_SOURCE_DYNAMIC_RIGIDBODY: u32 = 1 << 1;
const INVALID_SOURCE_CELL_ID: u32 = u32::MAX;
const SELF_COLLISION_FILTER_TAG: u128 = 1 << 127;
const MAX_RAGDOLL_MOTOR_BATCH_COUNT: usize = 128;

#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AlchemyRapierStatus {
    Ok = 0,
    NullPointer = 1,
    Panic = 2,
    InvalidHandle = 3,
    InvalidArgument = 4,
    Unsupported = 5,
}

impl Default for AlchemyRapierStatus {
    fn default() -> Self {
        Self::Ok
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AlchemyRapierBodyType {
    Dynamic = 0,
    Kinematic = 1,
    Fixed = 2,
    KinematicPosition = 3,
}

impl Default for AlchemyRapierBodyType {
    fn default() -> Self {
        Self::Dynamic
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AlchemyRapierQuerySourceKind {
    Unknown = 0,
    StaticTerrain = 1,
    DynamicPixelRigidbody = 2,
}

impl Default for AlchemyRapierQuerySourceKind {
    fn default() -> Self {
        Self::Unknown
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AlchemyFxSnapshotMode {
    Normal = 0,
    Deterministic = 1,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AlchemyFxDamageSource {
    Script = 0,
    ContactImpulse = 1,
    JointFeedback = 2,
    Stress = 3,
}

impl Default for AlchemyFxDamageSource {
    fn default() -> Self {
        Self::Script
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AlchemyFxFractureFieldMode {
    Stress = 0,
    DirectDamage = 1,
}

impl Default for AlchemyFxFractureFieldMode {
    fn default() -> Self {
        Self::Stress
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum AlchemyFxFamilyBodyMode {
    #[default]
    Unknown = 0,
    Dynamic = 1,
    Fixed = 2,
    KinematicVelocityBased = 3,
    KinematicPositionBased = 4,
    AttachedStatic = 5,
    Destroyed = 6,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum AlchemyFxFamilyDeltaKind {
    #[default]
    Created = 0,
    Updated = 1,
    Destroyed = 2,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AlchemyFxVoxelDestructibleDesc {
    pub family_id: u32,
    pub width: u32,
    pub height: u32,
    pub voxel_size: f32,
    pub occupancy_words: *const u64,
    pub occupancy_word_count: usize,
    pub fracture_material_ids: *const u16,
    pub fracture_material_id_count: usize,
    pub contact_material_ids: *const u16,
    pub contact_material_id_count: usize,
    pub external_ids: *const u32,
    pub external_id_count: usize,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AlchemyFxFractureFieldDesc {
    pub mode: AlchemyFxFractureFieldMode,
    pub has_family: u8,
    pub family_id: u32,
    pub center: AlchemyRapierVec2,
    pub radius: f32,
    pub force: AlchemyRapierVec2,
    pub health_loss: f32,
    pub effective_length_loss: f32,
    pub source: AlchemyFxDamageSource,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AlchemyFxFamilyStateResult {
    pub status: AlchemyRapierStatus,
    pub family_id: u32,
    pub actor_count: usize,
    pub body_count: usize,
    pub collider_count: usize,
    pub body_mode: AlchemyFxFamilyBodyMode,
    pub body_handle_index: u32,
    pub body_handle_generation: u32,
    pub body_packed_id: u64,
    pub collider_handle_index: u32,
    pub collider_handle_generation: u32,
    pub collider_packed_id: u64,
    pub has_single_body: u8,
    pub has_single_collider: u8,
    pub occupied_node_count: usize,
    pub occupied_voxel_count: usize,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AlchemyFxStepReport {
    pub status: AlchemyRapierStatus,
    pub tick: u64,
    pub quick_impact_count: usize,
    pub contact_impulse_count: usize,
    pub joint_feedback_count: usize,
    pub fracture_field_effect_count: usize,
    pub stress_input_count: usize,
    pub fracture_event_count: usize,
    pub split_event_count: usize,
    pub impulse_joint_handle_replacement_count: usize,
    pub occupied_voxel_count: usize,
    pub occupied_voxel_budget: usize,
    pub active_body_count: usize,
    pub active_body_budget: usize,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AlchemyFxSplitEventReadResult {
    pub status: AlchemyRapierStatus,
    pub row_count: usize,
    pub written_count: usize,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AlchemyFxSplitEventRow {
    pub event_id: u32,
    pub family_id: u32,
    pub internal_split_child_count: usize,
    pub fragment_count: usize,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AlchemyFxFamilyDeltaReadResult {
    pub status: AlchemyRapierStatus,
    pub row_count: usize,
    pub written_count: usize,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AlchemyFxFamilyDeltaRow {
    pub delta_sequence: u64,
    pub delta_id: u64,
    pub kind: AlchemyFxFamilyDeltaKind,
    pub family_id: u32,
    pub parent_family_id: u32,
    pub body_mode: AlchemyFxFamilyBodyMode,
    pub occupied_voxel_count: usize,
    pub actor_count: usize,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AlchemyFxByteSlice {
    pub data: *const u8,
    pub len: usize,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub enum AlchemyRapierCoefficientCombineRule {
    #[default]
    Average = 0,
    Min = 1,
    Multiply = 2,
    Max = 3,
    ClampedSum = 4,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AlchemyRapierVec2 {
    pub x: f32,
    pub y: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AlchemyRapierIVec2 {
    pub x: i32,
    pub y: i32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AlchemyRapierContactMaterialDesc {
    pub material_id: u16,
    pub friction: f32,
    pub restitution: f32,
    pub friction_combine_rule: AlchemyRapierCoefficientCombineRule,
    pub restitution_combine_rule: AlchemyRapierCoefficientCombineRule,
    pub hardness: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AlchemyRapierVoxelCell {
    pub coord: AlchemyRapierIVec2,
    pub material_id: u16,
    pub source_cell_id: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AlchemyRapierVoxelColliderDesc {
    pub translation: AlchemyRapierVec2,
    pub voxel_size: AlchemyRapierVec2,
    pub cells: *const AlchemyRapierVoxelCell,
    pub cell_count: usize,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AlchemyRapierStaticTerrainVoxelColliderDesc {
    pub voxel: AlchemyRapierVoxelColliderDesc,
    pub chunk_x: i32,
    pub chunk_y: i32,
    pub collision_revision: i64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AlchemyRapierPixelRigidbodyDesc {
    pub width: i32,
    pub height: i32,
    pub pixel_size: f32,
    pub local_origin: AlchemyRapierVec2,
    pub topology_revision: u64,
    pub topology_version: u32,
    pub update_kind: i32,
    pub occupancy_words: *const u64,
    pub occupancy_word_count: usize,
    pub material_ids: *const u16,
    pub material_id_count: usize,
    pub support_mask: *const u8,
    pub support_mask_count: usize,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct AlchemyRapierRigidBodyHandle {
    pub index: u32,
    pub generation: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct AlchemyRapierColliderHandle {
    pub index: u32,
    pub generation: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct AlchemyRapierJointHandle {
    pub index: u32,
    pub generation: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct AlchemyRapierBodyDesc {
    pub body_type: AlchemyRapierBodyType,
    pub position: AlchemyRapierVec2,
    pub rotation: f32,
    pub linear_velocity: AlchemyRapierVec2,
    pub angular_velocity: f32,
    pub linear_damping: f32,
    pub angular_damping: f32,
    pub gravity_scale: f32,
    pub local_center_of_mass: AlchemyRapierVec2,
    pub mass: f32,
    pub inertia: f32,
    pub fixed_rotation: u8,
    pub can_sleep: u8,
    pub write_transform: u8,
    pub write_velocity: u8,
    pub wake_up: u8,
    pub sleep: u8,
    pub use_collider_mass: u8,
    pub user_data: u64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct AlchemyRapierCreateWorldResult {
    pub status: AlchemyRapierStatus,
    pub world: *mut AlchemyRapierWorld,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct AlchemyRapierCreateBodyResult {
    pub status: AlchemyRapierStatus,
    pub handle: AlchemyRapierRigidBodyHandle,
    pub packed_id: u64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct AlchemyRapierCreateColliderResult {
    pub status: AlchemyRapierStatus,
    pub handle: AlchemyRapierColliderHandle,
    pub packed_id: u64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct AlchemyRapierCreateJointResult {
    pub status: AlchemyRapierStatus,
    pub handle: AlchemyRapierJointHandle,
    pub packed_id: u64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct AlchemyRapierJointImpulseResult {
    pub status: AlchemyRapierStatus,
    pub linear_impulse: AlchemyRapierVec2,
    pub angular_impulse: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct AlchemyRapierBodyStateResult {
    pub status: AlchemyRapierStatus,
    pub packed_id: u64,
    pub body_type: AlchemyRapierBodyType,
    pub position: AlchemyRapierVec2,
    pub rotation: f32,
    pub linear_velocity: AlchemyRapierVec2,
    pub angular_velocity: f32,
    pub linear_damping: f32,
    pub angular_damping: f32,
    pub can_sleep: u8,
    pub is_awake: u8,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct AlchemyRapierRagdollMotorDesc {
    pub body_a: AlchemyRapierRigidBodyHandle,
    pub body_b: AlchemyRapierRigidBodyHandle,
    pub target_relative_angle: f32,
    pub target_relative_angular_velocity: f32,
    pub reference_relative_angle: f32,
    pub stiffness: f32,
    pub damping: f32,
    pub max_torque: f32,
    pub delta_seconds: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AlchemyRapierRagdollMotorResult {
    pub angle_error: f32,
    pub applied_torque: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct AlchemyRapierMassResult {
    pub status: AlchemyRapierStatus,
    pub local_center_of_mass: AlchemyRapierVec2,
    pub mass: f32,
    pub inertia: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct AlchemyRapierPixelRigidbodyResult {
    pub status: AlchemyRapierStatus,
    pub collider_handle: AlchemyRapierColliderHandle,
    pub collider_packed_id: u64,
    pub solid_count: usize,
    pub shape_count: usize,
    pub local_center_of_mass: AlchemyRapierVec2,
    pub mass: f32,
    pub inertia: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct AlchemyRapierVec2Result {
    pub status: AlchemyRapierStatus,
    pub value: AlchemyRapierVec2,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AlchemyRapierQueryHit {
    pub source_kind: AlchemyRapierQuerySourceKind,
    pub body_packed_id: u64,
    pub collider_packed_id: u64,
    pub terrain_chunk_x: i32,
    pub terrain_chunk_y: i32,
    pub terrain_revision: i64,
    pub terrain_actor_key: i64,
    pub world_cell_x: i32,
    pub world_cell_y: i32,
    pub point: AlchemyRapierVec2,
    pub normal: AlchemyRapierVec2,
    pub local_point: AlchemyRapierVec2,
    pub point_velocity: AlchemyRapierVec2,
    pub distance: f32,
    pub fraction: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct AlchemyRapierQueryResult {
    pub status: AlchemyRapierStatus,
    pub hit_count: usize,
    pub written_count: usize,
    pub candidate_count: usize,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct AlchemyRapierStepResult {
    pub status: AlchemyRapierStatus,
    pub contact_begin_count: usize,
    pub contact_end_count: usize,
    pub contact_hit_count: usize,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AlchemyRapierContactSample {
    pub collider1_packed_id: u64,
    pub collider2_packed_id: u64,
    pub body1_packed_id: u64,
    pub body2_packed_id: u64,
    pub body1_user_data: u64,
    pub body2_user_data: u64,
    pub subshape1: u32,
    pub subshape2: u32,
    pub source_cell_id1: u32,
    pub source_cell_id2: u32,
    pub material_id1: u16,
    pub material_id2: u16,
    pub point: AlchemyRapierVec2,
    pub normal_from_2_to_1: AlchemyRapierVec2,
    pub impulse_on_body1: AlchemyRapierVec2,
    pub collision_impulse: f32,
}

pub type AlchemyRapierContactCallback =
    Option<unsafe extern "C" fn(user_data: *mut c_void, sample: *const AlchemyRapierContactSample)>;

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct AlchemyRapierRevoluteJointDesc {
    pub body1: AlchemyRapierRigidBodyHandle,
    pub body2: AlchemyRapierRigidBodyHandle,
    pub local_anchor1: AlchemyRapierVec2,
    pub local_anchor2: AlchemyRapierVec2,
    pub natural_frequency: f32,
    pub damping_ratio: f32,
    pub contacts_enabled: u8,
    pub wake_up: u8,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct AlchemyRapierGenericJointDesc {
    pub body1: AlchemyRapierRigidBodyHandle,
    pub body2: AlchemyRapierRigidBodyHandle,
    pub local_anchor1: AlchemyRapierVec2,
    pub local_anchor2: AlchemyRapierVec2,
    pub local_rotation1: f32,
    pub local_rotation2: f32,
    pub natural_frequency: f32,
    pub damping_ratio: f32,
    pub limit_min: f32,
    pub limit_max: f32,
    pub locked_axes: u8,
    pub limit_enabled: u8,
    pub contacts_enabled: u8,
    pub wake_up: u8,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct AlchemyRapierRopeJointDesc {
    pub body1: AlchemyRapierRigidBodyHandle,
    pub body2: AlchemyRapierRigidBodyHandle,
    pub local_anchor1: AlchemyRapierVec2,
    pub local_anchor2: AlchemyRapierVec2,
    pub max_distance: f32,
    pub contacts_enabled: u8,
    pub wake_up: u8,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct AlchemyRapierSplitEventReadResult {
    pub status: AlchemyRapierStatus,
    pub row_count: usize,
    pub written_count: usize,
}

#[derive(Clone, Copy, Debug, Default)]
struct CroppedSplitPayload {
    transition_id: u32,
    source_body_handle: AlchemyRapierRigidBodyHandle,
    source_body_packed_id: u64,
    source_collider_handle: AlchemyRapierColliderHandle,
    source_collider_packed_id: u64,
    child_body_handle: AlchemyRapierRigidBodyHandle,
    child_body_packed_id: u64,
    child_collider_handle: AlchemyRapierColliderHandle,
    child_collider_packed_id: u64,
    source_width: i32,
    source_height: i32,
    source_min_x: i32,
    source_min_y: i32,
    source_max_x: i32,
    source_max_y: i32,
    source_cell_count: usize,
    child_width: i32,
    child_height: i32,
    child_pixel_size: f32,
    child_local_origin: AlchemyRapierVec2,
    child_occupancy_word_count: usize,
    child_material_id_count: usize,
    child_solid_count: usize,
    source_local_origin: AlchemyRapierVec2,
    source_solid_count: usize,
    position: AlchemyRapierVec2,
    rotation: f32,
    linear_velocity: AlchemyRapierVec2,
    angular_velocity: f32,
    source_topology_revision: u64,
    source_topology_version: u32,
    child_topology_revision: u64,
    child_topology_version: u32,
    material_hash: u64,
    source_kind: AlchemyRapierQuerySourceKind,
    source_terrain_actor_key: i64,
    source_terrain_chunk_x: i32,
    source_terrain_chunk_y: i32,
    source_terrain_world_origin_x: i32,
    source_terrain_world_origin_y: i32,
    source_terrain_revision: i64,
    source_touched_cell_count: usize,
    source_removed_cell_count: usize,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AlchemyRapierSplitEventRow {
    pub transition_id: u32,
    pub source_kind: AlchemyRapierQuerySourceKind,
    pub source_body_handle: AlchemyRapierRigidBodyHandle,
    pub source_body_packed_id: u64,
    pub source_collider_handle: AlchemyRapierColliderHandle,
    pub source_collider_packed_id: u64,
    pub child_body_handle: AlchemyRapierRigidBodyHandle,
    pub child_body_packed_id: u64,
    pub child_collider_handle: AlchemyRapierColliderHandle,
    pub child_collider_packed_id: u64,
    pub source_terrain_actor_key: i64,
    pub source_terrain_chunk_x: i32,
    pub source_terrain_chunk_y: i32,
    pub source_terrain_world_origin_x: i32,
    pub source_terrain_world_origin_y: i32,
    pub source_terrain_revision: i64,
    pub source_width: i32,
    pub source_height: i32,
    pub source_min_x: i32,
    pub source_min_y: i32,
    pub source_max_x: i32,
    pub source_max_y: i32,
    pub source_cell_count: usize,
    pub source_touched_cell_count: usize,
    pub source_removed_cell_count: usize,
    pub source_local_origin: AlchemyRapierVec2,
    pub source_solid_count: usize,
    pub child_width: i32,
    pub child_height: i32,
    pub child_pixel_size: f32,
    pub child_local_origin: AlchemyRapierVec2,
    pub child_occupancy_word_count: usize,
    pub child_material_id_count: usize,
    pub child_solid_count: usize,
    pub position: AlchemyRapierVec2,
    pub rotation: f32,
    pub linear_velocity: AlchemyRapierVec2,
    pub angular_velocity: f32,
    pub source_topology_revision: u64,
    pub source_topology_version: u32,
    pub child_topology_revision: u64,
    pub child_topology_version: u32,
    pub material_hash: u64,
}

#[repr(C)]
pub struct AlchemyRapierWorld {
    _private: [u8; 0],
}

#[repr(C)]
pub struct AlchemyFxRapierWorld {
    _private: [u8; 0],
}

struct AlchemyFxRapierWorldInner {
    world: FxRapierWorld2D,
    last_step: Option<FxStepWithDiagnostics>,
    pending_family_deltas: Vec<AlchemyFxFamilyDeltaRow>,
    next_pending_family_delta_id: u64,
    snapshot_scratch: Vec<u8>,
}

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
struct TerrainKey {
    x: i32,
    y: i32,
}

#[allow(dead_code)]
struct TerrainChunkState {
    asset: AuthoredVoxelAsset,
    collider: ColliderHandle,
    chunk_x: i32,
    chunk_y: i32,
    source_world_origin_x: i32,
    source_world_origin_y: i32,
    local_origin_x: i32,
    local_origin_y: i32,
    pixel_shape_local_origin: Vector,
    revision: i64,
    width: u32,
    height: u32,
    pixel_size: f32,
    topology_revision: u64,
    topology_version: u32,
    occupancy: Vec<bool>,
    material_ids: Vec<u16>,
    support_mask: Vec<u8>,
    solid_count: usize,
}

#[allow(dead_code)]
struct TerrainFractureActorState {
    asset: AuthoredVoxelAsset,
    runtime: VoxelRuntime,
    actor: FxActorId,
    body: RigidBodyHandle,
    collider: ColliderHandle,
    actor_key: i64,
    chunk_x: i32,
    chunk_y: i32,
    source_world_origin_x: i32,
    source_world_origin_y: i32,
    local_origin_x: i32,
    local_origin_y: i32,
    pixel_shape_local_origin: Vector,
    revision: i64,
    width: u32,
    height: u32,
    pixel_size: f32,
    topology_revision: u64,
    topology_version: u32,
    material_ids: Vec<u16>,
    support_mask: Vec<u8>,
    solid_count: usize,
}

#[allow(dead_code)]
struct PixelRigidbodyState {
    asset: AuthoredVoxelAsset,
    runtime: VoxelRuntime,
    actor: FxActorId,
    collider: ColliderHandle,
    width: u32,
    height: u32,
    pixel_size: f32,
    local_origin: Vector,
    topology_revision: u64,
    topology_version: u32,
    material_ids: Vec<u16>,
    support_mask: Vec<u8>,
    solid_count: usize,
}

#[derive(Clone, Copy, Debug)]
struct ContactMaterial {
    friction: f32,
    restitution: f32,
    friction_combine_rule: CoefficientCombineRule,
    restitution_combine_rule: CoefficientCombineRule,
    #[allow(dead_code)]
    hardness: f32,
}

#[derive(Clone, Copy, Debug, Default)]
struct VoxelCellMetadata {
    material_id: u16,
    source_cell_id: u32,
}

#[derive(Clone, Copy, Debug, Default)]
struct VoxelTerrainSourceMetadata {
    chunk_x: i32,
    chunk_y: i32,
    revision: i64,
}

struct PendingSplitEvent {
    row: AlchemyRapierSplitEventRow,
    source_cell_indices: Vec<i32>,
    touched_source_cell_indices: Vec<i32>,
    removed_source_cell_indices: Vec<i32>,
    child_occupancy_words: Vec<u64>,
    child_material_ids: Vec<u16>,
}

struct AlchemyRapierWorldInner {
    gravity: Vector,
    integration_parameters: IntegrationParameters,
    pipeline: PhysicsPipeline,
    islands: IslandManager,
    broad_phase: BroadPhaseBvh,
    narrow_phase: NarrowPhase,
    bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd_solver: CCDSolver,
    terrain_chunks: HashMap<TerrainKey, TerrainChunkState>,
    terrain_by_collider: HashMap<ColliderHandle, TerrainKey>,
    terrain_fracture_actors: HashMap<i64, TerrainFractureActorState>,
    terrain_fracture_actor_by_collider: HashMap<ColliderHandle, i64>,
    pixel_rigidbodies: HashMap<RigidBodyHandle, PixelRigidbodyState>,
    contact_materials: HashMap<u16, ContactMaterial>,
    voxel_colliders: HashMap<(u32, u32), HashMap<u32, VoxelCellMetadata>>,
    voxel_terrain_sources: HashMap<ColliderHandle, VoxelTerrainSourceMetadata>,
    pending_split_events: Vec<PendingSplitEvent>,
    previous_active_contact_pairs: HashSet<(u64, u64)>,
}

impl AlchemyRapierWorldInner {
    fn new() -> Self {
        Self {
            gravity: Vector::new(0.0, -9.81),
            integration_parameters: IntegrationParameters::default(),
            pipeline: PhysicsPipeline::new(),
            islands: IslandManager::new(),
            broad_phase: BroadPhaseBvh::new(),
            narrow_phase: NarrowPhase::new(),
            bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            terrain_chunks: HashMap::new(),
            terrain_by_collider: HashMap::new(),
            terrain_fracture_actors: HashMap::new(),
            terrain_fracture_actor_by_collider: HashMap::new(),
            pixel_rigidbodies: HashMap::new(),
            contact_materials: HashMap::new(),
            voxel_colliders: HashMap::new(),
            voxel_terrain_sources: HashMap::new(),
            pending_split_events: Vec::new(),
            previous_active_contact_pairs: HashSet::new(),
        }
    }

    fn step_once(&mut self, dt: f32) {
        self.integration_parameters.dt = dt;
        let hooks = AlchemyContactHooks {
            contact_materials: &self.contact_materials,
            voxel_colliders: &self.voxel_colliders,
        };
        self.pipeline.step(
            self.gravity,
            &self.integration_parameters,
            &mut self.islands,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut self.ccd_solver,
            &hooks,
            &(),
        );
    }
}

fn empty_step_result(status: AlchemyRapierStatus) -> AlchemyRapierStepResult {
    AlchemyRapierStepResult {
        status,
        contact_begin_count: 0,
        contact_end_count: 0,
        contact_hit_count: 0,
    }
}

fn sorted_contact_pair_key(collider1: ColliderHandle, collider2: ColliderHandle) -> (u64, u64) {
    let packed1 = pack_collider_handle(collider1);
    let packed2 = pack_collider_handle(collider2);
    if packed1 <= packed2 {
        (packed1, packed2)
    } else {
        (packed2, packed1)
    }
}

fn body_packed_id(body: Option<RigidBodyHandle>) -> u64 {
    body.map(pack_body_handle).unwrap_or(0)
}

fn collect_active_contact_pairs(world: &AlchemyRapierWorldInner) -> HashSet<(u64, u64)> {
    let mut active_pairs = HashSet::new();
    for pair in world.narrow_phase.contact_pairs() {
        if !pair.has_any_active_contact() {
            continue;
        }
        active_pairs.insert(sorted_contact_pair_key(pair.collider1, pair.collider2));
    }
    active_pairs
}

fn emit_contact_samples(
    world: &AlchemyRapierWorldInner,
    callback: AlchemyRapierContactCallback,
    user_data: *mut c_void,
) -> usize {
    let Some(callback) = callback else {
        return 0;
    };

    let mut sample_count = 0usize;
    for pair in world.narrow_phase.contact_pairs() {
        if !pair.has_any_active_contact() {
            continue;
        }
        for manifold in &pair.manifolds {
            if manifold.data.solver_contacts.is_empty() {
                continue;
            }

            let force_dir1 = -manifold.data.normal;
            let tangent = Vector::new(-force_dir1.y, force_dir1.x);
            for (contact_index, contact) in manifold.points.iter().enumerate() {
                let normal_impulse = contact.data.impulse;
                let tangent_impulse = contact.data.tangent_impulse.x;
                if normal_impulse.abs() <= 0.000001 && tangent_impulse.abs() <= 0.000001 {
                    continue;
                }

                let impulse = force_dir1 * normal_impulse + tangent * tangent_impulse;
                let collision_impulse = impulse.length();
                if !collision_impulse.is_finite() || collision_impulse <= 0.000001 {
                    continue;
                }

                if let Some(solver_contact) = manifold.data.solver_contacts.get(contact_index) {
                    if !solver_contact.point.x.is_finite()
                        || !solver_contact.point.y.is_finite()
                        || !impulse.x.is_finite()
                        || !impulse.y.is_finite()
                    {
                        continue;
                    }

                    let voxel1 =
                        voxel_metadata(&world.voxel_colliders, pair.collider1, manifold.subshape1);
                    let voxel2 =
                        voxel_metadata(&world.voxel_colliders, pair.collider2, manifold.subshape2);
                    let sample = AlchemyRapierContactSample {
                        collider1_packed_id: pack_collider_handle(pair.collider1),
                        collider2_packed_id: pack_collider_handle(pair.collider2),
                        body1_packed_id: body_packed_id(manifold.data.rigid_body1),
                        body2_packed_id: body_packed_id(manifold.data.rigid_body2),
                        body1_user_data: manifold
                            .data
                            .rigid_body1
                            .and_then(|handle| world.bodies.get(handle))
                            .map(|body| body.user_data as u64)
                            .unwrap_or(0),
                        body2_user_data: manifold
                            .data
                            .rigid_body2
                            .and_then(|handle| world.bodies.get(handle))
                            .map(|body| body.user_data as u64)
                            .unwrap_or(0),
                        subshape1: manifold.subshape1,
                        subshape2: manifold.subshape2,
                        source_cell_id1: voxel1
                            .map(|metadata| metadata.source_cell_id)
                            .unwrap_or(INVALID_SOURCE_CELL_ID),
                        source_cell_id2: voxel2
                            .map(|metadata| metadata.source_cell_id)
                            .unwrap_or(INVALID_SOURCE_CELL_ID),
                        material_id1: voxel1.map(|metadata| metadata.material_id).unwrap_or(0),
                        material_id2: voxel2.map(|metadata| metadata.material_id).unwrap_or(0),
                        point: ffi_vec(solver_contact.point),
                        normal_from_2_to_1: ffi_vec(force_dir1),
                        impulse_on_body1: ffi_vec(impulse),
                        collision_impulse,
                    };
                    unsafe { callback(user_data, &sample) };
                    sample_count = sample_count.saturating_add(1);
                }
            }
        }
    }
    sample_count
}

fn to_inner<'a>(
    world: *mut AlchemyRapierWorld,
) -> Result<&'a mut AlchemyRapierWorldInner, AlchemyRapierStatus> {
    if world.is_null() {
        return Err(AlchemyRapierStatus::NullPointer);
    }
    Ok(unsafe { &mut *world.cast::<AlchemyRapierWorldInner>() })
}

fn to_fx_world<'a>(
    world: *mut AlchemyFxRapierWorld,
) -> Result<&'a mut AlchemyFxRapierWorldInner, AlchemyRapierStatus> {
    if world.is_null() {
        return Err(AlchemyRapierStatus::NullPointer);
    }
    Ok(unsafe { &mut *world.cast::<AlchemyFxRapierWorldInner>() })
}

fn fx_error_status(error: FxRapierError) -> AlchemyRapierStatus {
    match error {
        FxRapierError::UnknownFamily(_)
        | FxRapierError::UnknownActor { .. }
        | FxRapierError::MissingSplitParentSnapshot { .. }
        | FxRapierError::UnknownReplayFamily(_) => AlchemyRapierStatus::InvalidHandle,
        FxRapierError::UnsupportedConnectionPolicy(_)
        | FxRapierError::ReplayRequiresDeterministicMode
        | FxRapierError::UnsupportedSplitFamilyPromotion(_) => AlchemyRapierStatus::Unsupported,
        FxRapierError::DuplicateFamily(_)
        | FxRapierError::DuplicateReplayKey { .. }
        | FxRapierError::Connection(_)
        | FxRapierError::InvalidVoxelUpdate
        | FxRapierError::Voxel(_)
        | FxRapierError::Snapshot(_) => AlchemyRapierStatus::InvalidArgument,
    }
}

fn snapshot_mode_from_ffi(mode: AlchemyFxSnapshotMode) -> SnapshotMode {
    match mode {
        AlchemyFxSnapshotMode::Normal => SnapshotMode::Normal,
        AlchemyFxSnapshotMode::Deterministic => SnapshotMode::Deterministic,
    }
}

fn damage_source_from_ffi(source: AlchemyFxDamageSource) -> DamageSource {
    match source {
        AlchemyFxDamageSource::Script => DamageSource::Script,
        AlchemyFxDamageSource::ContactImpulse => DamageSource::ContactImpulse,
        AlchemyFxDamageSource::JointFeedback => DamageSource::JointFeedback,
        AlchemyFxDamageSource::Stress => DamageSource::Stress,
    }
}

fn fx_cell_count(width: u32, height: u32) -> Result<usize, AlchemyRapierStatus> {
    if width == 0 || height == 0 {
        return Err(AlchemyRapierStatus::InvalidArgument);
    }
    (width as usize)
        .checked_mul(height as usize)
        .filter(|count| *count > 0)
        .ok_or(AlchemyRapierStatus::InvalidArgument)
}

fn dense_u16_from_ffi(
    values: *const u16,
    value_count: usize,
    cell_count: usize,
    fallback: u16,
) -> Result<Vec<u16>, AlchemyRapierStatus> {
    if values.is_null() || value_count == 0 {
        return Ok(vec![fallback; cell_count]);
    }
    if value_count < cell_count {
        return Err(AlchemyRapierStatus::InvalidArgument);
    }
    Ok(unsafe { slice::from_raw_parts(values, cell_count) }.to_vec())
}

fn dense_u32_from_ffi(
    values: *const u32,
    value_count: usize,
    cell_count: usize,
) -> Result<Vec<u32>, AlchemyRapierStatus> {
    if values.is_null() || value_count == 0 {
        return Ok((0..cell_count)
            .map(|index| index.min(u32::MAX as usize) as u32)
            .collect());
    }
    if value_count < cell_count {
        return Err(AlchemyRapierStatus::InvalidArgument);
    }
    Ok(unsafe { slice::from_raw_parts(values, cell_count) }.to_vec())
}

fn authored_fx_asset_from_desc(
    desc: AlchemyFxVoxelDestructibleDesc,
) -> Result<AuthoredVoxelAsset, AlchemyRapierStatus> {
    if !desc.voxel_size.is_finite() || desc.voxel_size <= 0.0 {
        return Err(AlchemyRapierStatus::InvalidArgument);
    }
    let cell_count = fx_cell_count(desc.width, desc.height)?;
    let expected_word_count = cell_count.div_ceil(64);
    if desc.occupancy_words.is_null() || desc.occupancy_word_count < expected_word_count {
        return Err(AlchemyRapierStatus::InvalidArgument);
    }

    let words = unsafe { slice::from_raw_parts(desc.occupancy_words, expected_word_count) };
    let occupancy = occupancy_from_words(desc.width, desc.height, words);
    if !occupancy.iter().any(|occupied| *occupied) {
        return Err(AlchemyRapierStatus::InvalidArgument);
    }

    let fracture_material = dense_u16_from_ffi(
        desc.fracture_material_ids,
        desc.fracture_material_id_count,
        cell_count,
        0,
    )?;
    let contact_material =
        if desc.contact_material_ids.is_null() || desc.contact_material_id_count == 0 {
            fracture_material.clone()
        } else {
            dense_u16_from_ffi(
                desc.contact_material_ids,
                desc.contact_material_id_count,
                cell_count,
                0,
            )?
        };
    let external_id = dense_u32_from_ffi(desc.external_ids, desc.external_id_count, cell_count)?;
    let input = VoxelAuthoringInput::new(
        desc.width,
        desc.height,
        desc.voxel_size,
        occupancy,
        fracture_material,
        contact_material,
        external_id,
    );
    author_voxel_asset(input).map_err(|_| AlchemyRapierStatus::InvalidArgument)
}

fn fx_split_events(world: &AlchemyFxRapierWorldInner) -> Option<&[SplitEvent]> {
    world
        .last_step
        .as_ref()
        .map(|step| step.report.split_events.as_slice())
}

fn fx_family_deltas(
    world: &AlchemyFxRapierWorldInner,
) -> Option<&[fracture_rapier::FxFamilyDelta]> {
    world
        .last_step
        .as_ref()
        .map(|step| step.report.family_deltas.as_slice())
}

fn handle_from_ffi(handle: AlchemyRapierRigidBodyHandle) -> RigidBodyHandle {
    RigidBodyHandle::from_raw_parts(handle.index, handle.generation)
}

fn collider_handle_from_ffi(handle: AlchemyRapierColliderHandle) -> ColliderHandle {
    ColliderHandle::from_raw_parts(handle.index, handle.generation)
}

fn joint_handle_from_ffi(handle: AlchemyRapierJointHandle) -> ImpulseJointHandle {
    ImpulseJointHandle::from_raw_parts(handle.index, handle.generation)
}

fn collider_handle_from_packed(packed_id: u64) -> Option<ColliderHandle> {
    let packed_index = packed_id as u32;
    if packed_index == 0 {
        return None;
    }
    Some(ColliderHandle::from_raw_parts(
        packed_index - 1,
        (packed_id >> 32) as u32,
    ))
}

fn handle_to_ffi(handle: RigidBodyHandle) -> AlchemyRapierRigidBodyHandle {
    let (index, generation) = handle.into_raw_parts();
    AlchemyRapierRigidBodyHandle { index, generation }
}

fn collider_handle_to_ffi(handle: ColliderHandle) -> AlchemyRapierColliderHandle {
    let (index, generation) = handle.into_raw_parts();
    AlchemyRapierColliderHandle { index, generation }
}

fn joint_handle_to_ffi(handle: ImpulseJointHandle) -> AlchemyRapierJointHandle {
    let (index, generation) = handle.into_raw_parts();
    AlchemyRapierJointHandle { index, generation }
}

fn pack_parts(index: u32, generation: u32) -> u64 {
    (u64::from(index) + 1) | (u64::from(generation) << 32)
}

fn pack_body_handle(handle: RigidBodyHandle) -> u64 {
    let (index, generation) = handle.into_raw_parts();
    pack_parts(index, generation)
}

fn pack_collider_handle(handle: ColliderHandle) -> u64 {
    let (index, generation) = handle.into_raw_parts();
    pack_parts(index, generation)
}

fn pack_joint_handle(handle: ImpulseJointHandle) -> u64 {
    let (index, generation) = handle.into_raw_parts();
    pack_parts(index, generation)
}

fn fx_family_body_mode_to_ffi(body_type: FxActorBodyType) -> AlchemyFxFamilyBodyMode {
    match body_type {
        FxActorBodyType::Dynamic => AlchemyFxFamilyBodyMode::Dynamic,
        FxActorBodyType::KinematicVelocityBased => AlchemyFxFamilyBodyMode::KinematicVelocityBased,
        FxActorBodyType::Fixed => AlchemyFxFamilyBodyMode::Fixed,
        FxActorBodyType::KinematicPositionBased => AlchemyFxFamilyBodyMode::KinematicPositionBased,
    }
}

fn fx_family_delta_body_mode_to_ffi(body_mode: FxFamilyBodyMode) -> AlchemyFxFamilyBodyMode {
    match body_mode {
        FxFamilyBodyMode::Dynamic => AlchemyFxFamilyBodyMode::Dynamic,
        FxFamilyBodyMode::AttachedStatic => AlchemyFxFamilyBodyMode::AttachedStatic,
        FxFamilyBodyMode::Fixed => AlchemyFxFamilyBodyMode::Fixed,
        FxFamilyBodyMode::KinematicVelocityBased => AlchemyFxFamilyBodyMode::KinematicVelocityBased,
        FxFamilyBodyMode::KinematicPositionBased => AlchemyFxFamilyBodyMode::KinematicPositionBased,
        FxFamilyBodyMode::Destroyed => AlchemyFxFamilyBodyMode::Destroyed,
    }
}

fn fx_family_delta_kind_to_ffi(kind: FxFamilyDeltaKind) -> AlchemyFxFamilyDeltaKind {
    match kind {
        FxFamilyDeltaKind::Created => AlchemyFxFamilyDeltaKind::Created,
        FxFamilyDeltaKind::Updated => AlchemyFxFamilyDeltaKind::Updated,
        FxFamilyDeltaKind::Destroyed => AlchemyFxFamilyDeltaKind::Destroyed,
    }
}

fn next_pending_family_delta_id(world: &mut AlchemyFxRapierWorldInner) -> u64 {
    let id = world.next_pending_family_delta_id;
    world.next_pending_family_delta_id = world.next_pending_family_delta_id.saturating_add(1);
    id
}

fn push_pending_family_delta(world: &mut AlchemyFxRapierWorldInner, row: AlchemyFxFamilyDeltaRow) {
    world
        .pending_family_deltas
        .retain(|pending| pending.family_id != row.family_id);
    world.pending_family_deltas.push(row);
}

fn push_pending_family_delta_from_rapier(
    world: &mut AlchemyFxRapierWorldInner,
    delta: &fracture_rapier::FxFamilyDelta,
) {
    let row = AlchemyFxFamilyDeltaRow {
        delta_sequence: world.world.tick(),
        delta_id: delta.delta_id,
        kind: fx_family_delta_kind_to_ffi(delta.kind),
        family_id: delta.family_id.0,
        parent_family_id: delta.parent_family_id.0,
        body_mode: fx_family_delta_body_mode_to_ffi(delta.body_mode),
        occupied_voxel_count: delta.occupied_voxel_count,
        actor_count: delta.actor_count,
    };
    push_pending_family_delta(world, row);
}

fn vector(value: AlchemyRapierVec2) -> Vector {
    Vector::new(value.x, value.y)
}

fn ffi_vec(value: Vector) -> AlchemyRapierVec2 {
    AlchemyRapierVec2 {
        x: value.x,
        y: value.y,
    }
}

fn collider_key(handle: ColliderHandle) -> (u32, u32) {
    let (index, generation) = handle.into_raw_parts();
    (index, generation)
}

fn combine_rule(value: AlchemyRapierCoefficientCombineRule) -> CoefficientCombineRule {
    match value {
        AlchemyRapierCoefficientCombineRule::Average => CoefficientCombineRule::Average,
        AlchemyRapierCoefficientCombineRule::Min => CoefficientCombineRule::Min,
        AlchemyRapierCoefficientCombineRule::Multiply => CoefficientCombineRule::Multiply,
        AlchemyRapierCoefficientCombineRule::Max => CoefficientCombineRule::Max,
        AlchemyRapierCoefficientCombineRule::ClampedSum => CoefficientCombineRule::ClampedSum,
    }
}

fn combine_coefficient(
    left: f32,
    right: f32,
    left_rule: CoefficientCombineRule,
    right_rule: CoefficientCombineRule,
) -> f32 {
    match left_rule.max(right_rule) {
        CoefficientCombineRule::Average => (left + right) * 0.5,
        CoefficientCombineRule::Min => left.min(right).abs(),
        CoefficientCombineRule::Multiply => left * right,
        CoefficientCombineRule::Max => left.max(right),
        CoefficientCombineRule::ClampedSum => (left + right).clamp(0.0, 1.0),
    }
}

fn collider_contact_material(
    context: &ContactModificationContext,
    handle: ColliderHandle,
) -> ContactMaterial {
    if let Some(collider) = context.colliders.get(handle) {
        ContactMaterial {
            friction: collider.friction(),
            restitution: collider.restitution(),
            friction_combine_rule: collider.friction_combine_rule(),
            restitution_combine_rule: collider.restitution_combine_rule(),
            hardness: 1.0,
        }
    } else {
        ContactMaterial {
            friction: 0.5,
            restitution: 0.0,
            friction_combine_rule: CoefficientCombineRule::Average,
            restitution_combine_rule: CoefficientCombineRule::Average,
            hardness: 1.0,
        }
    }
}

fn voxel_metadata(
    voxel_colliders: &HashMap<(u32, u32), HashMap<u32, VoxelCellMetadata>>,
    handle: ColliderHandle,
    subshape: u32,
) -> Option<VoxelCellMetadata> {
    voxel_colliders
        .get(&collider_key(handle))
        .and_then(|metadata| metadata.get(&subshape))
        .copied()
}

fn voxel_terrain_source_metadata(
    desc: AlchemyRapierStaticTerrainVoxelColliderDesc,
) -> Result<VoxelTerrainSourceMetadata, AlchemyRapierStatus> {
    if desc.collision_revision < 0 {
        return Err(AlchemyRapierStatus::InvalidArgument);
    }

    Ok(VoxelTerrainSourceMetadata {
        chunk_x: desc.chunk_x,
        chunk_y: desc.chunk_y,
        revision: desc.collision_revision,
    })
}

fn remove_voxel_collider_metadata(world: &mut AlchemyRapierWorldInner, handle: ColliderHandle) {
    world.voxel_colliders.remove(&collider_key(handle));
    world.voxel_terrain_sources.remove(&handle);
}

struct AlchemyContactHooks<'a> {
    contact_materials: &'a HashMap<u16, ContactMaterial>,
    voxel_colliders: &'a HashMap<(u32, u32), HashMap<u32, VoxelCellMetadata>>,
}

impl AlchemyContactHooks<'_> {
    fn material_for(
        &self,
        context: &ContactModificationContext,
        handle: ColliderHandle,
        subshape: u32,
    ) -> ContactMaterial {
        if let Some(voxel) = voxel_metadata(self.voxel_colliders, handle, subshape) {
            if let Some(material) = self.contact_materials.get(&voxel.material_id) {
                return *material;
            }
        }

        collider_contact_material(context, handle)
    }
}

impl PhysicsHooks for AlchemyContactHooks<'_> {
    fn filter_contact_pair(&self, context: &PairFilterContext) -> Option<SolverFlags> {
        let group1 = context
            .colliders
            .get(context.collider1)
            .and_then(|collider| {
                (collider.user_data & SELF_COLLISION_FILTER_TAG != 0)
                    .then_some(collider.user_data as u64)
            });
        let group2 = context
            .colliders
            .get(context.collider2)
            .and_then(|collider| {
                (collider.user_data & SELF_COLLISION_FILTER_TAG != 0)
                    .then_some(collider.user_data as u64)
            });
        if group1.is_some() && group1 == group2 {
            None
        } else {
            Some(SolverFlags::COMPUTE_IMPULSES)
        }
    }

    fn modify_solver_contacts(&self, context: &mut ContactModificationContext) {
        let material1 = self.material_for(context, context.collider1, context.manifold.subshape1);
        let material2 = self.material_for(context, context.collider2, context.manifold.subshape2);
        let friction = combine_coefficient(
            material1.friction,
            material2.friction,
            material1.friction_combine_rule,
            material2.friction_combine_rule,
        );
        let restitution = combine_coefficient(
            material1.restitution,
            material2.restitution,
            material1.restitution_combine_rule,
            material2.restitution_combine_rule,
        );

        for contact in context.solver_contacts.iter_mut() {
            contact.friction = friction;
            contact.restitution = restitution;
        }
    }
}

fn pose_translation(value: Vector) -> Pose {
    Pose::from_parts(value, Rotation::identity())
}

fn ragdoll_motor_desc_is_valid(motor: AlchemyRapierRagdollMotorDesc) -> bool {
    motor.target_relative_angle.is_finite()
        && motor.target_relative_angular_velocity.is_finite()
        && motor.reference_relative_angle.is_finite()
        && motor.stiffness.is_finite()
        && motor.stiffness >= 0.0
        && motor.damping.is_finite()
        && motor.damping >= 0.0
        && motor.max_torque.is_finite()
        && motor.max_torque >= 0.0
        && motor.delta_seconds.is_finite()
        && motor.delta_seconds > 0.0
}

fn normalize_ragdoll_angle(angle: f32) -> f32 {
    (angle + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI
}

fn normalized_or_zero(value: Vector) -> Vector {
    let length = value.length();
    if length > 0.000001 && length.is_finite() {
        value / length
    } else {
        Vector::ZERO
    }
}

fn sanitize_positive(value: f32, fallback: f32) -> f32 {
    if value.is_finite() && value > 0.0 {
        value
    } else {
        fallback
    }
}

fn body_type_to_rapier(value: AlchemyRapierBodyType) -> RigidBodyType {
    match value {
        AlchemyRapierBodyType::Kinematic => RigidBodyType::KinematicVelocityBased,
        AlchemyRapierBodyType::KinematicPosition => RigidBodyType::KinematicPositionBased,
        AlchemyRapierBodyType::Fixed => RigidBodyType::Fixed,
        AlchemyRapierBodyType::Dynamic => RigidBodyType::Dynamic,
    }
}

fn body_type_from_rapier(value: RigidBodyType) -> AlchemyRapierBodyType {
    match value {
        RigidBodyType::Fixed => AlchemyRapierBodyType::Fixed,
        RigidBodyType::KinematicPositionBased => AlchemyRapierBodyType::KinematicPosition,
        RigidBodyType::KinematicVelocityBased => AlchemyRapierBodyType::Kinematic,
        RigidBodyType::Dynamic => AlchemyRapierBodyType::Dynamic,
    }
}

fn body_builder(desc: AlchemyRapierBodyDesc) -> RigidBodyBuilder {
    let mut builder = RigidBodyBuilder::new(body_type_to_rapier(desc.body_type))
        .translation(vector(desc.position))
        .rotation(desc.rotation)
        .linvel(vector(desc.linear_velocity))
        .angvel(desc.angular_velocity)
        .linear_damping(desc.linear_damping.max(0.0))
        .angular_damping(desc.angular_damping.max(0.0))
        .gravity_scale(desc.gravity_scale)
        .can_sleep(desc.can_sleep != 0);
    if desc.use_collider_mass == 0 {
        builder = builder.additional_mass_properties(MassProperties::new(
            vector(desc.local_center_of_mass),
            sanitize_positive(desc.mass, 1.0),
            sanitize_positive(desc.inertia, 1.0),
        ));
    }
    if desc.fixed_rotation != 0 {
        builder = builder.lock_rotations();
    }
    if desc.sleep != 0 {
        builder = builder.sleeping(true);
    }
    builder.user_data(desc.user_data as u128)
}

fn joint_softness(natural_frequency: f32, damping_ratio: f32) -> SpringCoefficients<f32> {
    if natural_frequency == 0.0 {
        SpringCoefficients::joint_defaults()
    } else {
        SpringCoefficients::new(natural_frequency, damping_ratio)
    }
}

fn body_can_sleep(body: &RigidBody) -> bool {
    body.activation().normalized_linear_threshold >= 0.0
}

fn set_body_can_sleep(body: &mut RigidBody, can_sleep: bool) {
    if body_can_sleep(body) == can_sleep {
        return;
    }

    let activation = body.activation_mut();
    if can_sleep {
        activation.normalized_linear_threshold =
            RigidBodyActivation::default_normalized_linear_threshold();
        activation.angular_threshold = RigidBodyActivation::default_angular_threshold();
        activation.time_until_sleep = RigidBodyActivation::default_time_until_sleep();
    } else {
        activation.normalized_linear_threshold = -1.0;
        activation.angular_threshold = -1.0;
    }

    if !can_sleep {
        body.wake_up(true);
    }
}

fn apply_body_desc(body: &mut RigidBody, desc: AlchemyRapierBodyDesc) {
    let wake_up = desc.wake_up != 0;
    body.set_body_type(body_type_to_rapier(desc.body_type), wake_up);
    if desc.write_transform != 0 {
        body.set_position(
            Pose::from_parts(vector(desc.position), Rotation::new(desc.rotation)),
            wake_up,
        );
    }
    if desc.write_velocity != 0 {
        body.set_linvel(vector(desc.linear_velocity), wake_up);
        body.set_angvel(desc.angular_velocity, wake_up);
    }
    body.set_linear_damping(desc.linear_damping.max(0.0));
    body.set_angular_damping(desc.angular_damping.max(0.0));
    body.set_gravity_scale(desc.gravity_scale, wake_up);
    body.lock_rotations(desc.fixed_rotation != 0, wake_up);
    if desc.use_collider_mass != 0 {
        body.set_additional_mass(0.0, wake_up);
    } else {
        body.set_additional_mass_properties(
            MassProperties::new(
                vector(desc.local_center_of_mass),
                sanitize_positive(desc.mass, 1.0),
                sanitize_positive(desc.inertia, 1.0),
            ),
            wake_up,
        );
    }
    set_body_can_sleep(body, desc.can_sleep != 0);
    if desc.sleep != 0 {
        body.sleep();
    } else if wake_up {
        body.wake_up(true);
    }
    body.user_data = desc.user_data as u128;
}

fn recompute_body_mass(world: &mut AlchemyRapierWorldInner, handle: RigidBodyHandle) {
    if let Some(body) = world.bodies.get_mut(handle) {
        body.recompute_mass_properties_from_colliders(&world.colliders);
    }
}

fn make_body_state(handle: RigidBodyHandle, body: &RigidBody) -> AlchemyRapierBodyStateResult {
    AlchemyRapierBodyStateResult {
        status: AlchemyRapierStatus::Ok,
        packed_id: pack_body_handle(handle),
        body_type: body_type_from_rapier(body.body_type()),
        position: ffi_vec(body.center_of_mass()),
        rotation: body.rotation().angle(),
        linear_velocity: ffi_vec(body.linvel()),
        angular_velocity: body.angvel(),
        linear_damping: body.linear_damping(),
        angular_damping: body.angular_damping(),
        can_sleep: if body.activation().normalized_linear_threshold >= 0.0 {
            1
        } else {
            0
        },
        is_awake: if body.is_sleeping() { 0 } else { 1 },
    }
}

fn status_result(status: AlchemyRapierStatus) -> AlchemyRapierStatus {
    status
}

fn empty_pixel_rigidbody_result(status: AlchemyRapierStatus) -> AlchemyRapierPixelRigidbodyResult {
    AlchemyRapierPixelRigidbodyResult {
        status,
        collider_handle: AlchemyRapierColliderHandle::default(),
        collider_packed_id: 0,
        solid_count: 0,
        shape_count: 0,
        local_center_of_mass: AlchemyRapierVec2::default(),
        mass: 0.0,
        inertia: 0.0,
    }
}

fn empty_create_collider_result(status: AlchemyRapierStatus) -> AlchemyRapierCreateColliderResult {
    AlchemyRapierCreateColliderResult {
        status,
        handle: AlchemyRapierColliderHandle::default(),
        packed_id: 0,
    }
}

fn empty_create_joint_result(status: AlchemyRapierStatus) -> AlchemyRapierCreateJointResult {
    AlchemyRapierCreateJointResult {
        status,
        handle: AlchemyRapierJointHandle::default(),
        packed_id: 0,
    }
}

fn empty_joint_impulse_result(status: AlchemyRapierStatus) -> AlchemyRapierJointImpulseResult {
    AlchemyRapierJointImpulseResult {
        status,
        linear_impulse: AlchemyRapierVec2::default(),
        angular_impulse: 0.0,
    }
}

fn empty_query_result(status: AlchemyRapierStatus) -> AlchemyRapierQueryResult {
    AlchemyRapierQueryResult {
        status,
        hit_count: 0,
        written_count: 0,
        candidate_count: 0,
    }
}

fn remove_pixel_rigidbody_state(world: &mut AlchemyRapierWorldInner, body_handle: RigidBodyHandle) {
    if let Some(existing) = world.pixel_rigidbodies.remove(&body_handle) {
        let _ = world.colliders.remove(
            existing.collider,
            &mut world.islands,
            &mut world.bodies,
            true,
        );
    }
}

fn discard_pending_adoptions_for_body(
    world: &mut AlchemyRapierWorldInner,
    body_handle: RigidBodyHandle,
) {
    world.pending_split_events.retain(|event| {
        let source = handle_from_ffi(event.row.source_body_handle);
        let child = handle_from_ffi(event.row.child_body_handle);
        source != body_handle && child != body_handle
    });
}

fn remove_pixel_rigidbody(world: &mut AlchemyRapierWorldInner, body_handle: RigidBodyHandle) {
    discard_pending_adoptions_for_body(world, body_handle);
    remove_pixel_rigidbody_state(world, body_handle);
}

fn clear_body_colliders(world: &mut AlchemyRapierWorldInner, body_handle: RigidBodyHandle) {
    discard_pending_adoptions_for_body(world, body_handle);
    world.pixel_rigidbodies.remove(&body_handle);
    let Some(body) = world.bodies.get(body_handle) else {
        return;
    };
    let colliders = body.colliders().to_vec();
    for collider in colliders {
        let _ = world
            .colliders
            .remove(collider, &mut world.islands, &mut world.bodies, true);
    }
}

fn occupancy_from_words(width: u32, height: u32, words: &[u64]) -> Vec<bool> {
    let cell_count = (width as usize).saturating_mul(height as usize);
    let mut occupancy = vec![false; cell_count];
    for (cell_index, occupied) in occupancy.iter_mut().enumerate() {
        let word_index = cell_index >> 6;
        let bit_index = cell_index & 63;
        if word_index < words.len() {
            *occupied = ((words[word_index] >> bit_index) & 1) != 0;
        }
    }
    occupancy
}

fn pixel_desc_payload(
    desc: AlchemyRapierPixelRigidbodyDesc,
) -> Result<(Vec<bool>, Vec<u16>, Vec<u8>, usize), AlchemyRapierStatus> {
    if desc.width <= 0 || desc.height <= 0 {
        return Err(AlchemyRapierStatus::InvalidArgument);
    }
    if !desc.pixel_size.is_finite() || desc.pixel_size <= 0.0 {
        return Err(AlchemyRapierStatus::InvalidArgument);
    }
    if !desc.local_origin.x.is_finite() || !desc.local_origin.y.is_finite() {
        return Err(AlchemyRapierStatus::InvalidArgument);
    }

    let width = desc.width as u32;
    let height = desc.height as u32;
    let cell_count = (width as usize).saturating_mul(height as usize);
    if cell_count == 0 {
        return Err(AlchemyRapierStatus::InvalidArgument);
    }
    let expected_word_count = cell_count.div_ceil(64);
    if desc.occupancy_words.is_null() || desc.occupancy_word_count < expected_word_count {
        return Err(AlchemyRapierStatus::InvalidArgument);
    }

    let words = unsafe { slice::from_raw_parts(desc.occupancy_words, expected_word_count) };
    let occupancy = occupancy_from_words(width, height, words);
    let solid_count = occupancy.iter().filter(|occupied| **occupied).count();
    if solid_count == 0 {
        return Err(AlchemyRapierStatus::InvalidArgument);
    }

    let material_ids = if !desc.material_ids.is_null() && desc.material_id_count >= cell_count {
        unsafe { slice::from_raw_parts(desc.material_ids, cell_count) }.to_vec()
    } else {
        vec![0; cell_count]
    };
    let support_mask = if !desc.support_mask.is_null() && desc.support_mask_count >= cell_count {
        unsafe { slice::from_raw_parts(desc.support_mask, cell_count) }.to_vec()
    } else {
        Vec::new()
    };
    Ok((occupancy, material_ids, support_mask, solid_count))
}

fn author_pixel_asset(
    desc: AlchemyRapierPixelRigidbodyDesc,
) -> Result<(AuthoredVoxelAsset, Vec<u16>, Vec<u8>, usize), AlchemyRapierStatus> {
    let (occupancy, material_ids, support_mask, solid_count) = pixel_desc_payload(desc)?;
    let width = desc.width as u32;
    let height = desc.height as u32;
    let cell_count = (width as usize).saturating_mul(height as usize);
    let external_id = (0..cell_count)
        .map(|index| index.min(u32::MAX as usize) as u32)
        .collect::<Vec<_>>();
    let input = VoxelAuthoringInput::new(
        width,
        height,
        desc.pixel_size,
        occupancy,
        material_ids.clone(),
        material_ids.clone(),
        external_id,
    );
    let asset = author_voxel_asset(input).map_err(|_| AlchemyRapierStatus::InvalidArgument)?;
    Ok((asset, material_ids, support_mask, solid_count))
}

fn pixel_shape_half_extents(width: u32, height: u32, pixel_size: f32) -> Vector {
    Vector::new(
        0.5 * width as f32 * pixel_size,
        0.5 * height as f32 * pixel_size,
    )
}

fn asset_point_to_body_local(
    width: u32,
    height: u32,
    pixel_size: f32,
    local_origin: Vector,
    asset_point: Vector,
) -> Vector {
    asset_point - pixel_shape_half_extents(width, height, pixel_size) + local_origin
}

fn build_pixel_collider(asset: &AuthoredVoxelAsset, local_origin: Vector) -> Option<Collider> {
    let voxel_size = asset.core().voxel_size();
    let occupancy = asset.occupancy();
    let width = asset.core().occupancy().width();
    let height = asset.core().occupancy().height();
    let mut shapes = Vec::new();
    for y in 0..height {
        for x in 0..width {
            let idx = (y as usize) * (width as usize) + (x as usize);
            if !occupancy.get(idx).copied().unwrap_or(false) {
                continue;
            }
            let center = Vector::new((x as f32 + 0.5) * voxel_size, (y as f32 + 0.5) * voxel_size);
            shapes.push((
                Pose::from_translation(asset_point_to_body_local(
                    width,
                    height,
                    voxel_size,
                    local_origin,
                    center,
                )),
                SharedShape::cuboid(voxel_size * 0.5, voxel_size * 0.5),
            ));
        }
    }
    if shapes.is_empty() {
        return None;
    }
    Some(ColliderBuilder::compound(shapes).density(1.0).build())
}

fn actor_cells(
    runtime: &VoxelRuntime,
    actor_id: FxActorId,
) -> Option<Vec<(GridCoord, usize, u16)>> {
    let actor = runtime.family().actor(actor_id)?;
    let asset = runtime.asset();
    let width = asset.core().occupancy().width();
    let mut cells = Vec::new();
    for node_id in &actor.owned_nodes {
        let Some(node) = asset.core().node(*node_id) else {
            continue;
        };
        for coord in &node.voxels {
            let metadata = asset.voxel_metadata(*coord).ok()?;
            if !metadata.occupied {
                continue;
            }
            let index = coord.y as usize * width as usize + coord.x as usize;
            cells.push((*coord, index, metadata.fracture_material));
        }
    }
    cells.sort_by_key(|(_, index, _)| *index);
    Some(cells)
}

fn actor_bounds(cells: &[(GridCoord, usize, u16)]) -> Option<(u32, u32, u32, u32)> {
    let first = cells.first()?.0;
    let mut min_x = first.x;
    let mut max_x = first.x;
    let mut min_y = first.y;
    let mut max_y = first.y;
    for (coord, _, _) in cells {
        min_x = min_x.min(coord.x);
        max_x = max_x.max(coord.x);
        min_y = min_y.min(coord.y);
        max_y = max_y.max(coord.y);
    }
    Some((min_x, max_x, min_y, max_y))
}

fn build_actor_collider(
    runtime: &VoxelRuntime,
    actor_id: FxActorId,
    local_origin: Vector,
) -> Option<Collider> {
    let voxel_size = runtime.asset().core().voxel_size();
    let width = runtime.asset().core().occupancy().width();
    let height = runtime.asset().core().occupancy().height();
    let cells = actor_cells(runtime, actor_id)?;
    if cells.is_empty() {
        return None;
    }
    let mut shapes = Vec::with_capacity(cells.len());
    for (coord, _, _) in cells {
        let center = Vector::new(
            (coord.x as f32 + 0.5) * voxel_size,
            (coord.y as f32 + 0.5) * voxel_size,
        );
        shapes.push((
            Pose::from_translation(asset_point_to_body_local(
                width,
                height,
                voxel_size,
                local_origin,
                center,
            )),
            SharedShape::cuboid(voxel_size * 0.5, voxel_size * 0.5),
        ));
    }
    Some(ColliderBuilder::compound(shapes).density(1.0).build())
}

fn pixel_shape_local_origin_for_com(
    width: u32,
    height: u32,
    pixel_size: f32,
    asset_com: Vector,
) -> Vector {
    pixel_shape_half_extents(width, height, pixel_size) - asset_com
}

fn build_cropped_actor_payload(
    runtime: &VoxelRuntime,
    actor_id: FxActorId,
    material_ids: &[u16],
    topology_revision: u64,
    topology_version: u32,
) -> Option<(CroppedSplitPayload, Vec<i32>, Vec<u64>, Vec<u16>)> {
    let cells = actor_cells(runtime, actor_id)?;
    let (min_x, max_x, min_y, max_y) = actor_bounds(&cells)?;
    let width = (max_x - min_x + 1) as usize;
    let height = (max_y - min_y + 1) as usize;
    let child_cell_count = width.checked_mul(height)?;
    let mut words = vec![0u64; child_cell_count.div_ceil(64)];
    let mut materials = vec![0u16; child_cell_count];
    let mut source_indices = Vec::with_capacity(cells.len());
    for (coord, source_index, fallback_material) in cells {
        source_indices.push(source_index.min(i32::MAX as usize) as i32);
        let child_x = (coord.x - min_x) as usize;
        let child_y = (coord.y - min_y) as usize;
        let child_index = child_y * width + child_x;
        words[child_index >> 6] |= 1u64 << (child_index & 63);
        let material = material_ids
            .get(source_index)
            .copied()
            .unwrap_or(fallback_material);
        materials[child_index] = material;
    }
    let actor = runtime.family().actor(actor_id)?;
    let cropped_com = Vector::new(
        actor.local_com.x - min_x as f32,
        actor.local_com.y - min_y as f32,
    );
    let local_origin = pixel_shape_local_origin_for_com(
        width as u32,
        height as u32,
        runtime.asset().core().voxel_size(),
        cropped_com,
    );
    let row = CroppedSplitPayload {
        source_min_x: min_x.min(i32::MAX as u32) as i32,
        source_min_y: min_y.min(i32::MAX as u32) as i32,
        source_max_x: max_x.min(i32::MAX as u32) as i32,
        source_max_y: max_y.min(i32::MAX as u32) as i32,
        source_cell_count: source_indices.len(),
        child_width: width.min(i32::MAX as usize) as i32,
        child_height: height.min(i32::MAX as usize) as i32,
        child_pixel_size: runtime.asset().core().voxel_size(),
        child_local_origin: ffi_vec(local_origin),
        child_occupancy_word_count: words.len(),
        child_material_id_count: materials.len(),
        child_solid_count: source_indices.len(),
        child_topology_revision: topology_revision,
        child_topology_version: topology_version,
        ..CroppedSplitPayload::default()
    };
    Some((row, source_indices, words, materials))
}

fn split_event_row_from_cropped_payload(row: CroppedSplitPayload) -> AlchemyRapierSplitEventRow {
    AlchemyRapierSplitEventRow {
        transition_id: row.transition_id,
        source_kind: row.source_kind,
        source_body_handle: row.source_body_handle,
        source_body_packed_id: row.source_body_packed_id,
        source_collider_handle: row.source_collider_handle,
        source_collider_packed_id: row.source_collider_packed_id,
        child_body_handle: row.child_body_handle,
        child_body_packed_id: row.child_body_packed_id,
        child_collider_handle: row.child_collider_handle,
        child_collider_packed_id: row.child_collider_packed_id,
        source_terrain_actor_key: row.source_terrain_actor_key,
        source_terrain_chunk_x: row.source_terrain_chunk_x,
        source_terrain_chunk_y: row.source_terrain_chunk_y,
        source_terrain_world_origin_x: row.source_terrain_world_origin_x,
        source_terrain_world_origin_y: row.source_terrain_world_origin_y,
        source_terrain_revision: row.source_terrain_revision,
        source_width: row.source_width,
        source_height: row.source_height,
        source_min_x: row.source_min_x,
        source_min_y: row.source_min_y,
        source_max_x: row.source_max_x,
        source_max_y: row.source_max_y,
        source_cell_count: row.source_cell_count,
        source_touched_cell_count: row.source_touched_cell_count,
        source_removed_cell_count: row.source_removed_cell_count,
        source_local_origin: row.source_local_origin,
        source_solid_count: row.source_solid_count,
        child_width: row.child_width,
        child_height: row.child_height,
        child_pixel_size: row.child_pixel_size,
        child_local_origin: row.child_local_origin,
        child_occupancy_word_count: row.child_occupancy_word_count,
        child_material_id_count: row.child_material_id_count,
        child_solid_count: row.child_solid_count,
        position: row.position,
        rotation: row.rotation,
        linear_velocity: row.linear_velocity,
        angular_velocity: row.angular_velocity,
        source_topology_revision: row.source_topology_revision,
        source_topology_version: row.source_topology_version,
        child_topology_revision: row.child_topology_revision,
        child_topology_version: row.child_topology_version,
        material_hash: row.material_hash,
    }
}

fn cropped_pixel_asset_from_words(
    width: u32,
    height: u32,
    pixel_size: f32,
    occupancy_words: &[u64],
    material_ids: &[u16],
) -> Option<AuthoredVoxelAsset> {
    let cell_count = (width as usize).checked_mul(height as usize)?;
    if material_ids.len() != cell_count {
        return None;
    }
    let mut occupancy = vec![false; cell_count];
    let mut fracture_material = vec![0u16; cell_count];
    let mut contact_material = vec![0u16; cell_count];
    let external_id = vec![0u32; cell_count];
    for cell_index in 0..cell_count {
        let word_index = cell_index >> 6;
        let bit_index = cell_index & 63;
        let occupied = occupancy_words
            .get(word_index)
            .map_or(false, |word| ((word >> bit_index) & 1) != 0);
        if occupied {
            let material = material_ids[cell_index];
            occupancy[cell_index] = true;
            fracture_material[cell_index] = material;
            contact_material[cell_index] = material;
        }
    }
    author_voxel_asset(VoxelAuthoringInput::new(
        width,
        height,
        pixel_size,
        occupancy,
        fracture_material,
        contact_material,
        external_id,
    ))
    .ok()
}

fn material_hash(material_ids: &[u16]) -> u64 {
    let mut hash = 14695981039346656037u64;
    for material in material_ids {
        hash ^= u64::from(*material);
        hash = hash.wrapping_mul(1099511628211u64);
    }
    hash
}

fn actor_host_local_origin(runtime: &VoxelRuntime, actor_id: FxActorId) -> AlchemyRapierVec2 {
    actor_com(runtime, actor_id)
        .map(|com| {
            ffi_vec(pixel_shape_local_origin_for_com(
                runtime.asset().core().occupancy().width(),
                runtime.asset().core().occupancy().height(),
                runtime.asset().core().voxel_size(),
                com,
            ))
        })
        .unwrap_or_default()
}

fn actor_solid_count(runtime: &VoxelRuntime, actor_id: FxActorId) -> usize {
    actor_cells(runtime, actor_id).map_or(0, |cells| cells.len())
}

fn actor_scoped_asset(
    runtime: &VoxelRuntime,
    actor_id: FxActorId,
    material_ids: &[u16],
) -> Option<AuthoredVoxelAsset> {
    let width = runtime.asset().core().occupancy().width();
    let height = runtime.asset().core().occupancy().height();
    let cell_count = (width as usize).checked_mul(height as usize)?;
    let mut occupancy = vec![false; cell_count];
    let mut fracture_material = vec![0u16; cell_count];
    let mut contact_material = vec![0u16; cell_count];
    let external_id = vec![0u32; cell_count];
    for (_, index, fallback_material) in actor_cells(runtime, actor_id)? {
        if index >= cell_count {
            return None;
        }
        let material = material_ids
            .get(index)
            .copied()
            .unwrap_or(fallback_material);
        occupancy[index] = true;
        fracture_material[index] = material;
        contact_material[index] = material;
    }
    author_voxel_asset(VoxelAuthoringInput::new(
        width,
        height,
        runtime.asset().core().voxel_size(),
        occupancy,
        fracture_material,
        contact_material,
        external_id,
    ))
    .ok()
}

fn first_actor_id(runtime: &VoxelRuntime) -> FxActorId {
    runtime
        .family()
        .actors()
        .next()
        .map(|(actor_id, _)| *actor_id)
        .unwrap_or(FxActorId(0))
}

fn actor_com(runtime: &VoxelRuntime, actor_id: FxActorId) -> Option<Vector> {
    runtime
        .family()
        .actor(actor_id)
        .map(|actor| Vector::new(actor.local_com.x, actor.local_com.y))
}

fn asset_point_to_world(
    body_pose: &Pose,
    width: u32,
    height: u32,
    pixel_size: f32,
    old_local_origin: Vector,
    asset_point: Vector,
) -> Vector {
    body_pose.translation
        + body_pose.rotation
            * asset_point_to_body_local(width, height, pixel_size, old_local_origin, asset_point)
}

fn try_apply_dirty_pixel_split(
    world: &mut AlchemyRapierWorldInner,
    body_handle: RigidBodyHandle,
    desc: AlchemyRapierPixelRigidbodyDesc,
    new_occupancy: &[bool],
    new_material_ids: &[u16],
    mut state: PixelRigidbodyState,
) -> Option<AlchemyRapierPixelRigidbodyResult> {
    if desc.update_kind != 2
        || state.width != desc.width as u32
        || state.height != desc.height as u32
        || (state.pixel_size - desc.pixel_size).abs() > 0.000001
    {
        world.pixel_rigidbodies.insert(body_handle, state);
        return None;
    }

    let old_occupancy = state.asset.occupancy();
    if old_occupancy.len() != new_occupancy.len() {
        world.pixel_rigidbodies.insert(body_handle, state);
        return None;
    }

    let width = state.width;
    let mut removed = Vec::new();
    for (index, (old_solid, new_solid)) in
        old_occupancy.iter().zip(new_occupancy.iter()).enumerate()
    {
        if *old_solid && !*new_solid {
            removed.push(GridCoord {
                x: (index % width as usize) as u32,
                y: (index / width as usize) as u32,
            });
        }
        if !*old_solid && *new_solid {
            world.pixel_rigidbodies.insert(body_handle, state);
            return None;
        }
    }
    if removed.is_empty() {
        world.pixel_rigidbodies.insert(body_handle, state);
        return None;
    }

    if state
        .runtime
        .apply_edit(RuntimeEdit::RemoveVoxels { voxels: removed })
        .is_err()
    {
        world.pixel_rigidbodies.insert(body_handle, state);
        return None;
    }
    let split_events = state.runtime.split_dirty_actors();
    let source_actor = state.actor;
    let source_com = actor_com(&state.runtime, source_actor)?;
    let source_local_origin =
        pixel_shape_local_origin_for_com(state.width, state.height, state.pixel_size, source_com);
    let source_collider = build_actor_collider(&state.runtime, source_actor, source_local_origin)?;

    let source_pose = {
        let body = world.bodies.get(body_handle)?;
        *body.position()
    };
    let source_linvel = world.bodies.get(body_handle)?.linvel();
    let source_angvel = world.bodies.get(body_handle)?.angvel();
    let source_rotation = source_pose.rotation.angle();
    let source_position = asset_point_to_world(
        &source_pose,
        state.width,
        state.height,
        state.pixel_size,
        state.local_origin,
        source_com,
    );
    state.topology_revision = desc.topology_revision;
    state.topology_version = desc.topology_version;
    state.material_ids = new_material_ids.to_vec();

    let _ = world
        .colliders
        .remove(state.collider, &mut world.islands, &mut world.bodies, true);
    if let Some(body) = world.bodies.get_mut(body_handle) {
        body.set_position(
            Pose::from_parts(source_position, source_pose.rotation),
            true,
        );
    }
    let source_collider_handle =
        world
            .colliders
            .insert_with_parent(source_collider, body_handle, &mut world.bodies);
    if let Some(body) = world.bodies.get_mut(body_handle) {
        body.set_additional_mass(0.0, true);
        body.recompute_mass_properties_from_colliders(&world.colliders);
    }

    let mut created_any = false;
    for event in split_events
        .iter()
        .filter(|event| event.parent_actor == source_actor)
    {
        created_any |= create_pending_split_events(
            world,
            body_handle,
            source_collider_handle,
            source_pose,
            &state,
            event,
            source_linvel,
            source_angvel,
            source_rotation,
            new_material_ids,
        );
    }

    let Some(source_asset) = actor_scoped_asset(&state.runtime, source_actor, new_material_ids)
    else {
        world.pixel_rigidbodies.insert(body_handle, state);
        return None;
    };
    let source_runtime = VoxelRuntime::instantiate(
        FxFamilyId(pack_body_handle(body_handle) as u32),
        source_asset.clone(),
    );
    let source_actor = first_actor_id(&source_runtime);
    state.asset = source_asset;
    state.runtime = source_runtime;
    state.actor = source_actor;
    state.collider = source_collider_handle;
    state.local_origin = source_local_origin;
    state.solid_count = actor_solid_count(&state.runtime, source_actor);
    let result = {
        let body = world.bodies.get(body_handle)?;
        pixel_result(
            AlchemyRapierStatus::Ok,
            source_collider_handle,
            state.solid_count,
            body,
        )
    };
    world.pixel_rigidbodies.insert(body_handle, state);
    if created_any {
        Some(result)
    } else {
        Some(result)
    }
}

#[allow(clippy::too_many_arguments)]
fn create_pending_split_events(
    world: &mut AlchemyRapierWorldInner,
    source_body_handle: RigidBodyHandle,
    source_collider_handle: ColliderHandle,
    source_pose: Pose,
    source_state: &PixelRigidbodyState,
    event: &SplitEvent,
    source_linvel: Vector,
    source_angvel: f32,
    source_rotation: f32,
    material_ids: &[u16],
) -> bool {
    if !world.bodies.contains(source_body_handle) {
        return false;
    }
    let Some(source_body) = world.bodies.get(source_body_handle) else {
        return false;
    };
    let source_can_sleep = body_can_sleep(source_body);
    let source_gravity_scale = source_body.gravity_scale();
    let mut created_any = false;
    for child_actor in &event.created_children {
        let Some((mut row, source_cells, child_words, child_materials)) =
            build_cropped_actor_payload(
                &source_state.runtime,
                *child_actor,
                material_ids,
                source_state.topology_revision,
                source_state.topology_version,
            )
        else {
            continue;
        };
        let child_local_origin = vector(row.child_local_origin);
        let Some(child_asset) = cropped_pixel_asset_from_words(
            row.child_width as u32,
            row.child_height as u32,
            row.child_pixel_size,
            &child_words,
            &child_materials,
        ) else {
            continue;
        };
        let Some(child_collider) = build_pixel_collider(&child_asset, child_local_origin) else {
            continue;
        };
        let child_half_extents = pixel_shape_half_extents(
            row.child_width.max(0) as u32,
            row.child_height.max(0) as u32,
            row.child_pixel_size,
        );
        let child_cropped_com = child_half_extents - child_local_origin;
        let child_source_com = Vector::new(
            row.source_min_x as f32 + child_cropped_com.x,
            row.source_min_y as f32 + child_cropped_com.y,
        );
        let child_position = asset_point_to_world(
            &source_pose,
            source_state.width,
            source_state.height,
            source_state.pixel_size,
            source_state.local_origin,
            child_source_com,
        );
        let child_body = world.bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(child_position)
                .rotation(source_rotation)
                .linvel(source_linvel)
                .angvel(source_angvel)
                .gravity_scale(source_gravity_scale)
                .can_sleep(source_can_sleep),
        );
        let child_collider_handle =
            world
                .colliders
                .insert_with_parent(child_collider, child_body, &mut world.bodies);
        if let Some(body) = world.bodies.get_mut(child_body) {
            body.set_additional_mass(0.0, true);
            body.recompute_mass_properties_from_colliders(&world.colliders);
        }
        let Some(body) = world.bodies.get(child_body) else {
            continue;
        };
        let child_runtime = VoxelRuntime::instantiate(
            FxFamilyId(pack_body_handle(child_body) as u32),
            child_asset.clone(),
        );
        let child_actor_state = first_actor_id(&child_runtime);
        let child_state = PixelRigidbodyState {
            asset: child_asset,
            runtime: child_runtime,
            actor: child_actor_state,
            collider: child_collider_handle,
            width: row.child_width.max(0) as u32,
            height: row.child_height.max(0) as u32,
            pixel_size: source_state.pixel_size,
            local_origin: child_local_origin,
            topology_revision: source_state.topology_revision,
            topology_version: source_state.topology_version,
            material_ids: child_materials.clone(),
            support_mask: vec![0u8; child_materials.len()],
            solid_count: source_cells.len(),
        };
        row.transition_id = event.event_id.0;
        row.source_kind = AlchemyRapierQuerySourceKind::DynamicPixelRigidbody;
        row.source_body_handle = handle_to_ffi(source_body_handle);
        row.source_body_packed_id = pack_body_handle(source_body_handle);
        row.source_collider_handle = collider_handle_to_ffi(source_collider_handle);
        row.source_collider_packed_id = pack_collider_handle(source_collider_handle);
        row.child_body_handle = handle_to_ffi(child_body);
        row.child_body_packed_id = pack_body_handle(child_body);
        row.child_collider_handle = collider_handle_to_ffi(child_collider_handle);
        row.child_collider_packed_id = pack_collider_handle(child_collider_handle);
        row.source_width = source_state.width.min(i32::MAX as u32) as i32;
        row.source_height = source_state.height.min(i32::MAX as u32) as i32;
        row.source_local_origin =
            actor_host_local_origin(&source_state.runtime, source_state.actor);
        row.source_solid_count = actor_solid_count(&source_state.runtime, source_state.actor);
        row.source_touched_cell_count = source_cells.len();
        row.position = ffi_vec(body.center_of_mass());
        row.rotation = body.rotation().angle();
        row.linear_velocity = ffi_vec(body.linvel());
        row.angular_velocity = body.angvel();
        row.source_topology_revision = source_state.topology_revision;
        row.source_topology_version = source_state.topology_version;
        row.child_topology_revision = source_state.topology_revision;
        row.child_topology_version = source_state.topology_version;
        row.material_hash = material_hash(material_ids);
        world.pixel_rigidbodies.insert(child_body, child_state);
        world.pending_split_events.push(PendingSplitEvent {
            row: split_event_row_from_cropped_payload(row),
            touched_source_cell_indices: source_cells.clone(),
            removed_source_cell_indices: Vec::new(),
            source_cell_indices: source_cells,
            child_occupancy_words: child_words,
            child_material_ids: child_materials,
        });
        created_any = true;
    }
    created_any
}

fn pixel_result(
    status: AlchemyRapierStatus,
    collider_handle: ColliderHandle,
    solid_count: usize,
    body: &RigidBody,
) -> AlchemyRapierPixelRigidbodyResult {
    AlchemyRapierPixelRigidbodyResult {
        status,
        collider_handle: collider_handle_to_ffi(collider_handle),
        collider_packed_id: pack_collider_handle(collider_handle),
        solid_count,
        shape_count: if solid_count > 0 { 1 } else { 0 },
        local_center_of_mass: ffi_vec(body.local_center_of_mass()),
        mass: body.mass(),
        inertia: body.mass_properties().local_mprops.principal_inertia(),
    }
}

fn terrain_cell_index(state: &TerrainChunkState, x: u32, y: u32) -> usize {
    (y as usize) * (state.width as usize) + (x as usize)
}

fn terrain_cell_occupied(state: &TerrainChunkState, x: i32, y: i32) -> bool {
    if x < 0 || y < 0 || x as u32 >= state.width || y as u32 >= state.height {
        return false;
    }
    state.occupancy[terrain_cell_index(state, x as u32, y as u32)]
}

fn terrain_body_origin(state: &TerrainChunkState) -> Vector {
    Vector::new(state.local_origin_x as f32, state.local_origin_y as f32)
}

fn terrain_asset_point_to_world(state: &TerrainChunkState, asset_point: Vector) -> Vector {
    terrain_body_origin(state)
        + asset_point_to_body_local(
            state.width,
            state.height,
            state.pixel_size,
            state.pixel_shape_local_origin,
            asset_point,
        )
}

fn terrain_point_to_asset_point(state: &TerrainChunkState, point: Vector) -> Vector {
    point - terrain_body_origin(state) - state.pixel_shape_local_origin
        + pixel_shape_half_extents(state.width, state.height, state.pixel_size)
}

fn terrain_world_cell(state: &TerrainChunkState, point: Vector) -> (i32, i32) {
    let asset_point = terrain_point_to_asset_point(state, point);
    let local_x = (asset_point.x / state.pixel_size).floor() as i32;
    let local_y = (asset_point.y / state.pixel_size).floor() as i32;
    if terrain_cell_occupied(state, local_x, local_y) {
        return (
            state.source_world_origin_x + local_x,
            state.source_world_origin_y + local_y,
        );
    }

    let epsilon = state.pixel_size.max(1.0) * 0.0001;
    for y in (local_y - 1)..=(local_y + 1) {
        for x in (local_x - 1)..=(local_x + 1) {
            if !terrain_cell_occupied(state, x, y) {
                continue;
            }
            let min = terrain_asset_point_to_world(
                state,
                Vector::new(x as f32 * state.pixel_size, y as f32 * state.pixel_size),
            );
            let min_x = min.x;
            let min_y = min.y;
            let max_x = min_x + state.pixel_size;
            let max_y = min_y + state.pixel_size;
            if point.x >= min_x - epsilon
                && point.x <= max_x + epsilon
                && point.y >= min_y - epsilon
                && point.y <= max_y + epsilon
            {
                return (
                    state.source_world_origin_x + x,
                    state.source_world_origin_y + y,
                );
            }
        }
    }

    let mut best_x = 0;
    let mut best_y = 0;
    let mut best_distance_sq = f32::INFINITY;
    for y in 0..state.height {
        for x in 0..state.width {
            if !state.occupancy[terrain_cell_index(state, x, y)] {
                continue;
            }
            let center = terrain_asset_point_to_world(
                state,
                Vector::new(
                    (x as f32 + 0.5) * state.pixel_size,
                    (y as f32 + 0.5) * state.pixel_size,
                ),
            );
            let distance_sq = (point - center).length_squared();
            if distance_sq < best_distance_sq {
                best_distance_sq = distance_sq;
                best_x = x as i32;
                best_y = y as i32;
            }
        }
    }
    (
        state.source_world_origin_x + best_x,
        state.source_world_origin_y + best_y,
    )
}

fn terrain_local_point(state: &TerrainChunkState, point: Vector) -> Vector {
    point - terrain_body_origin(state)
}

fn terrain_actor_body_origin(state: &TerrainFractureActorState) -> Vector {
    Vector::new(state.local_origin_x as f32, state.local_origin_y as f32)
}

fn terrain_actor_asset_point_to_world(
    state: &TerrainFractureActorState,
    asset_point: Vector,
) -> Vector {
    terrain_actor_body_origin(state)
        + asset_point_to_body_local(
            state.width,
            state.height,
            state.pixel_size,
            state.pixel_shape_local_origin,
            asset_point,
        )
}

fn terrain_actor_point_to_asset_point(state: &TerrainFractureActorState, point: Vector) -> Vector {
    point - terrain_actor_body_origin(state) - state.pixel_shape_local_origin
        + pixel_shape_half_extents(state.width, state.height, state.pixel_size)
}

fn terrain_actor_world_cell(state: &TerrainFractureActorState, point: Vector) -> (i32, i32) {
    let asset_point = terrain_actor_point_to_asset_point(state, point);
    let local_x = (asset_point.x / state.pixel_size).floor() as i32;
    let local_y = (asset_point.y / state.pixel_size).floor() as i32;
    if local_x >= 0
        && local_y >= 0
        && (local_x as u32) < state.width
        && (local_y as u32) < state.height
    {
        return (
            state.source_world_origin_x + local_x,
            state.source_world_origin_y + local_y,
        );
    }

    let mut best_x = 0;
    let mut best_y = 0;
    let mut best_distance_sq = f32::INFINITY;
    for y in 0..state.height {
        for x in 0..state.width {
            let idx = (y as usize) * (state.width as usize) + (x as usize);
            if !state.asset.occupancy().get(idx).copied().unwrap_or(false) {
                continue;
            }
            let center = terrain_actor_asset_point_to_world(
                state,
                Vector::new(
                    (x as f32 + 0.5) * state.pixel_size,
                    (y as f32 + 0.5) * state.pixel_size,
                ),
            );
            let distance_sq = (point - center).length_squared();
            if distance_sq < best_distance_sq {
                best_distance_sq = distance_sq;
                best_x = x as i32;
                best_y = y as i32;
            }
        }
    }
    (
        state.source_world_origin_x + best_x,
        state.source_world_origin_y + best_y,
    )
}

fn terrain_actor_local_point(state: &TerrainFractureActorState, point: Vector) -> Vector {
    point - terrain_actor_body_origin(state)
}

fn is_alchemy_rigidbody_query_body_type(body_type: RigidBodyType) -> bool {
    matches!(
        body_type,
        RigidBodyType::Dynamic
            | RigidBodyType::KinematicPositionBased
            | RigidBodyType::KinematicVelocityBased
    )
}

fn collider_supports_kinematic_alchemy_query(collider: &Collider) -> bool {
    collider.shape().as_compound().is_some()
}

fn alchemy_rigidbody_query_body(
    world: &AlchemyRapierWorldInner,
    collider: &Collider,
    ignored_body: Option<RigidBodyHandle>,
) -> Option<RigidBodyHandle> {
    let body_handle = collider.parent()?;
    if ignored_body == Some(body_handle) {
        return None;
    }
    let body = world.bodies.get(body_handle)?;
    let body_type = body.body_type();
    if is_alchemy_rigidbody_query_body_type(body_type) {
        if body_type != RigidBodyType::Dynamic
            && !collider_supports_kinematic_alchemy_query(collider)
        {
            return None;
        }
        Some(body_handle)
    } else {
        None
    }
}

enum QueryTarget {
    Dynamic(RigidBodyHandle),
    TerrainChunk(TerrainKey),
    TerrainFractureActor(i64),
    VoxelStaticTerrain(VoxelTerrainSourceMetadata),
}

fn query_target(
    world: &AlchemyRapierWorldInner,
    collider_handle: ColliderHandle,
    collider: &Collider,
    ignored_body: Option<RigidBodyHandle>,
    source_mask: u32,
) -> Option<QueryTarget> {
    if (source_mask & QUERY_SOURCE_DYNAMIC_RIGIDBODY) != 0 {
        if let Some(body_handle) = alchemy_rigidbody_query_body(world, collider, ignored_body) {
            return Some(QueryTarget::Dynamic(body_handle));
        }
    }
    if (source_mask & QUERY_SOURCE_TERRAIN) != 0 {
        if let Some(key) = world.terrain_by_collider.get(&collider_handle) {
            return Some(QueryTarget::TerrainChunk(*key));
        }
        if let Some(actor_key) = world
            .terrain_fracture_actor_by_collider
            .get(&collider_handle)
        {
            return Some(QueryTarget::TerrainFractureActor(*actor_key));
        }
        if let Some(source) = world.voxel_terrain_sources.get(&collider_handle) {
            return Some(QueryTarget::VoxelStaticTerrain(*source));
        }
    }
    None
}

fn make_dynamic_query_hit(
    world: &AlchemyRapierWorldInner,
    body_handle: RigidBodyHandle,
    collider_handle: ColliderHandle,
    point: Vector,
    normal: Vector,
    distance: f32,
    fraction: f32,
) -> Option<AlchemyRapierQueryHit> {
    let body = world.bodies.get(body_handle)?;
    Some(AlchemyRapierQueryHit {
        source_kind: AlchemyRapierQuerySourceKind::DynamicPixelRigidbody,
        body_packed_id: pack_body_handle(body_handle),
        collider_packed_id: pack_collider_handle(collider_handle),
        terrain_chunk_x: 0,
        terrain_chunk_y: 0,
        terrain_revision: -1,
        terrain_actor_key: 0,
        world_cell_x: point.x.floor() as i32,
        world_cell_y: point.y.floor() as i32,
        point: ffi_vec(point),
        normal: ffi_vec(normalized_or_zero(normal)),
        local_point: ffi_vec(body.position().inverse_transform_point(point)),
        point_velocity: ffi_vec(body.velocity_at_point(point)),
        distance,
        fraction,
    })
}

fn make_terrain_query_hit(
    world: &AlchemyRapierWorldInner,
    key: TerrainKey,
    collider_handle: ColliderHandle,
    point: Vector,
    normal: Vector,
    distance: f32,
    fraction: f32,
) -> Option<AlchemyRapierQueryHit> {
    let state = world.terrain_chunks.get(&key)?;
    let (world_cell_x, world_cell_y) = terrain_world_cell(state, point);
    Some(AlchemyRapierQueryHit {
        source_kind: AlchemyRapierQuerySourceKind::StaticTerrain,
        body_packed_id: 0,
        collider_packed_id: pack_collider_handle(collider_handle),
        terrain_chunk_x: state.chunk_x,
        terrain_chunk_y: state.chunk_y,
        terrain_revision: state.revision,
        terrain_actor_key: 0,
        world_cell_x,
        world_cell_y,
        point: ffi_vec(point),
        normal: ffi_vec(normalized_or_zero(normal)),
        local_point: ffi_vec(terrain_local_point(state, point)),
        point_velocity: AlchemyRapierVec2::default(),
        distance,
        fraction,
    })
}

fn make_terrain_actor_query_hit(
    world: &AlchemyRapierWorldInner,
    actor_key: i64,
    collider_handle: ColliderHandle,
    point: Vector,
    normal: Vector,
    distance: f32,
    fraction: f32,
) -> Option<AlchemyRapierQueryHit> {
    let state = world.terrain_fracture_actors.get(&actor_key)?;
    let (world_cell_x, world_cell_y) = terrain_actor_world_cell(state, point);
    Some(AlchemyRapierQueryHit {
        source_kind: AlchemyRapierQuerySourceKind::StaticTerrain,
        body_packed_id: pack_body_handle(state.body),
        collider_packed_id: pack_collider_handle(collider_handle),
        terrain_chunk_x: state.chunk_x,
        terrain_chunk_y: state.chunk_y,
        terrain_revision: state.revision,
        terrain_actor_key: actor_key,
        world_cell_x,
        world_cell_y,
        point: ffi_vec(point),
        normal: ffi_vec(normalized_or_zero(normal)),
        local_point: ffi_vec(terrain_actor_local_point(state, point)),
        point_velocity: AlchemyRapierVec2::default(),
        distance,
        fraction,
    })
}

fn make_voxel_terrain_query_hit(
    world: &AlchemyRapierWorldInner,
    source: VoxelTerrainSourceMetadata,
    collider_handle: ColliderHandle,
    point: Vector,
    normal: Vector,
    distance: f32,
    fraction: f32,
) -> Option<AlchemyRapierQueryHit> {
    let collider = world.colliders.get(collider_handle)?;
    Some(AlchemyRapierQueryHit {
        source_kind: AlchemyRapierQuerySourceKind::StaticTerrain,
        body_packed_id: 0,
        collider_packed_id: pack_collider_handle(collider_handle),
        terrain_chunk_x: source.chunk_x,
        terrain_chunk_y: source.chunk_y,
        terrain_revision: source.revision,
        terrain_actor_key: 0,
        world_cell_x: point.x.floor() as i32,
        world_cell_y: point.y.floor() as i32,
        point: ffi_vec(point),
        normal: ffi_vec(normalized_or_zero(normal)),
        local_point: ffi_vec(collider.position().inverse_transform_point(point)),
        point_velocity: AlchemyRapierVec2::default(),
        distance,
        fraction,
    })
}

fn make_query_hit(
    world: &AlchemyRapierWorldInner,
    target: QueryTarget,
    collider_handle: ColliderHandle,
    point: Vector,
    normal: Vector,
    distance: f32,
    fraction: f32,
) -> Option<AlchemyRapierQueryHit> {
    match target {
        QueryTarget::Dynamic(body_handle) => make_dynamic_query_hit(
            world,
            body_handle,
            collider_handle,
            point,
            normal,
            distance,
            fraction,
        ),
        QueryTarget::TerrainChunk(key) => make_terrain_query_hit(
            world,
            key,
            collider_handle,
            point,
            normal,
            distance,
            fraction,
        ),
        QueryTarget::TerrainFractureActor(actor_key) => make_terrain_actor_query_hit(
            world,
            actor_key,
            collider_handle,
            point,
            normal,
            distance,
            fraction,
        ),
        QueryTarget::VoxelStaticTerrain(source) => make_voxel_terrain_query_hit(
            world,
            source,
            collider_handle,
            point,
            normal,
            distance,
            fraction,
        ),
    }
}

fn write_query_hit(
    hit: AlchemyRapierQueryHit,
    hits: *mut AlchemyRapierQueryHit,
    hit_capacity: usize,
    hit_count: &mut usize,
    written_count: &mut usize,
) {
    if *written_count < hit_capacity {
        unsafe {
            *hits.add(*written_count) = hit;
        }
        *written_count += 1;
    }
    *hit_count += 1;
}

fn query_output_valid(hits: *mut AlchemyRapierQueryHit, hit_capacity: usize) -> bool {
    hit_capacity == 0 || !hits.is_null()
}

fn closest_capsule_axis_point(point: Vector, origin: Vector, half_height: f32) -> Vector {
    Vector::new(
        origin.x,
        point
            .y
            .max(origin.y - half_height)
            .min(origin.y + half_height),
    )
}

fn capsule_overlap_hit_point(
    collider: &Collider,
    origin: Vector,
    half_height: f32,
) -> (Vector, Vector, f32) {
    let aabb = collider.shape().compute_aabb(collider.position());
    let aabb_center_y = 0.5 * (aabb.mins.y + aabb.maxs.y);
    let sample_points = [
        Vector::new(origin.x, origin.y - half_height),
        Vector::new(origin.x, origin.y + half_height),
        Vector::new(
            origin.x,
            aabb_center_y
                .max(origin.y - half_height)
                .min(origin.y + half_height),
        ),
    ];

    let mut best_point = sample_points[0];
    let mut best_axis_point = sample_points[0];
    let mut best_distance_sq = f32::INFINITY;
    for sample in sample_points {
        let projection = collider
            .shape()
            .project_point(collider.position(), sample, true);
        let shape_point = projection.point;
        let axis_point = if projection.is_inside {
            sample
        } else {
            closest_capsule_axis_point(shape_point, origin, half_height)
        };
        let distance_sq = (shape_point - axis_point).length_squared();
        if distance_sq < best_distance_sq {
            best_distance_sq = distance_sq;
            best_point = shape_point;
            best_axis_point = axis_point;
        }
    }

    let distance = best_distance_sq.max(0.0).sqrt();
    let normal = if distance > 0.000001 {
        (best_point - best_axis_point) / distance
    } else {
        Vector::ZERO
    };
    (best_point, normal, distance)
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_create_world() -> AlchemyRapierCreateWorldResult {
    match catch_unwind(AssertUnwindSafe(AlchemyRapierWorldInner::new)) {
        Ok(world) => AlchemyRapierCreateWorldResult {
            status: AlchemyRapierStatus::Ok,
            world: Box::into_raw(Box::new(world)).cast::<AlchemyRapierWorld>(),
        },
        Err(_) => AlchemyRapierCreateWorldResult {
            status: AlchemyRapierStatus::Panic,
            world: ptr::null_mut(),
        },
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_destroy_world(
    world: *mut AlchemyRapierWorld,
) -> AlchemyRapierStatus {
    if world.is_null() {
        return AlchemyRapierStatus::NullPointer;
    }

    match catch_unwind(AssertUnwindSafe(|| unsafe {
        drop(Box::from_raw(world.cast::<AlchemyRapierWorldInner>()));
    })) {
        Ok(()) => AlchemyRapierStatus::Ok,
        Err(_) => AlchemyRapierStatus::Panic,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_set_gravity(
    world: *mut AlchemyRapierWorld,
    gravity: AlchemyRapierVec2,
) -> AlchemyRapierStatus {
    match catch_unwind(AssertUnwindSafe(|| {
        if !gravity.x.is_finite() || !gravity.y.is_finite() {
            return AlchemyRapierStatus::InvalidArgument;
        }
        let Ok(world) = to_inner(world) else {
            return AlchemyRapierStatus::NullPointer;
        };
        world.gravity = Vector::new(gravity.x, gravity.y);
        AlchemyRapierStatus::Ok
    })) {
        Ok(status) => status,
        Err(_) => AlchemyRapierStatus::Panic,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_set_max_ccd_substeps(
    world: *mut AlchemyRapierWorld,
    max_ccd_substeps: u32,
) -> AlchemyRapierStatus {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return AlchemyRapierStatus::NullPointer;
        };
        if max_ccd_substeps == 0 {
            return AlchemyRapierStatus::InvalidArgument;
        }
        world.integration_parameters.max_ccd_substeps = max_ccd_substeps as usize;
        AlchemyRapierStatus::Ok
    })) {
        Ok(status) => status,
        Err(_) => AlchemyRapierStatus::Panic,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_step(
    world: *mut AlchemyRapierWorld,
    time_step: f32,
    sub_step_count: i32,
) -> AlchemyRapierStepResult {
    step_with_contact_callback(world, time_step, sub_step_count, None, ptr::null_mut())
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_step_with_contact_callback(
    world: *mut AlchemyRapierWorld,
    time_step: f32,
    sub_step_count: i32,
    callback: AlchemyRapierContactCallback,
    user_data: *mut c_void,
) -> AlchemyRapierStepResult {
    step_with_contact_callback(world, time_step, sub_step_count, callback, user_data)
}

fn step_with_contact_callback(
    world: *mut AlchemyRapierWorld,
    time_step: f32,
    sub_step_count: i32,
    callback: AlchemyRapierContactCallback,
    user_data: *mut c_void,
) -> AlchemyRapierStepResult {
    match catch_unwind(AssertUnwindSafe(|| {
        if !time_step.is_finite() || time_step <= 0.0 || sub_step_count <= 0 {
            return empty_step_result(AlchemyRapierStatus::InvalidArgument);
        }
        let Ok(world) = to_inner(world) else {
            return empty_step_result(AlchemyRapierStatus::NullPointer);
        };
        let sub_steps = sub_step_count as usize;
        let dt = time_step / sub_steps as f32;
        let mut active_pairs = HashSet::new();
        let mut contact_hit_count = 0usize;
        for _ in 0..sub_steps {
            world.step_once(dt);
            if callback.is_some() {
                contact_hit_count = contact_hit_count
                    .saturating_add(emit_contact_samples(world, callback, user_data));
            }
            active_pairs = collect_active_contact_pairs(world);
        }

        let contact_begin_count = active_pairs
            .difference(&world.previous_active_contact_pairs)
            .count();
        let contact_end_count = world
            .previous_active_contact_pairs
            .difference(&active_pairs)
            .count();
        world.previous_active_contact_pairs = active_pairs;

        AlchemyRapierStepResult {
            status: AlchemyRapierStatus::Ok,
            contact_begin_count,
            contact_end_count,
            contact_hit_count,
        }
    })) {
        Ok(result) => result,
        Err(_) => empty_step_result(AlchemyRapierStatus::Panic),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_set_contact_materials(
    world: *mut AlchemyRapierWorld,
    materials: *const AlchemyRapierContactMaterialDesc,
    material_count: usize,
) -> AlchemyRapierStatus {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return AlchemyRapierStatus::NullPointer;
        };
        if material_count > 0 && materials.is_null() {
            return AlchemyRapierStatus::NullPointer;
        }

        world.contact_materials.clear();
        if material_count == 0 {
            return AlchemyRapierStatus::Ok;
        }

        let source = unsafe { slice::from_raw_parts(materials, material_count) };
        for material in source {
            if !material.friction.is_finite()
                || !material.restitution.is_finite()
                || !material.hardness.is_finite()
            {
                return AlchemyRapierStatus::InvalidArgument;
            }

            world.contact_materials.insert(
                material.material_id,
                ContactMaterial {
                    friction: material.friction,
                    restitution: material.restitution,
                    friction_combine_rule: combine_rule(material.friction_combine_rule),
                    restitution_combine_rule: combine_rule(material.restitution_combine_rule),
                    hardness: material.hardness,
                },
            );
        }
        AlchemyRapierStatus::Ok
    })) {
        Ok(status) => status,
        Err(_) => AlchemyRapierStatus::Panic,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_create_body(
    world: *mut AlchemyRapierWorld,
    desc: AlchemyRapierBodyDesc,
) -> AlchemyRapierCreateBodyResult {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return AlchemyRapierCreateBodyResult {
                status: AlchemyRapierStatus::NullPointer,
                handle: AlchemyRapierRigidBodyHandle::default(),
                packed_id: 0,
            };
        };
        let handle = world.bodies.insert(body_builder(desc));
        AlchemyRapierCreateBodyResult {
            status: AlchemyRapierStatus::Ok,
            handle: handle_to_ffi(handle),
            packed_id: pack_body_handle(handle),
        }
    })) {
        Ok(result) => result,
        Err(_) => AlchemyRapierCreateBodyResult {
            status: AlchemyRapierStatus::Panic,
            handle: AlchemyRapierRigidBodyHandle::default(),
            packed_id: 0,
        },
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_update_body(
    world: *mut AlchemyRapierWorld,
    handle: AlchemyRapierRigidBodyHandle,
    desc: AlchemyRapierBodyDesc,
) -> AlchemyRapierStatus {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return AlchemyRapierStatus::NullPointer;
        };
        let handle = handle_from_ffi(handle);
        let Some(body) = world.bodies.get_mut(handle) else {
            return AlchemyRapierStatus::InvalidHandle;
        };
        apply_body_desc(body, desc);
        body.recompute_mass_properties_from_colliders(&world.colliders);
        AlchemyRapierStatus::Ok
    })) {
        Ok(status) => status,
        Err(_) => AlchemyRapierStatus::Panic,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_set_body_user_data(
    world: *mut AlchemyRapierWorld,
    handle: AlchemyRapierRigidBodyHandle,
    user_data: u64,
) -> AlchemyRapierStatus {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return AlchemyRapierStatus::NullPointer;
        };
        let handle = handle_from_ffi(handle);
        let Some(body) = world.bodies.get_mut(handle) else {
            return AlchemyRapierStatus::InvalidHandle;
        };
        body.user_data = user_data as u128;
        AlchemyRapierStatus::Ok
    })) {
        Ok(status) => status,
        Err(_) => AlchemyRapierStatus::Panic,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_set_body_ccd_enabled(
    world: *mut AlchemyRapierWorld,
    handle: AlchemyRapierRigidBodyHandle,
    enabled: u8,
) -> AlchemyRapierStatus {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return AlchemyRapierStatus::NullPointer;
        };
        let Some(body) = world.bodies.get_mut(handle_from_ffi(handle)) else {
            return AlchemyRapierStatus::InvalidHandle;
        };
        body.enable_ccd(enabled != 0);
        AlchemyRapierStatus::Ok
    })) {
        Ok(status) => status,
        Err(_) => AlchemyRapierStatus::Panic,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_set_body_next_kinematic_position(
    world: *mut AlchemyRapierWorld,
    handle: AlchemyRapierRigidBodyHandle,
    position: AlchemyRapierVec2,
    rotation: f32,
) -> AlchemyRapierStatus {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return AlchemyRapierStatus::NullPointer;
        };
        if !position.x.is_finite() || !position.y.is_finite() || !rotation.is_finite() {
            return AlchemyRapierStatus::InvalidArgument;
        }
        let handle = handle_from_ffi(handle);
        let Some(body) = world.bodies.get_mut(handle) else {
            return AlchemyRapierStatus::InvalidHandle;
        };
        body.set_next_kinematic_position(Pose::from_parts(
            vector(position),
            Rotation::new(rotation),
        ));
        AlchemyRapierStatus::Ok
    })) {
        Ok(status) => status,
        Err(_) => AlchemyRapierStatus::Panic,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_destroy_body(
    world: *mut AlchemyRapierWorld,
    handle: AlchemyRapierRigidBodyHandle,
) -> AlchemyRapierStatus {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return AlchemyRapierStatus::NullPointer;
        };
        let handle = handle_from_ffi(handle);
        remove_pixel_rigidbody(world, handle);
        if world
            .bodies
            .remove(
                handle,
                &mut world.islands,
                &mut world.colliders,
                &mut world.impulse_joints,
                &mut world.multibody_joints,
                true,
            )
            .is_some()
        {
            AlchemyRapierStatus::Ok
        } else {
            AlchemyRapierStatus::InvalidHandle
        }
    })) {
        Ok(status) => status,
        Err(_) => AlchemyRapierStatus::Panic,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_clear_body_colliders(
    world: *mut AlchemyRapierWorld,
    handle: AlchemyRapierRigidBodyHandle,
) -> AlchemyRapierStatus {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return AlchemyRapierStatus::NullPointer;
        };
        let body_handle = handle_from_ffi(handle);
        let Some(body) = world.bodies.get(body_handle) else {
            return AlchemyRapierStatus::InvalidHandle;
        };
        world.pixel_rigidbodies.remove(&body_handle);
        let colliders = body.colliders().to_vec();
        for collider in colliders {
            remove_voxel_collider_metadata(world, collider);
            let _ = world
                .colliders
                .remove(collider, &mut world.islands, &mut world.bodies, true);
        }
        recompute_body_mass(world, body_handle);
        AlchemyRapierStatus::Ok
    })) {
        Ok(status) => status,
        Err(_) => AlchemyRapierStatus::Panic,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_create_capsule_collider(
    world: *mut AlchemyRapierWorld,
    body_handle: AlchemyRapierRigidBodyHandle,
    radius: f32,
    half_height: f32,
) -> AlchemyRapierCreateColliderResult {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return AlchemyRapierCreateColliderResult {
                status: AlchemyRapierStatus::NullPointer,
                handle: AlchemyRapierColliderHandle::default(),
                packed_id: 0,
            };
        };
        if !radius.is_finite() || radius <= 0.0 || !half_height.is_finite() || half_height < 0.0 {
            return AlchemyRapierCreateColliderResult {
                status: AlchemyRapierStatus::InvalidArgument,
                handle: AlchemyRapierColliderHandle::default(),
                packed_id: 0,
            };
        }
        let body_handle = handle_from_ffi(body_handle);
        if !world.bodies.contains(body_handle) {
            return AlchemyRapierCreateColliderResult {
                status: AlchemyRapierStatus::InvalidHandle,
                handle: AlchemyRapierColliderHandle::default(),
                packed_id: 0,
            };
        }
        let collider = ColliderBuilder::capsule_y(half_height, radius)
            .density(0.0)
            .build();
        let handle = world
            .colliders
            .insert_with_parent(collider, body_handle, &mut world.bodies);
        recompute_body_mass(world, body_handle);
        AlchemyRapierCreateColliderResult {
            status: AlchemyRapierStatus::Ok,
            handle: collider_handle_to_ffi(handle),
            packed_id: pack_collider_handle(handle),
        }
    })) {
        Ok(result) => result,
        Err(_) => AlchemyRapierCreateColliderResult {
            status: AlchemyRapierStatus::Panic,
            handle: AlchemyRapierColliderHandle::default(),
            packed_id: 0,
        },
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_create_convex_collider(
    world: *mut AlchemyRapierWorld,
    body_handle: AlchemyRapierRigidBodyHandle,
    points: *const AlchemyRapierVec2,
    point_count: usize,
) -> AlchemyRapierCreateColliderResult {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return AlchemyRapierCreateColliderResult {
                status: AlchemyRapierStatus::NullPointer,
                handle: AlchemyRapierColliderHandle::default(),
                packed_id: 0,
            };
        };
        if points.is_null() || point_count < 3 {
            return AlchemyRapierCreateColliderResult {
                status: AlchemyRapierStatus::InvalidArgument,
                handle: AlchemyRapierColliderHandle::default(),
                packed_id: 0,
            };
        }
        let body_handle = handle_from_ffi(body_handle);
        if !world.bodies.contains(body_handle) {
            return AlchemyRapierCreateColliderResult {
                status: AlchemyRapierStatus::InvalidHandle,
                handle: AlchemyRapierColliderHandle::default(),
                packed_id: 0,
            };
        }
        let points = unsafe { slice::from_raw_parts(points, point_count) };
        let points = points
            .iter()
            .map(|point| vector(*point))
            .collect::<Vec<_>>();
        let Some(builder) = ColliderBuilder::convex_hull(&points) else {
            return AlchemyRapierCreateColliderResult {
                status: AlchemyRapierStatus::InvalidArgument,
                handle: AlchemyRapierColliderHandle::default(),
                packed_id: 0,
            };
        };
        let collider = builder.density(0.0).build();
        let handle = world
            .colliders
            .insert_with_parent(collider, body_handle, &mut world.bodies);
        recompute_body_mass(world, body_handle);
        AlchemyRapierCreateColliderResult {
            status: AlchemyRapierStatus::Ok,
            handle: collider_handle_to_ffi(handle),
            packed_id: pack_collider_handle(handle),
        }
    })) {
        Ok(result) => result,
        Err(_) => AlchemyRapierCreateColliderResult {
            status: AlchemyRapierStatus::Panic,
            handle: AlchemyRapierColliderHandle::default(),
            packed_id: 0,
        },
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_set_collider_material(
    world: *mut AlchemyRapierWorld,
    handle: AlchemyRapierColliderHandle,
    friction: f32,
    restitution: f32,
) -> AlchemyRapierStatus {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return AlchemyRapierStatus::NullPointer;
        };
        if !friction.is_finite()
            || friction < 0.0
            || !restitution.is_finite()
            || !(0.0..=1.0).contains(&restitution)
        {
            return AlchemyRapierStatus::InvalidArgument;
        }
        let handle = collider_handle_from_ffi(handle);
        let Some(collider) = world.colliders.get_mut(handle) else {
            return AlchemyRapierStatus::InvalidHandle;
        };
        collider.set_friction(friction);
        collider.set_restitution(restitution);
        AlchemyRapierStatus::Ok
    })) {
        Ok(status) => status,
        Err(_) => AlchemyRapierStatus::Panic,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_set_collider_collision_filter(
    world: *mut AlchemyRapierWorld,
    handle: AlchemyRapierColliderHandle,
    memberships: u32,
    filter: u32,
    self_collision_group: u64,
    self_collision_enabled: u8,
) -> AlchemyRapierStatus {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return AlchemyRapierStatus::NullPointer;
        };
        if memberships == 0 || self_collision_group == 0 || self_collision_enabled > 1 {
            return AlchemyRapierStatus::InvalidArgument;
        }
        let Some(collider) = world.colliders.get_mut(collider_handle_from_ffi(handle)) else {
            return AlchemyRapierStatus::InvalidHandle;
        };
        collider.set_collision_groups(InteractionGroups::new(
            Group::from_bits_retain(memberships),
            Group::from_bits_retain(filter),
            InteractionTestMode::And,
        ));
        collider.user_data = SELF_COLLISION_FILTER_TAG | self_collision_group as u128;
        let mut active_hooks = collider.active_hooks();
        active_hooks.set(
            ActiveHooks::FILTER_CONTACT_PAIRS,
            self_collision_enabled == 0,
        );
        collider.set_active_hooks(active_hooks);
        AlchemyRapierStatus::Ok
    })) {
        Ok(status) => status,
        Err(_) => AlchemyRapierStatus::Panic,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_rebuild_pixel_rigidbody(
    world: *mut AlchemyRapierWorld,
    body_handle: AlchemyRapierRigidBodyHandle,
    desc: AlchemyRapierPixelRigidbodyDesc,
) -> AlchemyRapierPixelRigidbodyResult {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return empty_pixel_rigidbody_result(AlchemyRapierStatus::NullPointer);
        };
        let body_handle = handle_from_ffi(body_handle);
        if !world.bodies.contains(body_handle) {
            return empty_pixel_rigidbody_result(AlchemyRapierStatus::InvalidHandle);
        }
        let (asset, material_ids, support_mask, solid_count) = match author_pixel_asset(desc) {
            Ok(value) => value,
            Err(status) => return empty_pixel_rigidbody_result(status),
        };
        let occupancy = asset.occupancy();
        if let Some(existing) = world.pixel_rigidbodies.remove(&body_handle) {
            if let Some(result) = try_apply_dirty_pixel_split(
                world,
                body_handle,
                desc,
                &occupancy,
                &material_ids,
                existing,
            ) {
                return result;
            }
        }
        let local_origin = vector(desc.local_origin);
        let Some(collider) = build_pixel_collider(&asset, local_origin) else {
            return empty_pixel_rigidbody_result(AlchemyRapierStatus::InvalidArgument);
        };

        clear_body_colliders(world, body_handle);
        let collider_handle =
            world
                .colliders
                .insert_with_parent(collider, body_handle, &mut world.bodies);
        if let Some(body) = world.bodies.get_mut(body_handle) {
            body.set_additional_mass(0.0, true);
            body.recompute_mass_properties_from_colliders(&world.colliders);
        }
        let result = {
            let Some(body) = world.bodies.get(body_handle) else {
                return empty_pixel_rigidbody_result(AlchemyRapierStatus::InvalidHandle);
            };
            pixel_result(AlchemyRapierStatus::Ok, collider_handle, solid_count, body)
        };
        let runtime = VoxelRuntime::instantiate(
            FxFamilyId(pack_body_handle(body_handle) as u32),
            asset.clone(),
        );
        let actor = first_actor_id(&runtime);
        world.pixel_rigidbodies.insert(
            body_handle,
            PixelRigidbodyState {
                asset,
                runtime,
                actor,
                collider: collider_handle,
                width: desc.width as u32,
                height: desc.height as u32,
                pixel_size: desc.pixel_size,
                local_origin,
                topology_revision: desc.topology_revision,
                topology_version: desc.topology_version,
                material_ids,
                support_mask,
                solid_count,
            },
        );
        result
    })) {
        Ok(result) => result,
        Err(_) => empty_pixel_rigidbody_result(AlchemyRapierStatus::Panic),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_rebuild_pixel_rigidbody_from_owned_asset(
    world: *mut AlchemyRapierWorld,
    body_handle: AlchemyRapierRigidBodyHandle,
    local_origin: AlchemyRapierVec2,
) -> AlchemyRapierPixelRigidbodyResult {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return empty_pixel_rigidbody_result(AlchemyRapierStatus::NullPointer);
        };
        let body_handle = handle_from_ffi(body_handle);
        if !world.bodies.contains(body_handle) {
            return empty_pixel_rigidbody_result(AlchemyRapierStatus::InvalidHandle);
        }
        let Some(mut state) = world.pixel_rigidbodies.remove(&body_handle) else {
            return empty_pixel_rigidbody_result(AlchemyRapierStatus::InvalidHandle);
        };
        if !local_origin.x.is_finite() || !local_origin.y.is_finite() {
            world.pixel_rigidbodies.insert(body_handle, state);
            return empty_pixel_rigidbody_result(AlchemyRapierStatus::InvalidArgument);
        }
        let new_local_origin = vector(local_origin);
        let Some(collider) = build_actor_collider(&state.runtime, state.actor, new_local_origin)
        else {
            world.pixel_rigidbodies.insert(body_handle, state);
            return empty_pixel_rigidbody_result(AlchemyRapierStatus::InvalidArgument);
        };

        let _ = world
            .colliders
            .remove(state.collider, &mut world.islands, &mut world.bodies, true);
        let collider_handle =
            world
                .colliders
                .insert_with_parent(collider, body_handle, &mut world.bodies);
        if let Some(body) = world.bodies.get_mut(body_handle) {
            body.set_additional_mass(0.0, true);
            body.recompute_mass_properties_from_colliders(&world.colliders);
        }
        state.solid_count = actor_solid_count(&state.runtime, state.actor);
        let result = {
            let Some(body) = world.bodies.get(body_handle) else {
                return empty_pixel_rigidbody_result(AlchemyRapierStatus::InvalidHandle);
            };
            pixel_result(
                AlchemyRapierStatus::Ok,
                collider_handle,
                state.solid_count,
                body,
            )
        };
        state.collider = collider_handle;
        state.local_origin = new_local_origin;
        world.pixel_rigidbodies.insert(body_handle, state);
        result
    })) {
        Ok(result) => result,
        Err(_) => empty_pixel_rigidbody_result(AlchemyRapierStatus::Panic),
    }
}

fn build_voxel_collider_shape_and_metadata(
    desc: AlchemyRapierVoxelColliderDesc,
) -> Result<(SharedShape, HashMap<u32, VoxelCellMetadata>), AlchemyRapierStatus> {
    if !desc.translation.x.is_finite()
        || !desc.translation.y.is_finite()
        || !desc.voxel_size.x.is_finite()
        || !desc.voxel_size.y.is_finite()
        || desc.voxel_size.x <= 0.0
        || desc.voxel_size.y <= 0.0
        || desc.cells.is_null()
        || desc.cell_count == 0
    {
        return Err(AlchemyRapierStatus::InvalidArgument);
    }

    let source = unsafe { slice::from_raw_parts(desc.cells, desc.cell_count) };
    let coords = source
        .iter()
        .map(|cell| IVector::new(cell.coord.x, cell.coord.y))
        .collect::<Vec<_>>();
    let voxels = Voxels::new(vector(desc.voxel_size), &coords);
    let mut metadata = HashMap::with_capacity(source.len());
    for cell in source {
        let coord = IVector::new(cell.coord.x, cell.coord.y);
        if let Some(index) = voxels.linear_index(coord) {
            metadata.insert(
                index.flat_id() as u32,
                VoxelCellMetadata {
                    material_id: cell.material_id,
                    source_cell_id: cell.source_cell_id,
                },
            );
        }
    }

    Ok((SharedShape::new(voxels), metadata))
}

fn create_voxel_collider_inner(
    world: &mut AlchemyRapierWorldInner,
    desc: AlchemyRapierVoxelColliderDesc,
) -> Result<ColliderHandle, AlchemyRapierStatus> {
    let (shape, metadata) = build_voxel_collider_shape_and_metadata(desc)?;
    let collider = ColliderBuilder::new(shape)
        .translation(vector(desc.translation))
        .active_hooks(ActiveHooks::MODIFY_SOLVER_CONTACTS)
        .build();
    let handle = world.colliders.insert(collider);
    world.voxel_colliders.insert(collider_key(handle), metadata);
    Ok(handle)
}

fn update_voxel_collider_inner(
    world: &mut AlchemyRapierWorldInner,
    packed_id: u64,
    desc: AlchemyRapierVoxelColliderDesc,
) -> Result<ColliderHandle, AlchemyRapierStatus> {
    let (shape, metadata) = build_voxel_collider_shape_and_metadata(desc)?;
    let Some(handle) = collider_handle_from_packed(packed_id) else {
        return Err(AlchemyRapierStatus::InvalidHandle);
    };
    let Some(collider) = world.colliders.get_mut(handle) else {
        return Err(AlchemyRapierStatus::InvalidHandle);
    };

    collider.set_shape(shape);
    collider.set_translation(vector(desc.translation));
    collider.set_active_hooks(ActiveHooks::MODIFY_SOLVER_CONTACTS);
    world.voxel_colliders.insert(collider_key(handle), metadata);
    Ok(handle)
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_create_voxel_collider(
    world: *mut AlchemyRapierWorld,
    desc: AlchemyRapierVoxelColliderDesc,
) -> AlchemyRapierCreateColliderResult {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return empty_create_collider_result(AlchemyRapierStatus::NullPointer);
        };
        match create_voxel_collider_inner(world, desc) {
            Ok(handle) => AlchemyRapierCreateColliderResult {
                status: AlchemyRapierStatus::Ok,
                handle: collider_handle_to_ffi(handle),
                packed_id: pack_collider_handle(handle),
            },
            Err(status) => empty_create_collider_result(status),
        }
    })) {
        Ok(result) => result,
        Err(_) => empty_create_collider_result(AlchemyRapierStatus::Panic),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_create_static_terrain_voxel_collider(
    world: *mut AlchemyRapierWorld,
    desc: AlchemyRapierStaticTerrainVoxelColliderDesc,
) -> AlchemyRapierCreateColliderResult {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return empty_create_collider_result(AlchemyRapierStatus::NullPointer);
        };
        let source = match voxel_terrain_source_metadata(desc) {
            Ok(source) => source,
            Err(status) => return empty_create_collider_result(status),
        };
        match create_voxel_collider_inner(world, desc.voxel) {
            Ok(handle) => {
                world.voxel_terrain_sources.insert(handle, source);
                AlchemyRapierCreateColliderResult {
                    status: AlchemyRapierStatus::Ok,
                    handle: collider_handle_to_ffi(handle),
                    packed_id: pack_collider_handle(handle),
                }
            }
            Err(status) => empty_create_collider_result(status),
        }
    })) {
        Ok(result) => result,
        Err(_) => empty_create_collider_result(AlchemyRapierStatus::Panic),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_update_voxel_collider_by_id(
    world: *mut AlchemyRapierWorld,
    packed_id: u64,
    desc: AlchemyRapierVoxelColliderDesc,
) -> AlchemyRapierStatus {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return AlchemyRapierStatus::NullPointer;
        };
        match update_voxel_collider_inner(world, packed_id, desc) {
            Ok(handle) => {
                world.voxel_terrain_sources.remove(&handle);
                AlchemyRapierStatus::Ok
            }
            Err(status) => status,
        }
    })) {
        Ok(status) => status,
        Err(_) => AlchemyRapierStatus::Panic,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_update_static_terrain_voxel_collider_by_id(
    world: *mut AlchemyRapierWorld,
    packed_id: u64,
    desc: AlchemyRapierStaticTerrainVoxelColliderDesc,
) -> AlchemyRapierStatus {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return AlchemyRapierStatus::NullPointer;
        };
        let source = match voxel_terrain_source_metadata(desc) {
            Ok(source) => source,
            Err(status) => return status,
        };
        match update_voxel_collider_inner(world, packed_id, desc.voxel) {
            Ok(handle) => {
                world.voxel_terrain_sources.insert(handle, source);
                AlchemyRapierStatus::Ok
            }
            Err(status) => status,
        }
    })) {
        Ok(status) => status,
        Err(_) => AlchemyRapierStatus::Panic,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_destroy_collider(
    world: *mut AlchemyRapierWorld,
    handle: AlchemyRapierColliderHandle,
) -> AlchemyRapierStatus {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return AlchemyRapierStatus::NullPointer;
        };
        let handle = collider_handle_from_ffi(handle);
        let stale_pixel_body = world
            .pixel_rigidbodies
            .iter()
            .find_map(|(body, state)| (state.collider == handle).then_some(*body));
        if let Some(body) = stale_pixel_body {
            world.pixel_rigidbodies.remove(&body);
        }
        remove_voxel_collider_metadata(world, handle);
        if world
            .colliders
            .remove(handle, &mut world.islands, &mut world.bodies, true)
            .is_some()
        {
            AlchemyRapierStatus::Ok
        } else {
            AlchemyRapierStatus::InvalidHandle
        }
    })) {
        Ok(status) => status,
        Err(_) => AlchemyRapierStatus::Panic,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_destroy_collider_by_id(
    world: *mut AlchemyRapierWorld,
    packed_id: u64,
) -> AlchemyRapierStatus {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return AlchemyRapierStatus::NullPointer;
        };
        let Some(handle) = collider_handle_from_packed(packed_id) else {
            return AlchemyRapierStatus::InvalidHandle;
        };
        remove_voxel_collider_metadata(world, handle);
        if world
            .colliders
            .remove(handle, &mut world.islands, &mut world.bodies, true)
            .is_some()
        {
            AlchemyRapierStatus::Ok
        } else {
            AlchemyRapierStatus::InvalidHandle
        }
    })) {
        Ok(status) => status,
        Err(_) => AlchemyRapierStatus::Panic,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_body_state(
    world: *mut AlchemyRapierWorld,
    handle: AlchemyRapierRigidBodyHandle,
) -> AlchemyRapierBodyStateResult {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return AlchemyRapierBodyStateResult {
                status: AlchemyRapierStatus::NullPointer,
                packed_id: 0,
                body_type: AlchemyRapierBodyType::Dynamic,
                position: AlchemyRapierVec2::default(),
                rotation: 0.0,
                linear_velocity: AlchemyRapierVec2::default(),
                angular_velocity: 0.0,
                linear_damping: 0.0,
                angular_damping: 0.0,
                can_sleep: 0,
                is_awake: 0,
            };
        };
        let handle = handle_from_ffi(handle);
        let Some(body) = world.bodies.get(handle) else {
            return AlchemyRapierBodyStateResult {
                status: AlchemyRapierStatus::InvalidHandle,
                packed_id: 0,
                body_type: AlchemyRapierBodyType::Dynamic,
                position: AlchemyRapierVec2::default(),
                rotation: 0.0,
                linear_velocity: AlchemyRapierVec2::default(),
                angular_velocity: 0.0,
                linear_damping: 0.0,
                angular_damping: 0.0,
                can_sleep: 0,
                is_awake: 0,
            };
        };
        make_body_state(handle, body)
    })) {
        Ok(result) => result,
        Err(_) => AlchemyRapierBodyStateResult {
            status: AlchemyRapierStatus::Panic,
            packed_id: 0,
            body_type: AlchemyRapierBodyType::Dynamic,
            position: AlchemyRapierVec2::default(),
            rotation: 0.0,
            linear_velocity: AlchemyRapierVec2::default(),
            angular_velocity: 0.0,
            linear_damping: 0.0,
            angular_damping: 0.0,
            can_sleep: 0,
            is_awake: 0,
        },
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_body_mass(
    world: *mut AlchemyRapierWorld,
    handle: AlchemyRapierRigidBodyHandle,
) -> AlchemyRapierMassResult {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return AlchemyRapierMassResult {
                status: AlchemyRapierStatus::NullPointer,
                local_center_of_mass: AlchemyRapierVec2::default(),
                mass: 0.0,
                inertia: 0.0,
            };
        };
        let handle = handle_from_ffi(handle);
        let Some(body) = world.bodies.get(handle) else {
            return AlchemyRapierMassResult {
                status: AlchemyRapierStatus::InvalidHandle,
                local_center_of_mass: AlchemyRapierVec2::default(),
                mass: 0.0,
                inertia: 0.0,
            };
        };
        AlchemyRapierMassResult {
            status: AlchemyRapierStatus::Ok,
            local_center_of_mass: ffi_vec(body.local_center_of_mass()),
            mass: body.mass(),
            inertia: body.mass_properties().local_mprops.principal_inertia(),
        }
    })) {
        Ok(result) => result,
        Err(_) => AlchemyRapierMassResult {
            status: AlchemyRapierStatus::Panic,
            local_center_of_mass: AlchemyRapierVec2::default(),
            mass: 0.0,
            inertia: 0.0,
        },
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_body_point_velocity(
    world: *mut AlchemyRapierWorld,
    handle: AlchemyRapierRigidBodyHandle,
    world_point: AlchemyRapierVec2,
) -> AlchemyRapierVec2Result {
    body_vec2_query(world, handle, |body| {
        body.velocity_at_point(vector(world_point))
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_body_local_point(
    world: *mut AlchemyRapierWorld,
    handle: AlchemyRapierRigidBodyHandle,
    world_point: AlchemyRapierVec2,
) -> AlchemyRapierVec2Result {
    body_vec2_query(world, handle, |body| {
        body.position().inverse_transform_point(vector(world_point))
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_body_world_point(
    world: *mut AlchemyRapierWorld,
    handle: AlchemyRapierRigidBodyHandle,
    local_point: AlchemyRapierVec2,
) -> AlchemyRapierVec2Result {
    body_vec2_query(world, handle, |body| {
        body.position().transform_point(vector(local_point))
    })
}

fn body_vec2_query<F>(
    world: *mut AlchemyRapierWorld,
    handle: AlchemyRapierRigidBodyHandle,
    query: F,
) -> AlchemyRapierVec2Result
where
    F: FnOnce(&RigidBody) -> Vector,
{
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return AlchemyRapierVec2Result {
                status: AlchemyRapierStatus::NullPointer,
                value: AlchemyRapierVec2::default(),
            };
        };
        let handle = handle_from_ffi(handle);
        let Some(body) = world.bodies.get(handle) else {
            return AlchemyRapierVec2Result {
                status: AlchemyRapierStatus::InvalidHandle,
                value: AlchemyRapierVec2::default(),
            };
        };
        AlchemyRapierVec2Result {
            status: AlchemyRapierStatus::Ok,
            value: ffi_vec(query(body)),
        }
    })) {
        Ok(result) => result,
        Err(_) => AlchemyRapierVec2Result {
            status: AlchemyRapierStatus::Panic,
            value: AlchemyRapierVec2::default(),
        },
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_read_split_event_rows(
    world: *mut AlchemyRapierWorld,
    rows: *mut AlchemyRapierSplitEventRow,
    row_capacity: usize,
) -> AlchemyRapierSplitEventReadResult {
    match catch_unwind(AssertUnwindSafe(|| {
        if rows.is_null() && row_capacity > 0 {
            return AlchemyRapierSplitEventReadResult {
                status: AlchemyRapierStatus::NullPointer,
                row_count: 0,
                written_count: 0,
            };
        }
        let Ok(world) = to_inner(world) else {
            return AlchemyRapierSplitEventReadResult {
                status: AlchemyRapierStatus::NullPointer,
                row_count: 0,
                written_count: 0,
            };
        };
        let row_count = world.pending_split_events.len();
        let written_count = row_count.min(row_capacity);
        if written_count > 0 {
            let out = unsafe { slice::from_raw_parts_mut(rows, written_count) };
            for (slot, event) in out.iter_mut().zip(world.pending_split_events.iter()) {
                *slot = event.row;
            }
        }
        AlchemyRapierSplitEventReadResult {
            status: AlchemyRapierStatus::Ok,
            row_count,
            written_count,
        }
    })) {
        Ok(result) => result,
        Err(_) => AlchemyRapierSplitEventReadResult {
            status: AlchemyRapierStatus::Panic,
            row_count: 0,
            written_count: 0,
        },
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_copy_split_event_cells(
    world: *mut AlchemyRapierWorld,
    row_index: usize,
    cells: *mut i32,
    cell_capacity: usize,
) -> usize {
    match catch_unwind(AssertUnwindSafe(|| {
        if cells.is_null() && cell_capacity > 0 {
            return 0;
        }
        let Ok(world) = to_inner(world) else {
            return 0;
        };
        let Some(event) = world.pending_split_events.get(row_index) else {
            return 0;
        };
        let written_count = event.source_cell_indices.len().min(cell_capacity);
        if written_count > 0 {
            let out = unsafe { slice::from_raw_parts_mut(cells, written_count) };
            out.copy_from_slice(&event.source_cell_indices[..written_count]);
        }
        written_count
    })) {
        Ok(result) => result,
        Err(_) => 0,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_copy_split_event_touched_cells(
    world: *mut AlchemyRapierWorld,
    row_index: usize,
    cells: *mut i32,
    cell_capacity: usize,
) -> usize {
    match catch_unwind(AssertUnwindSafe(|| {
        if cells.is_null() && cell_capacity > 0 {
            return 0;
        }
        let Ok(world) = to_inner(world) else {
            return 0;
        };
        let Some(event) = world.pending_split_events.get(row_index) else {
            return 0;
        };
        let written_count = event.touched_source_cell_indices.len().min(cell_capacity);
        if written_count > 0 {
            let out = unsafe { slice::from_raw_parts_mut(cells, written_count) };
            out.copy_from_slice(&event.touched_source_cell_indices[..written_count]);
        }
        written_count
    })) {
        Ok(result) => result,
        Err(_) => 0,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_copy_split_event_removed_cells(
    world: *mut AlchemyRapierWorld,
    row_index: usize,
    cells: *mut i32,
    cell_capacity: usize,
) -> usize {
    match catch_unwind(AssertUnwindSafe(|| {
        if cells.is_null() && cell_capacity > 0 {
            return 0;
        }
        let Ok(world) = to_inner(world) else {
            return 0;
        };
        let Some(event) = world.pending_split_events.get(row_index) else {
            return 0;
        };
        let written_count = event.removed_source_cell_indices.len().min(cell_capacity);
        if written_count > 0 {
            let out = unsafe { slice::from_raw_parts_mut(cells, written_count) };
            out.copy_from_slice(&event.removed_source_cell_indices[..written_count]);
        }
        written_count
    })) {
        Ok(result) => result,
        Err(_) => 0,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_copy_split_event_occupancy_words(
    world: *mut AlchemyRapierWorld,
    row_index: usize,
    words: *mut u64,
    word_capacity: usize,
) -> usize {
    match catch_unwind(AssertUnwindSafe(|| {
        if words.is_null() && word_capacity > 0 {
            return 0;
        }
        let Ok(world) = to_inner(world) else {
            return 0;
        };
        let Some(event) = world.pending_split_events.get(row_index) else {
            return 0;
        };
        let written_count = event.child_occupancy_words.len().min(word_capacity);
        if written_count > 0 {
            let out = unsafe { slice::from_raw_parts_mut(words, written_count) };
            out.copy_from_slice(&event.child_occupancy_words[..written_count]);
        }
        written_count
    })) {
        Ok(result) => result,
        Err(_) => 0,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_copy_split_event_material_ids(
    world: *mut AlchemyRapierWorld,
    row_index: usize,
    material_ids: *mut u16,
    material_id_capacity: usize,
) -> usize {
    match catch_unwind(AssertUnwindSafe(|| {
        if material_ids.is_null() && material_id_capacity > 0 {
            return 0;
        }
        let Ok(world) = to_inner(world) else {
            return 0;
        };
        let Some(event) = world.pending_split_events.get(row_index) else {
            return 0;
        };
        let written_count = event.child_material_ids.len().min(material_id_capacity);
        if written_count > 0 {
            let out = unsafe { slice::from_raw_parts_mut(material_ids, written_count) };
            out.copy_from_slice(&event.child_material_ids[..written_count]);
        }
        written_count
    })) {
        Ok(result) => result,
        Err(_) => 0,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_acknowledge_split_event(
    world: *mut AlchemyRapierWorld,
    transition_id: u32,
    source_kind: AlchemyRapierQuerySourceKind,
    source_body_packed_id: u64,
    source_terrain_actor_key: i64,
) -> AlchemyRapierStatus {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return AlchemyRapierStatus::NullPointer;
        };
        let Some(index) = world.pending_split_events.iter().position(|event| {
            let row = event.row;
            row.transition_id == transition_id
                && row.source_kind == source_kind
                && row.source_body_packed_id == source_body_packed_id
                && row.source_terrain_actor_key == source_terrain_actor_key
        }) else {
            return AlchemyRapierStatus::InvalidHandle;
        };
        world.pending_split_events.remove(index);
        AlchemyRapierStatus::Ok
    })) {
        Ok(status) => status,
        Err(_) => AlchemyRapierStatus::Panic,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_apply_body_force_at_point(
    world: *mut AlchemyRapierWorld,
    handle: AlchemyRapierRigidBodyHandle,
    force: AlchemyRapierVec2,
    world_point: AlchemyRapierVec2,
) -> AlchemyRapierStatus {
    body_mutation(world, handle, |body| {
        body.add_force_at_point(vector(force), vector(world_point), true);
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_apply_body_impulse_at_point(
    world: *mut AlchemyRapierWorld,
    handle: AlchemyRapierRigidBodyHandle,
    impulse: AlchemyRapierVec2,
    world_point: AlchemyRapierVec2,
) -> AlchemyRapierStatus {
    body_mutation(world, handle, |body| {
        body.apply_impulse_at_point(vector(impulse), vector(world_point), true);
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_create_generic_joint(
    world: *mut AlchemyRapierWorld,
    desc: AlchemyRapierGenericJointDesc,
) -> AlchemyRapierCreateJointResult {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return empty_create_joint_result(AlchemyRapierStatus::NullPointer);
        };
        if !desc.local_anchor1.x.is_finite()
            || !desc.local_anchor1.y.is_finite()
            || !desc.local_anchor2.x.is_finite()
            || !desc.local_anchor2.y.is_finite()
            || !desc.local_rotation1.is_finite()
            || !desc.local_rotation2.is_finite()
            || !desc.natural_frequency.is_finite()
            || !desc.damping_ratio.is_finite()
            || !desc.limit_min.is_finite()
            || !desc.limit_max.is_finite()
            || desc.natural_frequency < 0.0
            || desc.damping_ratio < 0.0
            || desc.limit_enabled > 1
            || (desc.limit_enabled != 0 && desc.limit_min > desc.limit_max)
        {
            return empty_create_joint_result(AlchemyRapierStatus::InvalidArgument);
        }

        let Some(locked_axes) = JointAxesMask::from_bits(desc.locked_axes) else {
            return empty_create_joint_result(AlchemyRapierStatus::InvalidArgument);
        };

        let body1 = handle_from_ffi(desc.body1);
        let body2 = handle_from_ffi(desc.body2);
        if world.bodies.get(body1).is_none() || world.bodies.get(body2).is_none() {
            return empty_create_joint_result(AlchemyRapierStatus::InvalidHandle);
        }

        let mut joint = GenericJointBuilder::new(locked_axes)
            .local_frame1(Pose::from_parts(
                vector(desc.local_anchor1),
                Rotation::new(desc.local_rotation1),
            ))
            .local_frame2(Pose::from_parts(
                vector(desc.local_anchor2),
                Rotation::new(desc.local_rotation2),
            ))
            .contacts_enabled(desc.contacts_enabled != 0)
            .softness(joint_softness(desc.natural_frequency, desc.damping_ratio));
        if desc.limit_enabled != 0 {
            joint = joint.limits(JointAxis::AngX, [desc.limit_min, desc.limit_max]);
        }
        let handle = world
            .impulse_joints
            .insert(body1, body2, joint, desc.wake_up != 0);
        AlchemyRapierCreateJointResult {
            status: AlchemyRapierStatus::Ok,
            handle: joint_handle_to_ffi(handle),
            packed_id: pack_joint_handle(handle),
        }
    })) {
        Ok(result) => result,
        Err(_) => empty_create_joint_result(AlchemyRapierStatus::Panic),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_set_generic_joint_softness(
    world: *mut AlchemyRapierWorld,
    handle: AlchemyRapierJointHandle,
    natural_frequency: f32,
    damping_ratio: f32,
    wake_up: u8,
) -> AlchemyRapierStatus {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return AlchemyRapierStatus::NullPointer;
        };
        if !natural_frequency.is_finite()
            || !damping_ratio.is_finite()
            || natural_frequency < 0.0
            || damping_ratio < 0.0
        {
            return AlchemyRapierStatus::InvalidArgument;
        }
        let handle = joint_handle_from_ffi(handle);
        let Some(joint) = world.impulse_joints.get_mut(handle, wake_up != 0) else {
            return AlchemyRapierStatus::InvalidHandle;
        };
        let _ = joint
            .data
            .set_softness(joint_softness(natural_frequency, damping_ratio));
        AlchemyRapierStatus::Ok
    })) {
        Ok(status) => status,
        Err(_) => AlchemyRapierStatus::Panic,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_set_generic_joint_anchors(
    world: *mut AlchemyRapierWorld,
    handle: AlchemyRapierJointHandle,
    local_anchor1: AlchemyRapierVec2,
    local_anchor2: AlchemyRapierVec2,
    wake_up: u8,
) -> AlchemyRapierStatus {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return AlchemyRapierStatus::NullPointer;
        };
        if !local_anchor1.x.is_finite()
            || !local_anchor1.y.is_finite()
            || !local_anchor2.x.is_finite()
            || !local_anchor2.y.is_finite()
        {
            return AlchemyRapierStatus::InvalidArgument;
        }

        let handle = joint_handle_from_ffi(handle);
        let Some(joint) = world.impulse_joints.get_mut(handle, wake_up != 0) else {
            return AlchemyRapierStatus::InvalidHandle;
        };
        joint.data.set_local_anchor1(vector(local_anchor1));
        joint.data.set_local_anchor2(vector(local_anchor2));
        AlchemyRapierStatus::Ok
    })) {
        Ok(status) => status,
        Err(_) => AlchemyRapierStatus::Panic,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_create_revolute_joint(
    world: *mut AlchemyRapierWorld,
    desc: AlchemyRapierRevoluteJointDesc,
) -> AlchemyRapierCreateJointResult {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return empty_create_joint_result(AlchemyRapierStatus::NullPointer);
        };
        if !desc.local_anchor1.x.is_finite()
            || !desc.local_anchor1.y.is_finite()
            || !desc.local_anchor2.x.is_finite()
            || !desc.local_anchor2.y.is_finite()
            || !desc.natural_frequency.is_finite()
            || !desc.damping_ratio.is_finite()
            || desc.natural_frequency < 0.0
            || desc.damping_ratio < 0.0
        {
            return empty_create_joint_result(AlchemyRapierStatus::InvalidArgument);
        }

        let body1 = handle_from_ffi(desc.body1);
        let body2 = handle_from_ffi(desc.body2);
        if world.bodies.get(body1).is_none() || world.bodies.get(body2).is_none() {
            return empty_create_joint_result(AlchemyRapierStatus::InvalidHandle);
        }

        let joint = RevoluteJointBuilder::new()
            .local_anchor1(vector(desc.local_anchor1))
            .local_anchor2(vector(desc.local_anchor2))
            .contacts_enabled(desc.contacts_enabled != 0)
            .softness(joint_softness(desc.natural_frequency, desc.damping_ratio));
        let handle = world
            .impulse_joints
            .insert(body1, body2, joint, desc.wake_up != 0);
        AlchemyRapierCreateJointResult {
            status: AlchemyRapierStatus::Ok,
            handle: joint_handle_to_ffi(handle),
            packed_id: pack_joint_handle(handle),
        }
    })) {
        Ok(result) => result,
        Err(_) => empty_create_joint_result(AlchemyRapierStatus::Panic),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_set_revolute_joint_softness(
    world: *mut AlchemyRapierWorld,
    handle: AlchemyRapierJointHandle,
    natural_frequency: f32,
    damping_ratio: f32,
    wake_up: u8,
) -> AlchemyRapierStatus {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return AlchemyRapierStatus::NullPointer;
        };
        if !natural_frequency.is_finite()
            || !damping_ratio.is_finite()
            || natural_frequency < 0.0
            || damping_ratio < 0.0
        {
            return AlchemyRapierStatus::InvalidArgument;
        }
        let handle = joint_handle_from_ffi(handle);
        let Some(joint) = world.impulse_joints.get_mut(handle, wake_up != 0) else {
            return AlchemyRapierStatus::InvalidHandle;
        };
        let _ = joint
            .data
            .set_softness(joint_softness(natural_frequency, damping_ratio));
        AlchemyRapierStatus::Ok
    })) {
        Ok(status) => status,
        Err(_) => AlchemyRapierStatus::Panic,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_set_revolute_joint_anchors(
    world: *mut AlchemyRapierWorld,
    handle: AlchemyRapierJointHandle,
    local_anchor1: AlchemyRapierVec2,
    local_anchor2: AlchemyRapierVec2,
    wake_up: u8,
) -> AlchemyRapierStatus {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return AlchemyRapierStatus::NullPointer;
        };
        if !local_anchor1.x.is_finite()
            || !local_anchor1.y.is_finite()
            || !local_anchor2.x.is_finite()
            || !local_anchor2.y.is_finite()
        {
            return AlchemyRapierStatus::InvalidArgument;
        }

        let handle = joint_handle_from_ffi(handle);
        let Some(joint) = world.impulse_joints.get_mut(handle, wake_up != 0) else {
            return AlchemyRapierStatus::InvalidHandle;
        };
        joint.data.set_local_anchor1(vector(local_anchor1));
        joint.data.set_local_anchor2(vector(local_anchor2));
        AlchemyRapierStatus::Ok
    })) {
        Ok(status) => status,
        Err(_) => AlchemyRapierStatus::Panic,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_create_rope_joint(
    world: *mut AlchemyRapierWorld,
    desc: AlchemyRapierRopeJointDesc,
) -> AlchemyRapierCreateJointResult {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return empty_create_joint_result(AlchemyRapierStatus::NullPointer);
        };
        if !desc.local_anchor1.x.is_finite()
            || !desc.local_anchor1.y.is_finite()
            || !desc.local_anchor2.x.is_finite()
            || !desc.local_anchor2.y.is_finite()
            || !desc.max_distance.is_finite()
            || desc.max_distance <= 0.0
        {
            return empty_create_joint_result(AlchemyRapierStatus::InvalidArgument);
        }

        let body1 = handle_from_ffi(desc.body1);
        let body2 = handle_from_ffi(desc.body2);
        if world.bodies.get(body1).is_none() || world.bodies.get(body2).is_none() {
            return empty_create_joint_result(AlchemyRapierStatus::InvalidHandle);
        }

        let joint = RopeJointBuilder::new(desc.max_distance)
            .local_anchor1(vector(desc.local_anchor1))
            .local_anchor2(vector(desc.local_anchor2))
            .contacts_enabled(desc.contacts_enabled != 0);
        let handle = world
            .impulse_joints
            .insert(body1, body2, joint, desc.wake_up != 0);
        AlchemyRapierCreateJointResult {
            status: AlchemyRapierStatus::Ok,
            handle: joint_handle_to_ffi(handle),
            packed_id: pack_joint_handle(handle),
        }
    })) {
        Ok(result) => result,
        Err(_) => empty_create_joint_result(AlchemyRapierStatus::Panic),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_set_rope_joint_anchors(
    world: *mut AlchemyRapierWorld,
    handle: AlchemyRapierJointHandle,
    local_anchor1: AlchemyRapierVec2,
    local_anchor2: AlchemyRapierVec2,
    wake_up: u8,
) -> AlchemyRapierStatus {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return AlchemyRapierStatus::NullPointer;
        };
        if !local_anchor1.x.is_finite()
            || !local_anchor1.y.is_finite()
            || !local_anchor2.x.is_finite()
            || !local_anchor2.y.is_finite()
        {
            return AlchemyRapierStatus::InvalidArgument;
        }

        let handle = joint_handle_from_ffi(handle);
        let Some(joint) = world.impulse_joints.get_mut(handle, wake_up != 0) else {
            return AlchemyRapierStatus::InvalidHandle;
        };
        joint.data.set_local_anchor1(vector(local_anchor1));
        joint.data.set_local_anchor2(vector(local_anchor2));
        AlchemyRapierStatus::Ok
    })) {
        Ok(status) => status,
        Err(_) => AlchemyRapierStatus::Panic,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_set_rope_joint_max_distance(
    world: *mut AlchemyRapierWorld,
    handle: AlchemyRapierJointHandle,
    max_distance: f32,
    wake_up: u8,
) -> AlchemyRapierStatus {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return AlchemyRapierStatus::NullPointer;
        };
        if !max_distance.is_finite() || max_distance <= 0.0 {
            return AlchemyRapierStatus::InvalidArgument;
        }

        let handle = joint_handle_from_ffi(handle);
        let Some(joint) = world.impulse_joints.get_mut(handle, wake_up != 0) else {
            return AlchemyRapierStatus::InvalidHandle;
        };
        joint.data.set_limits(JointAxis::LinX, [0.0, max_distance]);
        AlchemyRapierStatus::Ok
    })) {
        Ok(status) => status,
        Err(_) => AlchemyRapierStatus::Panic,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_joint_impulse(
    world: *mut AlchemyRapierWorld,
    handle: AlchemyRapierJointHandle,
) -> AlchemyRapierJointImpulseResult {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return empty_joint_impulse_result(AlchemyRapierStatus::NullPointer);
        };

        let handle = joint_handle_from_ffi(handle);
        let Some(joint) = world.impulse_joints.get(handle) else {
            return empty_joint_impulse_result(AlchemyRapierStatus::InvalidHandle);
        };

        AlchemyRapierJointImpulseResult {
            status: AlchemyRapierStatus::Ok,
            linear_impulse: AlchemyRapierVec2 {
                x: joint.impulses[JointAxis::LinX as usize]
                    + joint.data.limits[JointAxis::LinX as usize].impulse,
                y: joint.impulses[JointAxis::LinY as usize]
                    + joint.data.limits[JointAxis::LinY as usize].impulse,
            },
            angular_impulse: joint.impulses[2],
        }
    })) {
        Ok(result) => result,
        Err(_) => empty_joint_impulse_result(AlchemyRapierStatus::Panic),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_destroy_joint(
    world: *mut AlchemyRapierWorld,
    handle: AlchemyRapierJointHandle,
    wake_up: u8,
) -> AlchemyRapierStatus {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return AlchemyRapierStatus::NullPointer;
        };
        let handle = joint_handle_from_ffi(handle);
        if world.impulse_joints.remove(handle, wake_up != 0).is_some() {
            AlchemyRapierStatus::Ok
        } else {
            AlchemyRapierStatus::InvalidHandle
        }
    })) {
        Ok(status) => status,
        Err(_) => AlchemyRapierStatus::Panic,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_apply_body_linear_impulse(
    world: *mut AlchemyRapierWorld,
    handle: AlchemyRapierRigidBodyHandle,
    impulse: AlchemyRapierVec2,
    wake_up: u8,
) -> AlchemyRapierStatus {
    body_mutation(world, handle, |body| {
        body.apply_impulse(vector(impulse), wake_up != 0);
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_apply_body_torque_impulse(
    world: *mut AlchemyRapierWorld,
    handle: AlchemyRapierRigidBodyHandle,
    impulse: f32,
    wake_up: u8,
) -> AlchemyRapierStatus {
    body_mutation(world, handle, |body| {
        body.apply_torque_impulse(impulse, wake_up != 0);
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_apply_ragdoll_motor_batch(
    world: *mut AlchemyRapierWorld,
    motors: *const AlchemyRapierRagdollMotorDesc,
    motor_count: usize,
    results: *mut AlchemyRapierRagdollMotorResult,
    result_capacity: usize,
) -> AlchemyRapierStatus {
    match catch_unwind(AssertUnwindSafe(|| {
        if motor_count > MAX_RAGDOLL_MOTOR_BATCH_COUNT
            || result_capacity < motor_count
            || (motor_count > 0 && (motors.is_null() || results.is_null()))
        {
            return AlchemyRapierStatus::InvalidArgument;
        }
        let Ok(world) = to_inner(world) else {
            return AlchemyRapierStatus::NullPointer;
        };
        if motor_count == 0 {
            return AlchemyRapierStatus::Ok;
        }
        let motors = unsafe { slice::from_raw_parts(motors, motor_count) };

        // Validate all descriptors and body handles before applying any impulse.
        for motor in motors {
            if !ragdoll_motor_desc_is_valid(*motor) {
                return AlchemyRapierStatus::InvalidArgument;
            }
            let body_a = handle_from_ffi(motor.body_a);
            let body_b = handle_from_ffi(motor.body_b);
            if body_a == body_b {
                return AlchemyRapierStatus::InvalidArgument;
            }
            if !world.bodies.contains(body_a) || !world.bodies.contains(body_b) {
                return AlchemyRapierStatus::InvalidHandle;
            }
        }

        let mut solved_results =
            [AlchemyRapierRagdollMotorResult::default(); MAX_RAGDOLL_MOTOR_BATCH_COUNT];
        let mut torque_impulses = [0.0; MAX_RAGDOLL_MOTOR_BATCH_COUNT];

        // Sample the full batch before changing any body state. Shared bodies
        // therefore see the same velocity snapshot regardless of joint order.
        for (index, motor) in motors.iter().enumerate() {
            let body_a_handle = handle_from_ffi(motor.body_a);
            let body_b_handle = handle_from_ffi(motor.body_b);
            let (body_a_rotation, body_a_velocity) = {
                let body = world
                    .bodies
                    .get(body_a_handle)
                    .expect("validated body handle");
                (body.rotation().angle(), body.angvel())
            };
            let (body_b_rotation, body_b_velocity) = {
                let body = world
                    .bodies
                    .get(body_b_handle)
                    .expect("validated body handle");
                (body.rotation().angle(), body.angvel())
            };
            let current_angle = normalize_ragdoll_angle(
                body_a_rotation - body_b_rotation - motor.reference_relative_angle,
            );
            let angle_error = normalize_ragdoll_angle(motor.target_relative_angle - current_angle);
            let requested_torque = motor.stiffness * angle_error
                + motor.damping
                    * (motor.target_relative_angular_velocity
                        - (body_a_velocity - body_b_velocity));
            let applied_torque = requested_torque.clamp(-motor.max_torque, motor.max_torque);
            solved_results[index] = AlchemyRapierRagdollMotorResult {
                angle_error,
                applied_torque,
            };
            torque_impulses[index] = applied_torque * motor.delta_seconds;
        }

        // Apply the precomputed opposing impulses, then publish diagnostics.
        for (index, motor) in motors.iter().enumerate() {
            let body_a_handle = handle_from_ffi(motor.body_a);
            let body_b_handle = handle_from_ffi(motor.body_b);
            let torque_impulse = torque_impulses[index];
            world
                .bodies
                .get_mut(body_a_handle)
                .expect("validated body handle")
                .apply_torque_impulse(torque_impulse, true);
            world
                .bodies
                .get_mut(body_b_handle)
                .expect("validated body handle")
                .apply_torque_impulse(-torque_impulse, true);
            unsafe {
                *results.add(index) = solved_results[index];
            }
        }
        AlchemyRapierStatus::Ok
    })) {
        Ok(status) => status,
        Err(_) => AlchemyRapierStatus::Panic,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_query_cast_segment(
    world: *mut AlchemyRapierWorld,
    from: AlchemyRapierVec2,
    to: AlchemyRapierVec2,
    radius: f32,
    ignored_body: AlchemyRapierRigidBodyHandle,
    has_ignored_body: u8,
    source_mask: u32,
    hits: *mut AlchemyRapierQueryHit,
    hit_capacity: usize,
) -> AlchemyRapierQueryResult {
    match catch_unwind(AssertUnwindSafe(|| {
        if !query_output_valid(hits, hit_capacity) {
            return empty_query_result(AlchemyRapierStatus::NullPointer);
        }
        if !radius.is_finite() || radius < 0.0 {
            return empty_query_result(AlchemyRapierStatus::InvalidArgument);
        }
        if source_mask == 0 {
            return empty_query_result(AlchemyRapierStatus::Ok);
        }
        let Ok(world) = to_inner(world) else {
            return empty_query_result(AlchemyRapierStatus::NullPointer);
        };
        world
            .bodies
            .propagate_modified_body_positions_to_colliders(&mut world.colliders);

        let from = vector(from);
        let to = vector(to);
        let delta = to - from;
        let distance = delta.length();
        if !distance.is_finite() || distance <= 0.000001 {
            return AlchemyRapierQueryResult {
                status: AlchemyRapierStatus::Ok,
                hit_count: 0,
                written_count: 0,
                candidate_count: 0,
            };
        }

        let ignored_body = if has_ignored_body != 0 {
            Some(handle_from_ffi(ignored_body))
        } else {
            None
        };
        let mut hit_count = 0;
        let mut written_count = 0;
        let mut candidate_count = 0;
        if radius <= 0.000001 {
            let ray = Ray::new(from, delta / distance);
            for (collider_handle, collider) in world.colliders.iter_enabled() {
                let Some(target) =
                    query_target(world, collider_handle, collider, ignored_body, source_mask)
                else {
                    continue;
                };
                candidate_count += 1;
                if let Some(intersection) = collider.shape().cast_ray_and_get_normal(
                    collider.position(),
                    &ray,
                    distance,
                    true,
                ) {
                    let fraction = (intersection.time_of_impact / distance).clamp(0.0, 1.0);
                    let point = ray.point_at(intersection.time_of_impact);
                    if let Some(hit) = make_query_hit(
                        world,
                        target,
                        collider_handle,
                        point,
                        intersection.normal,
                        intersection.time_of_impact,
                        fraction,
                    ) {
                        write_query_hit(
                            hit,
                            hits,
                            hit_capacity,
                            &mut hit_count,
                            &mut written_count,
                        );
                    }
                }
            }
        } else {
            let ball = Ball::new(radius);
            let ball_pose = pose_translation(from);
            let options = ShapeCastOptions {
                max_time_of_impact: 1.0,
                target_distance: 0.0,
                stop_at_penetration: true,
                compute_impact_geometry_on_penetration: true,
            };
            let dispatcher = world.narrow_phase.query_dispatcher();
            for (collider_handle, collider) in world.colliders.iter_enabled() {
                let Some(target) =
                    query_target(world, collider_handle, collider, ignored_body, source_mask)
                else {
                    continue;
                };
                candidate_count += 1;
                let pos12 = collider.position().inv_mul(&ball_pose);
                let local_vel12 = collider.position().inverse_transform_vector(delta);
                let Ok(Some(shape_hit)) =
                    dispatcher.cast_shapes(&pos12, local_vel12, collider.shape(), &ball, options)
                else {
                    continue;
                };
                let fraction = shape_hit.time_of_impact.clamp(0.0, 1.0);
                let point = collider.position().transform_point(shape_hit.witness1);
                let normal = collider.position().rotation * shape_hit.normal1;
                if let Some(hit) = make_query_hit(
                    world,
                    target,
                    collider_handle,
                    point,
                    normal,
                    fraction * distance,
                    fraction,
                ) {
                    write_query_hit(hit, hits, hit_capacity, &mut hit_count, &mut written_count);
                }
            }
        }

        AlchemyRapierQueryResult {
            status: AlchemyRapierStatus::Ok,
            hit_count,
            written_count,
            candidate_count,
        }
    })) {
        Ok(result) => result,
        Err(_) => empty_query_result(AlchemyRapierStatus::Panic),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_query_cast_capsule(
    world: *mut AlchemyRapierWorld,
    from_origin: AlchemyRapierVec2,
    to_origin: AlchemyRapierVec2,
    radius: f32,
    half_height: f32,
    ignored_body: AlchemyRapierRigidBodyHandle,
    has_ignored_body: u8,
    source_mask: u32,
    hits: *mut AlchemyRapierQueryHit,
    hit_capacity: usize,
) -> AlchemyRapierQueryResult {
    match catch_unwind(AssertUnwindSafe(|| {
        if !query_output_valid(hits, hit_capacity) {
            return empty_query_result(AlchemyRapierStatus::NullPointer);
        }
        if !radius.is_finite() || radius < 0.0 || !half_height.is_finite() || half_height < 0.0 {
            return empty_query_result(AlchemyRapierStatus::InvalidArgument);
        }
        if source_mask == 0 {
            return empty_query_result(AlchemyRapierStatus::Ok);
        }
        let Ok(world) = to_inner(world) else {
            return empty_query_result(AlchemyRapierStatus::NullPointer);
        };
        world
            .bodies
            .propagate_modified_body_positions_to_colliders(&mut world.colliders);

        let from_origin = vector(from_origin);
        let to_origin = vector(to_origin);
        let delta = to_origin - from_origin;
        let distance = delta.length();
        if !distance.is_finite() || distance <= 0.000001 {
            return AlchemyRapierQueryResult {
                status: AlchemyRapierStatus::Ok,
                hit_count: 0,
                written_count: 0,
                candidate_count: 0,
            };
        }

        let ignored_body = if has_ignored_body != 0 {
            Some(handle_from_ffi(ignored_body))
        } else {
            None
        };
        let capsule = Capsule::new_y(half_height, radius.max(0.000001));
        let capsule_pose = pose_translation(from_origin);
        let options = ShapeCastOptions {
            max_time_of_impact: 1.0,
            target_distance: 0.0,
            stop_at_penetration: true,
            compute_impact_geometry_on_penetration: true,
        };
        let dispatcher = world.narrow_phase.query_dispatcher();
        let mut hit_count = 0;
        let mut written_count = 0;
        let mut candidate_count = 0;
        for (collider_handle, collider) in world.colliders.iter_enabled() {
            let Some(target) =
                query_target(world, collider_handle, collider, ignored_body, source_mask)
            else {
                continue;
            };
            candidate_count += 1;
            let pos12 = collider.position().inv_mul(&capsule_pose);
            let local_vel12 = collider.position().inverse_transform_vector(delta);
            let Ok(Some(shape_hit)) =
                dispatcher.cast_shapes(&pos12, local_vel12, collider.shape(), &capsule, options)
            else {
                continue;
            };
            let fraction = shape_hit.time_of_impact.clamp(0.0, 1.0);
            let impact_pose = pose_translation(from_origin + delta * fraction);
            let point = impact_pose.transform_point(shape_hit.witness2);
            let normal = collider.position().rotation * shape_hit.normal1;
            if let Some(hit) = make_query_hit(
                world,
                target,
                collider_handle,
                point,
                normal,
                fraction * distance,
                fraction,
            ) {
                write_query_hit(hit, hits, hit_capacity, &mut hit_count, &mut written_count);
            }
        }

        AlchemyRapierQueryResult {
            status: AlchemyRapierStatus::Ok,
            hit_count,
            written_count,
            candidate_count,
        }
    })) {
        Ok(result) => result,
        Err(_) => empty_query_result(AlchemyRapierStatus::Panic),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_query_overlap_capsule(
    world: *mut AlchemyRapierWorld,
    origin: AlchemyRapierVec2,
    radius: f32,
    half_height: f32,
    ignored_body: AlchemyRapierRigidBodyHandle,
    has_ignored_body: u8,
    source_mask: u32,
    hits: *mut AlchemyRapierQueryHit,
    hit_capacity: usize,
) -> AlchemyRapierQueryResult {
    match catch_unwind(AssertUnwindSafe(|| {
        if !query_output_valid(hits, hit_capacity) {
            return empty_query_result(AlchemyRapierStatus::NullPointer);
        }
        if !radius.is_finite() || radius < 0.0 || !half_height.is_finite() || half_height < 0.0 {
            return empty_query_result(AlchemyRapierStatus::InvalidArgument);
        }
        if source_mask == 0 {
            return empty_query_result(AlchemyRapierStatus::Ok);
        }
        let Ok(world) = to_inner(world) else {
            return empty_query_result(AlchemyRapierStatus::NullPointer);
        };
        world
            .bodies
            .propagate_modified_body_positions_to_colliders(&mut world.colliders);

        let origin = vector(origin);
        let capsule = Capsule::new_y(half_height, radius.max(0.000001));
        let capsule_pose = pose_translation(origin);
        let ignored_body = if has_ignored_body != 0 {
            Some(handle_from_ffi(ignored_body))
        } else {
            None
        };
        let dispatcher = world.narrow_phase.query_dispatcher();
        let mut hit_count = 0;
        let mut written_count = 0;
        let mut candidate_count = 0;
        for (collider_handle, collider) in world.colliders.iter_enabled() {
            let Some(target) =
                query_target(world, collider_handle, collider, ignored_body, source_mask)
            else {
                continue;
            };
            candidate_count += 1;
            let pos12 = capsule_pose.inv_mul(collider.position());
            let Ok(intersects) = dispatcher.intersection_test(&pos12, &capsule, collider.shape())
            else {
                continue;
            };
            if !intersects {
                continue;
            }

            let (point, normal, distance) =
                capsule_overlap_hit_point(collider, origin, half_height);
            if let Some(hit) =
                make_query_hit(world, target, collider_handle, point, normal, distance, 0.0)
            {
                write_query_hit(hit, hits, hit_capacity, &mut hit_count, &mut written_count);
            }
        }

        AlchemyRapierQueryResult {
            status: AlchemyRapierStatus::Ok,
            hit_count,
            written_count,
            candidate_count,
        }
    })) {
        Ok(result) => result,
        Err(_) => empty_query_result(AlchemyRapierStatus::Panic),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_query_surface_anchor(
    world: *mut AlchemyRapierWorld,
    source_kind: AlchemyRapierQuerySourceKind,
    target_body: AlchemyRapierRigidBodyHandle,
    has_target_body: u8,
    terrain_cell_x: i32,
    terrain_cell_y: i32,
    has_terrain_cell: u8,
    anchor_world: AlchemyRapierVec2,
    max_distance: f32,
    hits: *mut AlchemyRapierQueryHit,
    hit_capacity: usize,
) -> AlchemyRapierQueryResult {
    match catch_unwind(AssertUnwindSafe(|| {
        if !query_output_valid(hits, hit_capacity) {
            return empty_query_result(AlchemyRapierStatus::NullPointer);
        }
        if !max_distance.is_finite() || max_distance < 0.0 {
            return empty_query_result(AlchemyRapierStatus::InvalidArgument);
        }
        let Ok(world) = to_inner(world) else {
            return empty_query_result(AlchemyRapierStatus::NullPointer);
        };
        world
            .bodies
            .propagate_modified_body_positions_to_colliders(&mut world.colliders);

        let anchor = vector(anchor_world);
        let mut hit_count = 0;
        let mut written_count = 0;
        let mut candidate_count = 0;

        if source_kind == AlchemyRapierQuerySourceKind::DynamicPixelRigidbody {
            if has_target_body == 0 {
                return empty_query_result(AlchemyRapierStatus::InvalidArgument);
            }
            let target_body = handle_from_ffi(target_body);
            let Some(body) = world.bodies.get(target_body) else {
                return empty_query_result(AlchemyRapierStatus::InvalidHandle);
            };
            if !is_alchemy_rigidbody_query_body_type(body.body_type()) {
                return empty_query_result(AlchemyRapierStatus::Ok);
            }
            let body_type = body.body_type();

            for collider_handle in body.colliders() {
                let Some(collider) = world.colliders.get(*collider_handle) else {
                    continue;
                };
                if body_type != RigidBodyType::Dynamic
                    && !collider_supports_kinematic_alchemy_query(collider)
                {
                    continue;
                }
                candidate_count += 1;
                let projection = collider
                    .shape()
                    .project_point(collider.position(), anchor, true);
                let point = projection.point;
                let distance = if projection.is_inside {
                    0.0
                } else {
                    (anchor - point).length()
                };
                if distance > max_distance {
                    continue;
                }
                let normal = if projection.is_inside {
                    Vector::ZERO
                } else {
                    normalized_or_zero(anchor - point)
                };
                if let Some(hit) = make_query_hit(
                    world,
                    QueryTarget::Dynamic(target_body),
                    *collider_handle,
                    point,
                    normal,
                    distance,
                    0.0,
                ) {
                    write_query_hit(hit, hits, hit_capacity, &mut hit_count, &mut written_count);
                }
            }
        } else if source_kind == AlchemyRapierQuerySourceKind::StaticTerrain {
            for (collider_handle, collider) in world.colliders.iter_enabled() {
                let target =
                    if let Some(key) = world.terrain_by_collider.get(&collider_handle).copied() {
                        QueryTarget::TerrainChunk(key)
                    } else if let Some(actor_key) = world
                        .terrain_fracture_actor_by_collider
                        .get(&collider_handle)
                        .copied()
                    {
                        QueryTarget::TerrainFractureActor(actor_key)
                    } else if let Some(source) =
                        world.voxel_terrain_sources.get(&collider_handle).copied()
                    {
                        QueryTarget::VoxelStaticTerrain(source)
                    } else {
                        continue;
                    };
                candidate_count += 1;
                let projection = collider
                    .shape()
                    .project_point(collider.position(), anchor, true);
                let point = projection.point;
                let distance = if projection.is_inside {
                    0.0
                } else {
                    (anchor - point).length()
                };
                if distance > max_distance {
                    continue;
                }
                let normal = if projection.is_inside {
                    Vector::ZERO
                } else {
                    normalized_or_zero(anchor - point)
                };
                let Some(hit) =
                    make_query_hit(world, target, collider_handle, point, normal, distance, 0.0)
                else {
                    continue;
                };
                if has_terrain_cell != 0
                    && (hit.world_cell_x != terrain_cell_x || hit.world_cell_y != terrain_cell_y)
                {
                    continue;
                }
                write_query_hit(hit, hits, hit_capacity, &mut hit_count, &mut written_count);
            }
        } else {
            return empty_query_result(AlchemyRapierStatus::InvalidArgument);
        }

        AlchemyRapierQueryResult {
            status: AlchemyRapierStatus::Ok,
            hit_count,
            written_count,
            candidate_count,
        }
    })) {
        Ok(result) => result,
        Err(_) => empty_query_result(AlchemyRapierStatus::Panic),
    }
}

fn body_mutation<F>(
    world: *mut AlchemyRapierWorld,
    handle: AlchemyRapierRigidBodyHandle,
    mutation: F,
) -> AlchemyRapierStatus
where
    F: FnOnce(&mut RigidBody),
{
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return AlchemyRapierStatus::NullPointer;
        };
        let handle = handle_from_ffi(handle);
        let Some(body) = world.bodies.get_mut(handle) else {
            return AlchemyRapierStatus::InvalidHandle;
        };
        mutation(body);
        status_result(AlchemyRapierStatus::Ok)
    })) {
        Ok(status) => status,
        Err(_) => AlchemyRapierStatus::Panic,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_fx_world_create() -> *mut AlchemyFxRapierWorld {
    match catch_unwind(AssertUnwindSafe(|| {
        let world = AlchemyFxRapierWorldInner {
            world: FxRapierWorld2D::new(),
            last_step: None,
            pending_family_deltas: Vec::new(),
            next_pending_family_delta_id: 1,
            snapshot_scratch: Vec::new(),
        };
        Box::into_raw(Box::new(world)).cast::<AlchemyFxRapierWorld>()
    })) {
        Ok(world) => world,
        Err(_) => ptr::null_mut(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_fx_world_destroy(world: *mut AlchemyFxRapierWorld) {
    if !world.is_null() {
        unsafe {
            drop(Box::from_raw(world.cast::<AlchemyFxRapierWorldInner>()));
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_fx_world_set_gravity(
    world: *mut AlchemyFxRapierWorld,
    gravity: AlchemyRapierVec2,
) -> AlchemyRapierStatus {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_fx_world(world) else {
            return AlchemyRapierStatus::NullPointer;
        };
        world
            .world
            .set_gravity(vector![gravity.x, gravity.y].into());
        AlchemyRapierStatus::Ok
    })) {
        Ok(status) => status,
        Err(_) => AlchemyRapierStatus::Panic,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_fx_world_set_timestep(
    world: *mut AlchemyFxRapierWorld,
    dt: f32,
) -> AlchemyRapierStatus {
    match catch_unwind(AssertUnwindSafe(|| {
        if !dt.is_finite() || dt <= 0.0 {
            return AlchemyRapierStatus::InvalidArgument;
        }
        let Ok(world) = to_fx_world(world) else {
            return AlchemyRapierStatus::NullPointer;
        };
        world.world.integration_parameters_mut().dt = dt;
        AlchemyRapierStatus::Ok
    })) {
        Ok(status) => status,
        Err(_) => AlchemyRapierStatus::Panic,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_fx_world_set_snapshot_mode(
    world: *mut AlchemyFxRapierWorld,
    mode: AlchemyFxSnapshotMode,
) -> AlchemyRapierStatus {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_fx_world(world) else {
            return AlchemyRapierStatus::NullPointer;
        };
        world.world.set_snapshot_mode(snapshot_mode_from_ffi(mode));
        AlchemyRapierStatus::Ok
    })) {
        Ok(status) => status,
        Err(_) => AlchemyRapierStatus::Panic,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_fx_world_update_destructible_from_voxels(
    world: *mut AlchemyFxRapierWorld,
    desc: AlchemyFxVoxelDestructibleDesc,
) -> AlchemyRapierStatus {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_fx_world(world) else {
            return AlchemyRapierStatus::NullPointer;
        };
        let asset = match authored_fx_asset_from_desc(desc) {
            Ok(asset) => asset,
            Err(status) => return status,
        };
        match world
            .world
            .update_destructible_from_voxels(FxFamilyId(desc.family_id), asset)
        {
            Ok((_sync_report, family_deltas)) => {
                for delta in &family_deltas {
                    push_pending_family_delta_from_rapier(world, delta);
                }
                AlchemyRapierStatus::Ok
            }
            Err(error) => fx_error_status(error),
        }
    })) {
        Ok(status) => status,
        Err(_) => AlchemyRapierStatus::Panic,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_fx_world_remove_destructible(
    world: *mut AlchemyFxRapierWorld,
    family_id: u32,
) -> AlchemyRapierStatus {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_fx_world(world) else {
            return AlchemyRapierStatus::NullPointer;
        };
        match world.world.remove_destructible(FxFamilyId(family_id)) {
            Ok(()) => {
                let row = AlchemyFxFamilyDeltaRow {
                    delta_sequence: world.world.tick(),
                    delta_id: next_pending_family_delta_id(world),
                    kind: AlchemyFxFamilyDeltaKind::Destroyed,
                    family_id,
                    parent_family_id: 0,
                    body_mode: AlchemyFxFamilyBodyMode::Destroyed,
                    occupied_voxel_count: 0,
                    actor_count: 0,
                };
                push_pending_family_delta(world, row);
                AlchemyRapierStatus::Ok
            }
            Err(error) => fx_error_status(error),
        }
    })) {
        Ok(status) => status,
        Err(_) => AlchemyRapierStatus::Panic,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_fx_world_read_family_state(
    world: *mut AlchemyFxRapierWorld,
    family_id: u32,
) -> AlchemyFxFamilyStateResult {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_fx_world(world) else {
            return AlchemyFxFamilyStateResult {
                status: AlchemyRapierStatus::NullPointer,
                ..AlchemyFxFamilyStateResult::default()
            };
        };
        match world.world.read_family_state(FxFamilyId(family_id)) {
            Ok(state) => {
                let body_handle = state.single_body_handle.unwrap_or_default();
                let collider_handle = state.single_collider_handle.unwrap_or_default();
                AlchemyFxFamilyStateResult {
                    status: AlchemyRapierStatus::Ok,
                    family_id: state.family_id.0,
                    actor_count: state.actor_count,
                    body_count: state.body_count,
                    collider_count: state.collider_count,
                    body_mode: state
                        .single_body_type
                        .map(fx_family_body_mode_to_ffi)
                        .unwrap_or_default(),
                    body_handle_index: body_handle.index,
                    body_handle_generation: body_handle.generation,
                    body_packed_id: body_handle.packed_id,
                    collider_handle_index: collider_handle.index,
                    collider_handle_generation: collider_handle.generation,
                    collider_packed_id: collider_handle.packed_id,
                    has_single_body: state.single_body_handle.is_some() as u8,
                    has_single_collider: state.single_collider_handle.is_some() as u8,
                    occupied_node_count: state.occupied_node_count,
                    occupied_voxel_count: state.occupied_voxel_count,
                }
            }
            Err(error) => AlchemyFxFamilyStateResult {
                status: fx_error_status(error),
                ..AlchemyFxFamilyStateResult::default()
            },
        }
    })) {
        Ok(result) => result,
        Err(_) => AlchemyFxFamilyStateResult {
            status: AlchemyRapierStatus::Panic,
            ..AlchemyFxFamilyStateResult::default()
        },
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_fx_world_queue_fracture_field(
    world: *mut AlchemyFxRapierWorld,
    desc: AlchemyFxFractureFieldDesc,
) -> AlchemyRapierStatus {
    match catch_unwind(AssertUnwindSafe(|| {
        if !desc.radius.is_finite()
            || desc.radius < 0.0
            || !desc.center.x.is_finite()
            || !desc.center.y.is_finite()
            || !desc.force.x.is_finite()
            || !desc.force.y.is_finite()
            || !desc.health_loss.is_finite()
            || !desc.effective_length_loss.is_finite()
        {
            return AlchemyRapierStatus::InvalidArgument;
        }
        let Ok(world) = to_fx_world(world) else {
            return AlchemyRapierStatus::NullPointer;
        };
        let center = Vec2::new(desc.center.x, desc.center.y);
        let mut field = match desc.mode {
            AlchemyFxFractureFieldMode::Stress => {
                FractureField2D::stress(center, desc.radius, Vec2::new(desc.force.x, desc.force.y))
            }
            AlchemyFxFractureFieldMode::DirectDamage => {
                FractureField2D::direct_damage(center, desc.radius, desc.health_loss)
                    .with_effective_length_loss(desc.effective_length_loss)
            }
        }
        .with_source(damage_source_from_ffi(desc.source));
        if desc.has_family != 0 {
            field = field.with_family(FxFamilyId(desc.family_id));
        }
        world.world.queue_fracture_field(field);
        AlchemyRapierStatus::Ok
    })) {
        Ok(status) => status,
        Err(_) => AlchemyRapierStatus::Panic,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_fx_world_clear_fracture_fields(
    world: *mut AlchemyFxRapierWorld,
) -> AlchemyRapierStatus {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_fx_world(world) else {
            return AlchemyRapierStatus::NullPointer;
        };
        world.world.clear_fracture_fields();
        AlchemyRapierStatus::Ok
    })) {
        Ok(status) => status,
        Err(_) => AlchemyRapierStatus::Panic,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_fx_world_step(
    world: *mut AlchemyFxRapierWorld,
    out_report: *mut AlchemyFxStepReport,
) -> AlchemyRapierStatus {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_fx_world(world) else {
            return AlchemyRapierStatus::NullPointer;
        };
        match world.world.step_with_diagnostics() {
            Ok(step) => {
                let budget = step.diagnostics.budget;
                let report = AlchemyFxStepReport {
                    status: AlchemyRapierStatus::Ok,
                    tick: world.world.tick(),
                    quick_impact_count: step.report.quick_impacts.len(),
                    contact_impulse_count: step.report.contact_impulses.len(),
                    joint_feedback_count: step.report.joint_feedback.len(),
                    fracture_field_effect_count: step.report.fracture_field_effects.len(),
                    stress_input_count: step.report.stress_inputs.len(),
                    fracture_event_count: step.report.fracture_events.len(),
                    split_event_count: step.report.split_events.len(),
                    impulse_joint_handle_replacement_count: step
                        .report
                        .impulse_joint_handle_replacements
                        .len(),
                    occupied_voxel_count: budget
                        .map(|budget| budget.occupied_voxels)
                        .unwrap_or_default(),
                    occupied_voxel_budget: budget
                        .map(|budget| budget.occupied_voxel_budget)
                        .unwrap_or_default(),
                    active_body_count: budget
                        .map(|budget| budget.active_bodies)
                        .unwrap_or_default(),
                    active_body_budget: budget
                        .map(|budget| budget.active_body_budget)
                        .unwrap_or_default(),
                };
                world.last_step = Some(step);
                if !out_report.is_null() {
                    unsafe {
                        *out_report = report;
                    }
                }
                AlchemyRapierStatus::Ok
            }
            Err(error) => fx_error_status(error),
        }
    })) {
        Ok(status) => status,
        Err(_) => AlchemyRapierStatus::Panic,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_fx_world_read_split_event_rows(
    world: *mut AlchemyFxRapierWorld,
    rows: *mut AlchemyFxSplitEventRow,
    capacity: usize,
) -> AlchemyFxSplitEventReadResult {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_fx_world(world) else {
            return AlchemyFxSplitEventReadResult {
                status: AlchemyRapierStatus::NullPointer,
                row_count: 0,
                written_count: 0,
            };
        };
        let Some(events) = fx_split_events(world) else {
            return AlchemyFxSplitEventReadResult {
                status: AlchemyRapierStatus::Ok,
                row_count: 0,
                written_count: 0,
            };
        };
        let written_count = if rows.is_null() {
            0
        } else {
            events.len().min(capacity)
        };
        for (index, event) in events.iter().take(written_count).enumerate() {
            unsafe {
                *rows.add(index) = AlchemyFxSplitEventRow {
                    event_id: event.event_id.0,
                    family_id: event.family.0,
                    internal_split_child_count: event.created_children.len(),
                    fragment_count: event.fragments.len(),
                };
            }
        }
        AlchemyFxSplitEventReadResult {
            status: AlchemyRapierStatus::Ok,
            row_count: events.len(),
            written_count,
        }
    })) {
        Ok(result) => result,
        Err(_) => AlchemyFxSplitEventReadResult {
            status: AlchemyRapierStatus::Panic,
            row_count: 0,
            written_count: 0,
        },
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_fx_world_read_family_delta_rows(
    world: *mut AlchemyFxRapierWorld,
    rows: *mut AlchemyFxFamilyDeltaRow,
    capacity: usize,
) -> AlchemyFxFamilyDeltaReadResult {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_fx_world(world) else {
            return AlchemyFxFamilyDeltaReadResult {
                status: AlchemyRapierStatus::NullPointer,
                row_count: 0,
                written_count: 0,
            };
        };
        let step_deltas = fx_family_deltas(world).unwrap_or(&[]);
        let row_count = world.pending_family_deltas.len() + step_deltas.len();
        let written_count = if rows.is_null() {
            0
        } else {
            row_count.min(capacity)
        };
        let delta_sequence = world.world.tick();
        let mut index = 0usize;
        for row in world.pending_family_deltas.iter().take(written_count) {
            unsafe {
                *rows.add(index) = *row;
            }
            index += 1;
        }
        for delta in step_deltas.iter().take(written_count.saturating_sub(index)) {
            unsafe {
                *rows.add(index) = AlchemyFxFamilyDeltaRow {
                    delta_sequence,
                    delta_id: delta.delta_id,
                    kind: fx_family_delta_kind_to_ffi(delta.kind),
                    family_id: delta.family_id.0,
                    parent_family_id: delta.parent_family_id.0,
                    body_mode: fx_family_delta_body_mode_to_ffi(delta.body_mode),
                    occupied_voxel_count: delta.occupied_voxel_count,
                    actor_count: delta.actor_count,
                };
            }
            index += 1;
        }
        AlchemyFxFamilyDeltaReadResult {
            status: AlchemyRapierStatus::Ok,
            row_count,
            written_count,
        }
    })) {
        Ok(result) => result,
        Err(_) => AlchemyFxFamilyDeltaReadResult {
            status: AlchemyRapierStatus::Panic,
            row_count: 0,
            written_count: 0,
        },
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_fx_world_capture_snapshot(
    world: *mut AlchemyFxRapierWorld,
    out_slice: *mut AlchemyFxByteSlice,
) -> AlchemyRapierStatus {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_fx_world(world) else {
            return AlchemyRapierStatus::NullPointer;
        };
        match world.world.snapshot() {
            Ok(bytes) => {
                world.snapshot_scratch = bytes;
                if !out_slice.is_null() {
                    unsafe {
                        *out_slice = AlchemyFxByteSlice {
                            data: world.snapshot_scratch.as_ptr(),
                            len: world.snapshot_scratch.len(),
                        };
                    }
                }
                AlchemyRapierStatus::Ok
            }
            Err(error) => fx_error_status(error),
        }
    })) {
        Ok(status) => status,
        Err(_) => AlchemyRapierStatus::Panic,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_fx_world_restore_snapshot(
    bytes: *const u8,
    len: usize,
) -> *mut AlchemyFxRapierWorld {
    match catch_unwind(AssertUnwindSafe(|| {
        if bytes.is_null() && len != 0 {
            return ptr::null_mut();
        }
        let bytes = if len == 0 {
            &[]
        } else {
            unsafe { slice::from_raw_parts(bytes, len) }
        };
        match FxRapierWorld2D::restore_snapshot(bytes) {
            Ok(world) => Box::into_raw(Box::new(AlchemyFxRapierWorldInner {
                world,
                last_step: None,
                pending_family_deltas: Vec::new(),
                next_pending_family_delta_id: 1,
                snapshot_scratch: Vec::new(),
            }))
            .cast::<AlchemyFxRapierWorld>(),
            Err(_) => ptr::null_mut(),
        }
    })) {
        Ok(world) => world,
        Err(_) => ptr::null_mut(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_fx_world_version_string() -> *const c_char {
    concat!("alchemy_fx_rapier_ffi ", env!("CARGO_PKG_VERSION"), "\0")
        .as_ptr()
        .cast::<c_char>()
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_version_string() -> *const c_char {
    concat!("alchemy_rapier_ffi ", env!("CARGO_PKG_VERSION"), "\0")
        .as_ptr()
        .cast::<c_char>()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_body_desc(rotation: f32) -> AlchemyRapierBodyDesc {
        AlchemyRapierBodyDesc {
            body_type: AlchemyRapierBodyType::Dynamic,
            position: AlchemyRapierVec2::default(),
            rotation,
            linear_velocity: AlchemyRapierVec2::default(),
            angular_velocity: 0.0,
            linear_damping: 0.0,
            angular_damping: 0.0,
            gravity_scale: 0.0,
            local_center_of_mass: AlchemyRapierVec2::default(),
            mass: 1.0,
            inertia: 1.0,
            fixed_rotation: 0,
            can_sleep: 0,
            write_transform: 1,
            write_velocity: 1,
            wake_up: 1,
            sleep: 0,
            use_collider_mass: 0,
            user_data: 0,
        }
    }

    fn test_motor(
        body_a: AlchemyRapierRigidBodyHandle,
        body_b: AlchemyRapierRigidBodyHandle,
    ) -> AlchemyRapierRagdollMotorDesc {
        AlchemyRapierRagdollMotorDesc {
            body_a,
            body_b,
            target_relative_angle: 0.5,
            target_relative_angular_velocity: 0.0,
            reference_relative_angle: 0.0,
            stiffness: 10.0,
            damping: 0.0,
            max_torque: 2.0,
            delta_seconds: 0.25,
        }
    }

    fn create_motor_test_body(
        world: *mut AlchemyRapierWorld,
        rotation: f32,
        angular_velocity: f32,
    ) -> AlchemyRapierRigidBodyHandle {
        let mut desc = test_body_desc(rotation);
        desc.angular_velocity = angular_velocity;
        let body = alchemy_rapier_create_body(world, desc);
        assert_eq!(body.status, AlchemyRapierStatus::Ok);
        assert_eq!(
            alchemy_rapier_create_capsule_collider(world, body.handle, 0.5, 0.5).status,
            AlchemyRapierStatus::Ok
        );
        body.handle
    }

    fn solve_shared_body_motors(
        reverse_order: bool,
    ) -> ([AlchemyRapierRagdollMotorResult; 2], [f32; 3]) {
        let world = alchemy_rapier_create_world();
        assert_eq!(world.status, AlchemyRapierStatus::Ok);
        let left = create_motor_test_body(world.world, 0.1, 0.7);
        let center = create_motor_test_body(world.world, -0.2, -0.45);
        let right = create_motor_test_body(world.world, 0.05, 0.25);

        let mut left_motor = test_motor(left, center);
        left_motor.target_relative_angle = 0.35;
        left_motor.target_relative_angular_velocity = 0.2;
        left_motor.stiffness = 12.0;
        left_motor.damping = 8.0;
        left_motor.max_torque = 100.0;
        let mut right_motor = test_motor(center, right);
        right_motor.target_relative_angle = -0.4;
        right_motor.target_relative_angular_velocity = -0.1;
        right_motor.stiffness = 9.0;
        right_motor.damping = 6.0;
        right_motor.max_torque = 100.0;
        let motors = if reverse_order {
            [right_motor, left_motor]
        } else {
            [left_motor, right_motor]
        };
        let mut batch_results = [AlchemyRapierRagdollMotorResult::default(); 2];
        assert_eq!(
            alchemy_rapier_apply_ragdoll_motor_batch(
                world.world,
                motors.as_ptr(),
                motors.len(),
                batch_results.as_mut_ptr(),
                batch_results.len(),
            ),
            AlchemyRapierStatus::Ok
        );

        let mut results_by_joint = [AlchemyRapierRagdollMotorResult::default(); 2];
        for (motor, result) in motors.iter().zip(batch_results) {
            let joint_index = if motor.body_a == left { 0 } else { 1 };
            results_by_joint[joint_index] = result;
        }
        let angular_velocities = [
            alchemy_rapier_body_state(world.world, left).angular_velocity,
            alchemy_rapier_body_state(world.world, center).angular_velocity,
            alchemy_rapier_body_state(world.world, right).angular_velocity,
        ];
        assert_eq!(
            alchemy_rapier_destroy_world(world.world),
            AlchemyRapierStatus::Ok
        );
        (results_by_joint, angular_velocities)
    }

    #[test]
    fn ragdoll_motor_batch_applies_clamped_opposing_impulses() {
        let world = alchemy_rapier_create_world();
        assert_eq!(world.status, AlchemyRapierStatus::Ok);
        let body_a = alchemy_rapier_create_body(world.world, test_body_desc(0.0));
        let body_b = alchemy_rapier_create_body(world.world, test_body_desc(0.0));
        assert_eq!(body_a.status, AlchemyRapierStatus::Ok);
        assert_eq!(body_b.status, AlchemyRapierStatus::Ok);
        assert_eq!(
            alchemy_rapier_create_capsule_collider(world.world, body_a.handle, 0.5, 0.5).status,
            AlchemyRapierStatus::Ok
        );
        assert_eq!(
            alchemy_rapier_create_capsule_collider(world.world, body_b.handle, 0.5, 0.5).status,
            AlchemyRapierStatus::Ok
        );

        let motor = test_motor(body_a.handle, body_b.handle);
        let mut result = AlchemyRapierRagdollMotorResult::default();
        assert_eq!(
            alchemy_rapier_apply_ragdoll_motor_batch(world.world, &motor, 1, &mut result, 1),
            AlchemyRapierStatus::Ok
        );
        assert!((result.angle_error - 0.5).abs() < 0.0001);
        assert!((result.applied_torque - 2.0).abs() < 0.0001);
        assert_eq!(
            alchemy_rapier_step(world.world, 1.0 / 60.0, 1).status,
            AlchemyRapierStatus::Ok
        );

        let state_a = alchemy_rapier_body_state(world.world, body_a.handle);
        let state_b = alchemy_rapier_body_state(world.world, body_b.handle);
        assert!(
            state_a.angular_velocity > 0.0,
            "expected positive angular velocity, got {}",
            state_a.angular_velocity
        );
        assert!(
            state_b.angular_velocity < 0.0,
            "expected negative angular velocity, got {}",
            state_b.angular_velocity
        );
        assert_eq!(
            alchemy_rapier_destroy_world(world.world),
            AlchemyRapierStatus::Ok
        );
    }

    #[test]
    fn ragdoll_motor_batch_rejects_invalid_handle_without_partial_impulses() {
        let world = alchemy_rapier_create_world();
        assert_eq!(world.status, AlchemyRapierStatus::Ok);
        let body_a = alchemy_rapier_create_body(world.world, test_body_desc(0.0));
        let body_b = alchemy_rapier_create_body(world.world, test_body_desc(0.0));
        assert_eq!(body_a.status, AlchemyRapierStatus::Ok);
        assert_eq!(body_b.status, AlchemyRapierStatus::Ok);

        let valid_motor = test_motor(body_a.handle, body_b.handle);
        let invalid_motor = test_motor(
            body_a.handle,
            AlchemyRapierRigidBodyHandle {
                index: u32::MAX,
                generation: u32::MAX,
            },
        );
        let motors = [valid_motor, invalid_motor];
        let mut results = [AlchemyRapierRagdollMotorResult::default(); 2];
        assert_eq!(
            alchemy_rapier_apply_ragdoll_motor_batch(
                world.world,
                motors.as_ptr(),
                motors.len(),
                results.as_mut_ptr(),
                results.len(),
            ),
            AlchemyRapierStatus::InvalidHandle
        );

        let state_a = alchemy_rapier_body_state(world.world, body_a.handle);
        let state_b = alchemy_rapier_body_state(world.world, body_b.handle);
        assert!(state_a.angular_velocity.abs() < 0.0001);
        assert!(state_b.angular_velocity.abs() < 0.0001);
        assert_eq!(
            alchemy_rapier_destroy_world(world.world),
            AlchemyRapierStatus::Ok
        );
    }

    #[test]
    fn ragdoll_motor_batch_is_order_independent_for_shared_bodies() {
        let (forward_results, forward_velocities) = solve_shared_body_motors(false);
        let (reverse_results, reverse_velocities) = solve_shared_body_motors(true);

        for joint_index in 0..forward_results.len() {
            assert!(
                (forward_results[joint_index].angle_error
                    - reverse_results[joint_index].angle_error)
                    .abs()
                    < 0.000001
            );
            assert!(
                (forward_results[joint_index].applied_torque
                    - reverse_results[joint_index].applied_torque)
                    .abs()
                    < 0.000001
            );
        }
        for body_index in 0..forward_velocities.len() {
            assert!(
                (forward_velocities[body_index] - reverse_velocities[body_index]).abs() < 0.00001
            );
        }
    }

    #[test]
    fn ragdoll_motor_batch_accepts_zero_and_exact_maximum_counts() {
        let world = alchemy_rapier_create_world();
        assert_eq!(world.status, AlchemyRapierStatus::Ok);
        assert_eq!(
            alchemy_rapier_apply_ragdoll_motor_batch(
                world.world,
                ptr::null(),
                0,
                ptr::null_mut(),
                0,
            ),
            AlchemyRapierStatus::Ok
        );

        let body_a = create_motor_test_body(world.world, 0.0, 0.0);
        let body_b = create_motor_test_body(world.world, 0.0, 0.0);
        let motors = [test_motor(body_a, body_b); MAX_RAGDOLL_MOTOR_BATCH_COUNT];
        let mut results =
            [AlchemyRapierRagdollMotorResult::default(); MAX_RAGDOLL_MOTOR_BATCH_COUNT];
        assert_eq!(
            alchemy_rapier_apply_ragdoll_motor_batch(
                world.world,
                motors.as_ptr(),
                motors.len(),
                results.as_mut_ptr(),
                results.len(),
            ),
            AlchemyRapierStatus::Ok
        );
        assert!(
            results
                .iter()
                .all(|result| result.applied_torque.is_finite())
        );
        assert_eq!(
            alchemy_rapier_destroy_world(world.world),
            AlchemyRapierStatus::Ok
        );
    }

    #[test]
    fn ragdoll_motor_batch_rejects_oversized_and_insufficient_result_capacity() {
        let world = alchemy_rapier_create_world();
        assert_eq!(world.status, AlchemyRapierStatus::Ok);
        assert_eq!(
            alchemy_rapier_apply_ragdoll_motor_batch(
                world.world,
                ptr::null(),
                MAX_RAGDOLL_MOTOR_BATCH_COUNT + 1,
                ptr::null_mut(),
                0,
            ),
            AlchemyRapierStatus::InvalidArgument
        );

        let body_a = create_motor_test_body(world.world, 0.0, 0.0);
        let body_b = create_motor_test_body(world.world, 0.0, 0.0);
        let motor = test_motor(body_a, body_b);
        let mut sentinel = AlchemyRapierRagdollMotorResult {
            angle_error: 17.0,
            applied_torque: 19.0,
        };
        assert_eq!(
            alchemy_rapier_apply_ragdoll_motor_batch(world.world, &motor, 1, &mut sentinel, 0),
            AlchemyRapierStatus::InvalidArgument
        );
        assert_eq!(sentinel.angle_error, 17.0);
        assert_eq!(sentinel.applied_torque, 19.0);
        assert!(
            alchemy_rapier_body_state(world.world, body_a)
                .angular_velocity
                .abs()
                < 0.0001
        );
        assert!(
            alchemy_rapier_body_state(world.world, body_b)
                .angular_velocity
                .abs()
                < 0.0001
        );
        assert_eq!(
            alchemy_rapier_destroy_world(world.world),
            AlchemyRapierStatus::Ok
        );
    }

    #[test]
    fn ragdoll_motor_batch_rejects_nan_without_side_effects() {
        let world = alchemy_rapier_create_world();
        assert_eq!(world.status, AlchemyRapierStatus::Ok);
        let body_a = create_motor_test_body(world.world, 0.0, 0.0);
        let body_b = create_motor_test_body(world.world, 0.0, 0.0);
        let mut invalid_motor = test_motor(body_a, body_b);
        invalid_motor.damping = f32::NAN;
        let motors = [test_motor(body_a, body_b), invalid_motor];
        let mut results = [AlchemyRapierRagdollMotorResult {
            angle_error: 17.0,
            applied_torque: 19.0,
        }; 2];
        assert_eq!(
            alchemy_rapier_apply_ragdoll_motor_batch(
                world.world,
                motors.as_ptr(),
                motors.len(),
                results.as_mut_ptr(),
                results.len(),
            ),
            AlchemyRapierStatus::InvalidArgument
        );
        assert!(
            results
                .iter()
                .all(|result| { result.angle_error == 17.0 && result.applied_torque == 19.0 })
        );
        assert!(
            alchemy_rapier_body_state(world.world, body_a)
                .angular_velocity
                .abs()
                < 0.0001
        );
        assert!(
            alchemy_rapier_body_state(world.world, body_b)
                .angular_velocity
                .abs()
                < 0.0001
        );
        assert_eq!(
            alchemy_rapier_destroy_world(world.world),
            AlchemyRapierStatus::Ok
        );
    }

    #[test]
    fn contact_material_setter_writes_authored_values() {
        let world = alchemy_rapier_create_world();
        assert_eq!(world.status, AlchemyRapierStatus::Ok);
        let materials = [AlchemyRapierContactMaterialDesc {
            material_id: 42,
            friction: 0.75,
            restitution: 0.2,
            friction_combine_rule: AlchemyRapierCoefficientCombineRule::Multiply,
            restitution_combine_rule: AlchemyRapierCoefficientCombineRule::Max,
            hardness: 3.5,
        }];
        assert_eq!(
            alchemy_rapier_set_contact_materials(world.world, materials.as_ptr(), materials.len()),
            AlchemyRapierStatus::Ok
        );
        {
            let inner = to_inner(world.world).expect("world remains valid");
            let material = inner
                .contact_materials
                .get(&42)
                .expect("authored material must be stored");
            assert_eq!(material.friction, 0.75);
            assert_eq!(material.restitution, 0.2);
            assert_eq!(
                material.friction_combine_rule,
                CoefficientCombineRule::Multiply
            );
            assert_eq!(
                material.restitution_combine_rule,
                CoefficientCombineRule::Max
            );
            assert_eq!(material.hardness, 3.5);
        }
        assert_eq!(
            alchemy_rapier_destroy_world(world.world),
            AlchemyRapierStatus::Ok
        );
    }

    #[test]
    fn zero_frequency_revolute_joint_uses_finite_hard_defaults() {
        let world = alchemy_rapier_create_world();
        assert_eq!(world.status, AlchemyRapierStatus::Ok);
        let mut left_desc = test_body_desc(0.0);
        left_desc.position.x = -0.5;
        let mut right_desc = test_body_desc(0.0);
        right_desc.position.x = 0.5;
        let left = alchemy_rapier_create_body(world.world, left_desc);
        let right = alchemy_rapier_create_body(world.world, right_desc);
        assert_eq!(left.status, AlchemyRapierStatus::Ok);
        assert_eq!(right.status, AlchemyRapierStatus::Ok);
        assert_eq!(
            alchemy_rapier_set_max_ccd_substeps(world.world, 0),
            AlchemyRapierStatus::InvalidArgument
        );
        assert_eq!(
            alchemy_rapier_set_max_ccd_substeps(world.world, 8),
            AlchemyRapierStatus::Ok
        );
        assert_eq!(
            alchemy_rapier_set_body_ccd_enabled(world.world, right.handle, 1),
            AlchemyRapierStatus::Ok
        );

        let joint = alchemy_rapier_create_revolute_joint(
            world.world,
            AlchemyRapierRevoluteJointDesc {
                body1: left.handle,
                body2: right.handle,
                local_anchor1: AlchemyRapierVec2 { x: 0.5, y: 0.0 },
                local_anchor2: AlchemyRapierVec2 { x: -0.5, y: 0.0 },
                natural_frequency: 0.0,
                damping_ratio: 0.0,
                contacts_enabled: 0,
                wake_up: 1,
            },
        );
        assert_eq!(joint.status, AlchemyRapierStatus::Ok);
        assert_eq!(
            alchemy_rapier_apply_body_linear_impulse(
                world.world,
                right.handle,
                AlchemyRapierVec2 { x: 2.0, y: 0.0 },
                1,
            ),
            AlchemyRapierStatus::Ok
        );
        assert_eq!(
            alchemy_rapier_step(world.world, 1.0 / 60.0, 1).status,
            AlchemyRapierStatus::Ok
        );

        let left_state = alchemy_rapier_body_state(world.world, left.handle);
        let right_state = alchemy_rapier_body_state(world.world, right.handle);
        assert!(left_state.position.x.is_finite());
        assert!(left_state.linear_velocity.x.is_finite());
        assert!(right_state.position.x.is_finite());
        assert!(right_state.linear_velocity.x.is_finite());
        assert_eq!(
            alchemy_rapier_destroy_world(world.world),
            AlchemyRapierStatus::Ok
        );
    }
}
