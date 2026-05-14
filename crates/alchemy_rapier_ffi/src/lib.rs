use fracture_voxel::{AuthoredVoxelAsset, VoxelAuthoringInput, author_voxel_asset};
use rapier2d::parry::query::ShapeCastOptions;
use rapier2d::prelude::*;
use std::collections::HashMap;
use std::os::raw::c_char;
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::ptr;
use std::slice;

const QUERY_SOURCE_TERRAIN: u32 = 1;
const QUERY_SOURCE_DYNAMIC_RIGIDBODY: u32 = 1 << 1;

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

#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AlchemyRapierBodyType {
    Dynamic = 0,
    Kinematic = 1,
    Fixed = 2,
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
#[derive(Clone, Copy, Debug, Default)]
pub struct AlchemyRapierVec2 {
    pub x: f32,
    pub y: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AlchemyRapierTerrainDesc {
    pub chunk_x: i32,
    pub chunk_y: i32,
    pub source_world_origin_x: i32,
    pub source_world_origin_y: i32,
    pub local_origin_x: i32,
    pub local_origin_y: i32,
    pub revision: i64,
    pub width: i32,
    pub height: i32,
    pub pixel_size: f32,
    pub topology_revision: u64,
    pub topology_version: u32,
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
pub struct AlchemyRapierMassResult {
    pub status: AlchemyRapierStatus,
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
#[derive(Clone, Copy, Debug)]
pub struct AlchemyRapierTerrainApplyResult {
    pub status: AlchemyRapierStatus,
    pub solid_count: usize,
    pub terrain_chunk_count: usize,
    pub terrain_collider_count: usize,
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
pub struct AlchemyRapierWorld {
    _private: [u8; 0],
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
        }
    }

    fn step_once(&mut self, dt: f32) {
        self.integration_parameters.dt = dt;
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
            &(),
            &(),
        );
    }
}

fn to_inner<'a>(
    world: *mut AlchemyRapierWorld,
) -> Result<&'a mut AlchemyRapierWorldInner, AlchemyRapierStatus> {
    if world.is_null() {
        return Err(AlchemyRapierStatus::NullPointer);
    }
    Ok(unsafe { &mut *world.cast::<AlchemyRapierWorldInner>() })
}

fn handle_from_ffi(handle: AlchemyRapierRigidBodyHandle) -> RigidBodyHandle {
    RigidBodyHandle::from_raw_parts(handle.index, handle.generation)
}

fn collider_handle_from_ffi(handle: AlchemyRapierColliderHandle) -> ColliderHandle {
    ColliderHandle::from_raw_parts(handle.index, handle.generation)
}

fn handle_to_ffi(handle: RigidBodyHandle) -> AlchemyRapierRigidBodyHandle {
    let (index, generation) = handle.into_raw_parts();
    AlchemyRapierRigidBodyHandle { index, generation }
}

fn collider_handle_to_ffi(handle: ColliderHandle) -> AlchemyRapierColliderHandle {
    let (index, generation) = handle.into_raw_parts();
    AlchemyRapierColliderHandle { index, generation }
}

fn pack_parts(index: u32, generation: u32) -> u64 {
    u64::from(index) | (u64::from(generation) << 32)
}

fn pack_body_handle(handle: RigidBodyHandle) -> u64 {
    let (index, generation) = handle.into_raw_parts();
    pack_parts(index, generation)
}

fn pack_collider_handle(handle: ColliderHandle) -> u64 {
    let (index, generation) = handle.into_raw_parts();
    pack_parts(index, generation)
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

fn pose_translation(value: Vector) -> Pose {
    Pose::from_parts(value, Rotation::identity())
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
        AlchemyRapierBodyType::Fixed => RigidBodyType::Fixed,
        AlchemyRapierBodyType::Dynamic => RigidBodyType::Dynamic,
    }
}

fn body_type_from_rapier(value: RigidBodyType) -> AlchemyRapierBodyType {
    match value {
        RigidBodyType::Fixed => AlchemyRapierBodyType::Fixed,
        RigidBodyType::KinematicPositionBased | RigidBodyType::KinematicVelocityBased => {
            AlchemyRapierBodyType::Kinematic
        }
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
        .can_sleep(desc.can_sleep != 0)
        .additional_mass_properties(MassProperties::new(
            vector(desc.local_center_of_mass),
            sanitize_positive(desc.mass, 1.0),
            sanitize_positive(desc.inertia, 1.0),
        ));
    if desc.fixed_rotation != 0 {
        builder = builder.lock_rotations();
    }
    if desc.sleep != 0 {
        builder = builder.sleeping(true);
    }
    builder
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
    body.set_additional_mass_properties(
        MassProperties::new(
            vector(desc.local_center_of_mass),
            sanitize_positive(desc.mass, 1.0),
            sanitize_positive(desc.inertia, 1.0),
        ),
        wake_up,
    );
    set_body_can_sleep(body, desc.can_sleep != 0);
    if desc.sleep != 0 {
        body.sleep();
    } else if wake_up {
        body.wake_up(true);
    }
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
        position: ffi_vec(body.translation()),
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

fn empty_terrain_apply_result(status: AlchemyRapierStatus) -> AlchemyRapierTerrainApplyResult {
    AlchemyRapierTerrainApplyResult {
        status,
        solid_count: 0,
        terrain_chunk_count: 0,
        terrain_collider_count: 0,
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

fn terrain_key(desc: AlchemyRapierTerrainDesc) -> TerrainKey {
    TerrainKey {
        x: desc.chunk_x,
        y: desc.chunk_y,
    }
}

fn remove_terrain_chunk(world: &mut AlchemyRapierWorldInner, key: TerrainKey) {
    if let Some(existing) = world.terrain_chunks.remove(&key) {
        world.terrain_by_collider.remove(&existing.collider);
        let _ = world.colliders.remove(
            existing.collider,
            &mut world.islands,
            &mut world.bodies,
            true,
        );
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

fn terrain_cell_index(state: &TerrainChunkState, x: u32, y: u32) -> usize {
    (y as usize) * (state.width as usize) + (x as usize)
}

fn terrain_cell_occupied(state: &TerrainChunkState, x: i32, y: i32) -> bool {
    if x < 0 || y < 0 || x as u32 >= state.width || y as u32 >= state.height {
        return false;
    }
    state.occupancy[terrain_cell_index(state, x as u32, y as u32)]
}

fn terrain_world_cell(state: &TerrainChunkState, point: Vector) -> (i32, i32) {
    let local_x =
        ((point.x - state.source_world_origin_x as f32) / state.pixel_size).floor() as i32;
    let local_y =
        ((point.y - state.source_world_origin_y as f32) / state.pixel_size).floor() as i32;
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
            let min_x = state.source_world_origin_x as f32 + x as f32 * state.pixel_size;
            let min_y = state.source_world_origin_y as f32 + y as f32 * state.pixel_size;
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
            let center = Vector::new(
                state.source_world_origin_x as f32 + (x as f32 + 0.5) * state.pixel_size,
                state.source_world_origin_y as f32 + (y as f32 + 0.5) * state.pixel_size,
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
    point
        - Vector::new(
            state.source_world_origin_x as f32,
            state.source_world_origin_y as f32,
        )
}

fn dynamic_query_body(
    world: &AlchemyRapierWorldInner,
    collider: &Collider,
    ignored_body: Option<RigidBodyHandle>,
) -> Option<RigidBodyHandle> {
    let body_handle = collider.parent()?;
    if ignored_body == Some(body_handle) {
        return None;
    }
    let body = world.bodies.get(body_handle)?;
    if body.body_type() == RigidBodyType::Dynamic {
        Some(body_handle)
    } else {
        None
    }
}

enum QueryTarget {
    Dynamic(RigidBodyHandle),
    Terrain(TerrainKey),
}

fn query_target(
    world: &AlchemyRapierWorldInner,
    collider_handle: ColliderHandle,
    collider: &Collider,
    ignored_body: Option<RigidBodyHandle>,
    source_mask: u32,
) -> Option<QueryTarget> {
    if (source_mask & QUERY_SOURCE_DYNAMIC_RIGIDBODY) != 0 {
        if let Some(body_handle) = dynamic_query_body(world, collider, ignored_body) {
            return Some(QueryTarget::Dynamic(body_handle));
        }
    }
    if (source_mask & QUERY_SOURCE_TERRAIN) != 0 {
        if let Some(key) = world.terrain_by_collider.get(&collider_handle) {
            return Some(QueryTarget::Terrain(*key));
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
        QueryTarget::Terrain(key) => make_terrain_query_hit(
            world,
            key,
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
pub extern "C" fn alchemy_rapier_step(
    world: *mut AlchemyRapierWorld,
    time_step: f32,
    sub_step_count: i32,
) -> AlchemyRapierStatus {
    match catch_unwind(AssertUnwindSafe(|| {
        if !time_step.is_finite() || time_step <= 0.0 || sub_step_count <= 0 {
            return AlchemyRapierStatus::InvalidArgument;
        }
        let Ok(world) = to_inner(world) else {
            return AlchemyRapierStatus::NullPointer;
        };
        let sub_steps = sub_step_count as usize;
        let dt = time_step / sub_steps as f32;
        for _ in 0..sub_steps {
            world.step_once(dt);
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
pub extern "C" fn alchemy_rapier_destroy_body(
    world: *mut AlchemyRapierWorld,
    handle: AlchemyRapierRigidBodyHandle,
) -> AlchemyRapierStatus {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return AlchemyRapierStatus::NullPointer;
        };
        let handle = handle_from_ffi(handle);
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
        let colliders = body.colliders().to_vec();
        for collider in colliders {
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
pub extern "C" fn alchemy_rapier_destroy_collider(
    world: *mut AlchemyRapierWorld,
    handle: AlchemyRapierColliderHandle,
) -> AlchemyRapierStatus {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return AlchemyRapierStatus::NullPointer;
        };
        let handle = collider_handle_from_ffi(handle);
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
pub extern "C" fn alchemy_rapier_apply_terrain(
    world: *mut AlchemyRapierWorld,
    desc: AlchemyRapierTerrainDesc,
) -> AlchemyRapierTerrainApplyResult {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return empty_terrain_apply_result(AlchemyRapierStatus::NullPointer);
        };
        if desc.width <= 0 || desc.height <= 0 {
            let key = terrain_key(desc);
            remove_terrain_chunk(world, key);
            return AlchemyRapierTerrainApplyResult {
                status: AlchemyRapierStatus::Ok,
                solid_count: 0,
                terrain_chunk_count: world.terrain_chunks.len(),
                terrain_collider_count: world.terrain_by_collider.len(),
            };
        }
        if !desc.pixel_size.is_finite() || desc.pixel_size <= 0.0 {
            return empty_terrain_apply_result(AlchemyRapierStatus::InvalidArgument);
        }

        let width = desc.width as u32;
        let height = desc.height as u32;
        let cell_count = (width as usize).saturating_mul(height as usize);
        if cell_count == 0 {
            return empty_terrain_apply_result(AlchemyRapierStatus::InvalidArgument);
        }
        let expected_word_count = cell_count.div_ceil(64);
        if desc.occupancy_words.is_null() || desc.occupancy_word_count < expected_word_count {
            return empty_terrain_apply_result(AlchemyRapierStatus::InvalidArgument);
        }

        let words = unsafe { slice::from_raw_parts(desc.occupancy_words, expected_word_count) };
        let occupancy = occupancy_from_words(width, height, words);
        let solid_count = occupancy.iter().filter(|occupied| **occupied).count();
        let key = terrain_key(desc);
        remove_terrain_chunk(world, key);
        if solid_count == 0 {
            return AlchemyRapierTerrainApplyResult {
                status: AlchemyRapierStatus::Ok,
                solid_count: 0,
                terrain_chunk_count: world.terrain_chunks.len(),
                terrain_collider_count: world.terrain_by_collider.len(),
            };
        }

        let material_ids = if !desc.material_ids.is_null() && desc.material_id_count >= cell_count {
            unsafe { slice::from_raw_parts(desc.material_ids, cell_count) }.to_vec()
        } else {
            vec![0; cell_count]
        };
        let support_mask = if !desc.support_mask.is_null() && desc.support_mask_count >= cell_count
        {
            unsafe { slice::from_raw_parts(desc.support_mask, cell_count) }.to_vec()
        } else {
            Vec::new()
        };
        let external_id = (0..cell_count)
            .map(|index| index.min(u32::MAX as usize) as u32)
            .collect::<Vec<_>>();
        let input = VoxelAuthoringInput::new(
            width,
            height,
            desc.pixel_size,
            occupancy.clone(),
            material_ids.clone(),
            material_ids.clone(),
            external_id,
        );
        let Ok(asset) = author_voxel_asset(input) else {
            return empty_terrain_apply_result(AlchemyRapierStatus::InvalidArgument);
        };

        let mut shapes = Vec::with_capacity(solid_count);
        for y in 0..height {
            for x in 0..width {
                let idx = (y as usize) * (width as usize) + (x as usize);
                if !occupancy[idx] {
                    continue;
                }
                let center = Vector::new(
                    desc.source_world_origin_x as f32 + (x as f32 + 0.5) * desc.pixel_size,
                    desc.source_world_origin_y as f32 + (y as f32 + 0.5) * desc.pixel_size,
                );
                shapes.push((
                    Pose::from_translation(center),
                    SharedShape::cuboid(desc.pixel_size * 0.5, desc.pixel_size * 0.5),
                ));
            }
        }
        if shapes.is_empty() {
            return AlchemyRapierTerrainApplyResult {
                status: AlchemyRapierStatus::Ok,
                solid_count: 0,
                terrain_chunk_count: world.terrain_chunks.len(),
                terrain_collider_count: world.terrain_by_collider.len(),
            };
        }

        let collider = ColliderBuilder::compound(shapes).build();
        let collider_handle = world.colliders.insert(collider);
        let state = TerrainChunkState {
            asset,
            collider: collider_handle,
            chunk_x: desc.chunk_x,
            chunk_y: desc.chunk_y,
            source_world_origin_x: desc.source_world_origin_x,
            source_world_origin_y: desc.source_world_origin_y,
            local_origin_x: desc.local_origin_x,
            local_origin_y: desc.local_origin_y,
            revision: desc.revision,
            width,
            height,
            pixel_size: desc.pixel_size,
            topology_revision: desc.topology_revision,
            topology_version: desc.topology_version,
            occupancy,
            material_ids,
            support_mask,
            solid_count,
        };
        world.terrain_by_collider.insert(collider_handle, key);
        world.terrain_chunks.insert(key, state);

        AlchemyRapierTerrainApplyResult {
            status: AlchemyRapierStatus::Ok,
            solid_count,
            terrain_chunk_count: world.terrain_chunks.len(),
            terrain_collider_count: world.terrain_by_collider.len(),
        }
    })) {
        Ok(result) => result,
        Err(_) => empty_terrain_apply_result(AlchemyRapierStatus::Panic),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_clear_terrain(
    world: *mut AlchemyRapierWorld,
    chunk_x: i32,
    chunk_y: i32,
) -> AlchemyRapierTerrainApplyResult {
    match catch_unwind(AssertUnwindSafe(|| {
        let Ok(world) = to_inner(world) else {
            return empty_terrain_apply_result(AlchemyRapierStatus::NullPointer);
        };
        remove_terrain_chunk(
            world,
            TerrainKey {
                x: chunk_x,
                y: chunk_y,
            },
        );
        AlchemyRapierTerrainApplyResult {
            status: AlchemyRapierStatus::Ok,
            solid_count: 0,
            terrain_chunk_count: world.terrain_chunks.len(),
            terrain_collider_count: world.terrain_by_collider.len(),
        }
    })) {
        Ok(result) => result,
        Err(_) => empty_terrain_apply_result(AlchemyRapierStatus::Panic),
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
                mass: 0.0,
                inertia: 0.0,
            };
        };
        let handle = handle_from_ffi(handle);
        let Some(body) = world.bodies.get(handle) else {
            return AlchemyRapierMassResult {
                status: AlchemyRapierStatus::InvalidHandle,
                mass: 0.0,
                inertia: 0.0,
            };
        };
        AlchemyRapierMassResult {
            status: AlchemyRapierStatus::Ok,
            mass: body.mass(),
            inertia: body.mass_properties().local_mprops.principal_inertia(),
        }
    })) {
        Ok(result) => result,
        Err(_) => AlchemyRapierMassResult {
            status: AlchemyRapierStatus::Panic,
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
            if body.body_type() != RigidBodyType::Dynamic {
                return empty_query_result(AlchemyRapierStatus::Ok);
            }

            for collider_handle in body.colliders() {
                let Some(collider) = world.colliders.get(*collider_handle) else {
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
                let Some(key) = world.terrain_by_collider.get(&collider_handle).copied() else {
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
                let Some(hit) = make_terrain_query_hit(
                    world,
                    key,
                    collider_handle,
                    point,
                    normal,
                    distance,
                    0.0,
                ) else {
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
pub extern "C" fn alchemy_rapier_version_string() -> *const c_char {
    concat!("alchemy_rapier_ffi ", env!("CARGO_PKG_VERSION"), "\0")
        .as_ptr()
        .cast::<c_char>()
}
