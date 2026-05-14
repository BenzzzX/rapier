use fracture_core::{
    CommandId, DamageSource, DeterministicOrderKey, ExternalBondId, ExternalTarget2D,
    ExternalTargetKind, ExternalTargetToken, FractureTarget, FxActorId, FxFamilyId, GridCoord,
    StaticAnchorDesc, StressInput, StressSettings, StressSolver2D, SupportNodeId,
};
use fracture_rapier::{
    FractureField2D, FxRapierWorld2D, QuickImpactSettings, StaticAnchorBodyPolicy,
    StaticAnchorConnectionDesc,
};
use fracture_voxel::{
    AuthoredVoxelAsset, NaturalVoronoi, NaturalVoronoiClusterField, VoxelAuthoringInput,
    VoxelAuthoringOptions, VoxelClusterAxis, VoxelClusterMode, VoxelClusterPolicy,
    VoxelHierarchyPolicy, author_voxel_asset_with_options,
};
use kiss3d::color::Color;
use rapier_testbed2d::{PhysicsState, SnapshotHook, Testbed, TestbedGraphics};
use rapier2d::prelude::*;
use std::{cell::RefCell, rc::Rc};

const IMPACT_FAMILY: FxFamilyId = FxFamilyId(1);
const BRIDGE_FAMILY: FxFamilyId = FxFamilyId(2);
const JOINT_FAMILY: FxFamilyId = FxFamilyId(3);
const COLLISION_LEFT_FAMILY: FxFamilyId = FxFamilyId(4);
const COLLISION_RIGHT_FAMILY: FxFamilyId = FxFamilyId(5);

const VIEWPORT_REFERENCE_WIDTH: u32 = 480;
const VIEWPORT_REFERENCE_HEIGHT: u32 = 270;
const VIEWPORT_REFERENCE_WORLD_WIDTH: f32 = 48.0;
const VIEWPORT_REFERENCE_WORLD_HEIGHT: f32 = 27.0;
const VOXEL_SIZE: f32 = VIEWPORT_REFERENCE_WORLD_WIDTH / VIEWPORT_REFERENCE_WIDTH as f32;
const CAMERA_PIXELS_PER_WORLD_UNIT: f32 =
    VIEWPORT_REFERENCE_WIDTH as f32 / VIEWPORT_REFERENCE_WORLD_WIDTH;

const WALL_VOXEL_WIDTH: u32 = 12;
const WALL_VOXEL_HEIGHT: u32 = 180;
const WALL_WORLD_WIDTH: f32 = WALL_VOXEL_WIDTH as f32 * VOXEL_SIZE;
const WALL_WORLD_HEIGHT: f32 = WALL_VOXEL_HEIGHT as f32 * VOXEL_SIZE;

const BRIDGE_VOXEL_WIDTH: u32 = 160;
const BRIDGE_VOXEL_HEIGHT: u32 = 10;
const BRIDGE_WORLD_WIDTH: f32 = BRIDGE_VOXEL_WIDTH as f32 * VOXEL_SIZE;

const JOINT_VOXEL_WIDTH: u32 = 48;
const JOINT_VOXEL_HEIGHT: u32 = 8;

const COLLIDER_VOXEL_WIDTH: u32 = 36;
const COLLIDER_VOXEL_HEIGHT: u32 = 24;

const WALL_CENTER: Vector = Vector::new(0.0, 0.0);
const WALL_SCENE_CENTER: Vector = Vector::new(-2.0, 0.0);
const BRIDGE_SCENE_CENTER: Vector = Vector::new(0.0, 1.0);
const JOINT_SCENE_CENTER: Vector = Vector::new(0.0, 3.0);
const COLLISION_SCENE_CENTER: Vector = Vector::new(0.0, 0.0);
const GRAVITY: Vector = Vector::new(0.0, -9.81);

const FRACTURE_DEMO_SNAPSHOT_ID: &str = "fracture2";
const FRACTURE_DEMO_SNAPSHOT_MAGIC: &[u8; 8] = b"FX2DEMO1";

#[derive(Clone, Copy, Debug, PartialEq)]
enum DemoDriver {
    PhysicsOnly,
    BridgeGravity {
        apply_after_tick: u64,
        load_scale: f32,
        load_step: f32,
        fractured: bool,
    },
    VoxelCollisionFractureField {
        apply_after_tick: u64,
        applied: bool,
    },
}

struct FractureDemoRuntime {
    world: FxRapierWorld2D,
    driver: DemoDriver,
}

struct FractureDemoSnapshotHook {
    runtime: Rc<RefCell<FractureDemoRuntime>>,
}

impl SnapshotHook for FractureDemoSnapshotHook {
    fn snapshot_id(&self) -> &'static str {
        FRACTURE_DEMO_SNAPSHOT_ID
    }

    fn save_snapshot(&mut self) -> Result<Vec<u8>, String> {
        let runtime = self.runtime.borrow();
        encode_demo_snapshot(&runtime.world, runtime.driver)
            .map_err(|err| format!("fracture world snapshot failed: {err}"))
    }

    fn restore_snapshot(
        &mut self,
        snapshot: &[u8],
        physics: &mut PhysicsState,
    ) -> Result<(), String> {
        let (world, driver) = decode_demo_snapshot(snapshot)?;
        let mut runtime = self.runtime.borrow_mut();
        runtime.world = world;
        runtime.driver = driver;
        mirror_fracture_world(&runtime.world, physics);
        Ok(())
    }
}

pub fn init_world(testbed: &mut Testbed) {
    init_high_speed_wall(testbed);
}

pub fn init_high_speed_wall(testbed: &mut Testbed) {
    install_fracture_world(
        testbed,
        build_high_speed_wall_world(),
        &[IMPACT_FAMILY],
        WALL_SCENE_CENTER,
        DemoDriver::PhysicsOnly,
    );
}

pub fn init_bridge_collapse(testbed: &mut Testbed) {
    install_fracture_world(
        testbed,
        build_bridge_collapse_world(),
        &[BRIDGE_FAMILY],
        BRIDGE_SCENE_CENTER,
        DemoDriver::BridgeGravity {
            apply_after_tick: 45,
            load_scale: 0.25,
            load_step: 0.20,
            fractured: false,
        },
    );
}

pub fn init_joint_pull(testbed: &mut Testbed) {
    install_fracture_world(
        testbed,
        build_joint_pull_world(),
        &[JOINT_FAMILY],
        JOINT_SCENE_CENTER,
        DemoDriver::PhysicsOnly,
    );
}

pub fn init_voxel_collision(testbed: &mut Testbed) {
    install_fracture_world(
        testbed,
        build_voxel_collision_world(),
        &[COLLISION_LEFT_FAMILY, COLLISION_RIGHT_FAMILY],
        COLLISION_SCENE_CENTER,
        DemoDriver::VoxelCollisionFractureField {
            apply_after_tick: 20,
            applied: false,
        },
    );
}

fn install_fracture_world(
    testbed: &mut Testbed,
    fracture_world: FxRapierWorld2D,
    destructible_families: &'static [FxFamilyId],
    camera_center: Vector,
    driver: DemoDriver,
) {
    let bodies = fracture_world.rigid_bodies().clone();
    let colliders = fracture_world.colliders().clone();
    testbed.set_world_with_params(
        bodies,
        colliders,
        fracture_world.impulse_joints().clone(),
        MultibodyJointSet::new(),
        fracture_world.gravity(),
        (),
    );
    color_initial_bodies(testbed, &fracture_world, destructible_families);

    let runtime = Rc::new(RefCell::new(FractureDemoRuntime {
        world: fracture_world,
        driver,
    }));
    testbed.add_snapshot_hook(FractureDemoSnapshotHook {
        runtime: Rc::clone(&runtime),
    });

    testbed.add_callback(move |graphics, physics, _, _| {
        let mut runtime = runtime.borrow_mut();
        let FractureDemoRuntime {
            world: fracture_world,
            driver,
        } = &mut *runtime;
        let mut rebuild_graphics = false;
        let step = match fracture_world.step() {
            Ok(step) => step,
            Err(err) => {
                eprintln!("fracture demo step failed: {err}");
                return;
            }
        };
        rebuild_graphics |= !step.split_events.is_empty();

        match driver {
            DemoDriver::PhysicsOnly => {}
            DemoDriver::BridgeGravity {
                apply_after_tick,
                load_scale,
                load_step,
                fractured,
            } => {
                if !*fractured && fracture_world.tick() >= *apply_after_tick {
                    match apply_bridge_gravity_load(fracture_world, *load_scale) {
                        Ok(step) => {
                            rebuild_graphics |= !step.report.split_events.is_empty();
                            *fractured |= !step.report.split_events.is_empty();
                            *load_scale += *load_step;
                        }
                        Err(err) => eprintln!("bridge fracture load failed: {err}"),
                    }
                }
            }
            DemoDriver::VoxelCollisionFractureField {
                apply_after_tick,
                applied,
            } => {
                if !*applied && fracture_world.tick() >= *apply_after_tick {
                    queue_voxel_collision_fracture_field(fracture_world);
                    *applied = true;
                }
            }
        }

        mirror_fracture_world(&fracture_world, physics);

        if rebuild_graphics {
            if let Some(graphics) = graphics {
                rebuild_render_nodes(graphics, physics, &fracture_world, destructible_families);
            }
        }
    });

    testbed.set_number_of_steps_per_frame(1);
    testbed.look_at(camera_center, CAMERA_PIXELS_PER_WORLD_UNIT);
}

fn encode_demo_snapshot(
    world: &FxRapierWorld2D,
    driver: DemoDriver,
) -> Result<Vec<u8>, fracture_rapier::FxRapierError> {
    let world_snapshot = world.snapshot()?;
    let mut bytes =
        Vec::with_capacity(FRACTURE_DEMO_SNAPSHOT_MAGIC.len() + world_snapshot.len() + 32);
    bytes.extend_from_slice(FRACTURE_DEMO_SNAPSHOT_MAGIC);
    write_u64(&mut bytes, world_snapshot.len() as u64);
    bytes.extend_from_slice(&world_snapshot);
    write_demo_driver(&mut bytes, driver);
    Ok(bytes)
}

fn decode_demo_snapshot(bytes: &[u8]) -> Result<(FxRapierWorld2D, DemoDriver), String> {
    let mut reader = DemoSnapshotReader::new(bytes);
    reader.magic()?;
    let world_len = usize::try_from(reader.u64("world length")?)
        .map_err(|_| "fracture demo world snapshot length is too large".to_string())?;
    let world_bytes = reader.bytes(world_len, "world snapshot")?;
    let world = FxRapierWorld2D::restore_snapshot(world_bytes)
        .map_err(|err| format!("fracture world restore failed: {err}"))?;
    let driver = read_demo_driver(&mut reader)?;
    reader.finish()?;
    Ok((world, driver))
}

fn write_demo_driver(bytes: &mut Vec<u8>, driver: DemoDriver) {
    match driver {
        DemoDriver::PhysicsOnly => bytes.push(0),
        DemoDriver::BridgeGravity {
            apply_after_tick,
            load_scale,
            load_step,
            fractured,
        } => {
            bytes.push(1);
            write_u64(bytes, apply_after_tick);
            write_f32(bytes, load_scale);
            write_f32(bytes, load_step);
            bytes.push(u8::from(fractured));
        }
        DemoDriver::VoxelCollisionFractureField {
            apply_after_tick,
            applied,
        } => {
            bytes.push(2);
            write_u64(bytes, apply_after_tick);
            bytes.push(u8::from(applied));
        }
    }
}

fn read_demo_driver(reader: &mut DemoSnapshotReader<'_>) -> Result<DemoDriver, String> {
    match reader.u8("driver tag")? {
        0 => Ok(DemoDriver::PhysicsOnly),
        1 => Ok(DemoDriver::BridgeGravity {
            apply_after_tick: reader.u64("bridge apply tick")?,
            load_scale: reader.f32("bridge load scale")?,
            load_step: reader.f32("bridge load step")?,
            fractured: reader.bool("bridge fractured")?,
        }),
        2 => Ok(DemoDriver::VoxelCollisionFractureField {
            apply_after_tick: reader.u64("voxel field apply tick")?,
            applied: reader.bool("voxel field applied")?,
        }),
        tag => Err(format!("unsupported fracture demo driver tag {tag}")),
    }
}

fn write_u64(bytes: &mut Vec<u8>, value: u64) {
    bytes.extend_from_slice(&value.to_le_bytes());
}

fn write_f32(bytes: &mut Vec<u8>, value: f32) {
    bytes.extend_from_slice(&value.to_le_bytes());
}

struct DemoSnapshotReader<'a> {
    bytes: &'a [u8],
    offset: usize,
}

impl<'a> DemoSnapshotReader<'a> {
    fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, offset: 0 }
    }

    fn magic(&mut self) -> Result<(), String> {
        let magic = self.bytes(FRACTURE_DEMO_SNAPSHOT_MAGIC.len(), "magic")?;
        if magic == FRACTURE_DEMO_SNAPSHOT_MAGIC {
            Ok(())
        } else {
            Err("invalid fracture demo snapshot magic".to_string())
        }
    }

    fn finish(&self) -> Result<(), String> {
        if self.offset == self.bytes.len() {
            Ok(())
        } else {
            Err("fracture demo snapshot has trailing bytes".to_string())
        }
    }

    fn bool(&mut self, field: &'static str) -> Result<bool, String> {
        match self.u8(field)? {
            0 => Ok(false),
            1 => Ok(true),
            _ => Err(format!("invalid boolean in {field}")),
        }
    }

    fn u8(&mut self, field: &'static str) -> Result<u8, String> {
        Ok(self.bytes(1, field)?[0])
    }

    fn u64(&mut self, field: &'static str) -> Result<u64, String> {
        let bytes = self.bytes(8, field)?;
        Ok(u64::from_le_bytes(bytes.try_into().expect("u64 length")))
    }

    fn f32(&mut self, field: &'static str) -> Result<f32, String> {
        let bytes = self.bytes(4, field)?;
        let value = f32::from_le_bytes(bytes.try_into().expect("f32 length"));
        if value.is_finite() {
            Ok(value)
        } else {
            Err(format!("non-finite value in {field}"))
        }
    }

    fn bytes(&mut self, len: usize, field: &'static str) -> Result<&'a [u8], String> {
        let end = self
            .offset
            .checked_add(len)
            .ok_or_else(|| format!("overflow while reading {field}"))?;
        let bytes = self
            .bytes
            .get(self.offset..end)
            .ok_or_else(|| format!("fracture demo snapshot ended while reading {field}"))?;
        self.offset = end;
        Ok(bytes)
    }
}

fn build_high_speed_wall_world() -> FxRapierWorld2D {
    let mut world = base_world(StressSettings {
        damage_per_overload: 2.0,
        max_fractures_per_frame: 24,
        enable_gravity: false,
        ..StressSettings::default()
    });
    configure_high_energy_impact_solver(&mut world);

    world
        .add_destructible(IMPACT_FAMILY, wall_asset())
        .expect("fracture wall asset should be valid");
    add_static_wall_anchors(&mut world);
    place_actor_body(&mut world, IMPACT_FAMILY, FxActorId(0), WALL_CENTER);

    add_projectile(&mut world);
    add_fixed_box(
        &mut world,
        Vector::new(0.0, -WALL_WORLD_HEIGHT * 0.5 - 0.18),
        Vector::new(1.1, 0.18),
        0.8,
    );
    add_floor(
        &mut world,
        Vector::new(0.0, -WALL_WORLD_HEIGHT * 0.65),
        20.0,
    );
    let rear_stop =
        world.insert_rigid_body(RigidBodyBuilder::fixed().translation(Vector::new(7.0, 0.0)));
    world.insert_collider_with_parent(ColliderBuilder::cuboid(0.12, 5.5).build(), rear_stop);

    world
}

fn build_bridge_collapse_world() -> FxRapierWorld2D {
    let mut world = base_world(bridge_stress_settings());

    world
        .add_destructible(BRIDGE_FAMILY, bridge_asset())
        .expect("fracture bridge asset should be valid");
    add_bridge_static_anchors(&mut world);
    place_actor_body(
        &mut world,
        BRIDGE_FAMILY,
        FxActorId(0),
        Vector::new(-4.0, 5.0),
    );
    add_fixed_box(
        &mut world,
        Vector::new(-4.0 - BRIDGE_WORLD_WIDTH * 0.5 - 0.25, 4.6),
        Vector::new(0.25, 1.4),
        0.8,
    );
    add_floor(&mut world, Vector::new(0.0, -7.0), 22.0);

    world
}

fn build_joint_pull_world() -> FxRapierWorld2D {
    let mut world = base_world(StressSettings {
        damage_per_overload: 2.0,
        max_fractures_per_frame: 16,
        ..StressSettings::default()
    });

    world
        .add_destructible(JOINT_FAMILY, joint_pull_asset())
        .expect("fracture joint asset should be valid");
    place_actor_body(
        &mut world,
        JOINT_FAMILY,
        FxActorId(0),
        Vector::new(-1.4, 4.2),
    );
    let destructible = world
        .actor_handles(JOINT_FAMILY, FxActorId(0))
        .expect("joint demo actor should have physics handles");
    let anchor_translation = Vector::new(0.9, 4.75);
    let anchor = add_fixed_box(&mut world, anchor_translation, Vector::new(0.45, 0.28), 0.5);
    let body_translation = world.rigid_bodies()[destructible.body].translation();
    let local_anchor = anchor_translation - body_translation;
    world.insert_impulse_joint(
        destructible.body,
        anchor,
        FixedJointBuilder::new()
            .local_anchor1(local_anchor)
            .local_anchor2(Vector::ZERO),
        true,
    );
    add_floor(&mut world, Vector::new(0.0, -4.0), 16.0);

    world
}

fn build_voxel_collision_world() -> FxRapierWorld2D {
    let mut world = base_world(StressSettings {
        damage_per_overload: 2.0,
        max_fractures_per_frame: 64,
        ..StressSettings::default()
    });

    world
        .add_destructible(COLLISION_LEFT_FAMILY, collision_asset(8))
        .expect("left voxel collision asset should be valid");
    world
        .add_destructible(COLLISION_RIGHT_FAMILY, collision_asset(9))
        .expect("right voxel collision asset should be valid");

    place_actor_body_with_motion(
        &mut world,
        COLLISION_LEFT_FAMILY,
        FxActorId(0),
        Vector::new(-6.0, 2.0),
        Vector::new(12.0, 0.8),
        -0.3,
    );
    place_actor_body_with_motion(
        &mut world,
        COLLISION_RIGHT_FAMILY,
        FxActorId(0),
        Vector::new(6.0, 2.0),
        Vector::new(-12.0, 0.8),
        0.3,
    );
    add_floor(&mut world, Vector::new(0.0, -5.0), 18.0);

    world
}

fn queue_voxel_collision_fracture_field(world: &mut FxRapierWorld2D) {
    let Some(handles) = world.actor_handles(COLLISION_LEFT_FAMILY, FxActorId(0)) else {
        return;
    };
    let Some(body) = world.rigid_bodies().get(handles.body) else {
        return;
    };
    let center = body.translation();
    world.queue_fracture_field(
        FractureField2D::direct_damage(fracture_core::Vec2::new(center.x, center.y), 2.4, 2.0)
            .with_family(COLLISION_LEFT_FAMILY),
    );
}

fn base_world(settings: StressSettings) -> FxRapierWorld2D {
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(GRAVITY);
    world.integration_parameters_mut().dt = 1.0 / 60.0;
    world.set_stress_settings(settings);
    world
}

fn configure_high_energy_impact_solver(world: &mut FxRapierWorld2D) {
    let params = world.integration_parameters_mut();
    params.num_solver_iterations = 14;
    world.set_quick_impact_settings(QuickImpactSettings {
        enabled: true,
        static_soften_impulse_threshold: 0.8,
        static_suppress_impulse_threshold: 2.5,
        dynamic_soften_impulse_threshold: 8.0,
        dynamic_suppress_impulse_threshold: 24.0,
        penetration_impulse_scale: 0.0,
        stress_force_scale: 0.35,
        softened_friction_scale: 0.35,
        softened_restitution_scale: 0.0,
        suppress_tunnel_window_frames: 3,
        ..QuickImpactSettings::default()
    });
    world.set_material_impact_hardness(1, 1.0);
}

fn add_static_wall_anchors(world: &mut FxRapierWorld2D) {
    let anchored_nodes = support_nodes_touching_y(world, IMPACT_FAMILY, 0);
    for (index, node) in anchored_nodes.into_iter().enumerate() {
        world
            .connect_static_anchor(
                IMPACT_FAMILY,
                StaticAnchorConnectionDesc::new(StaticAnchorDesc {
                    id: ExternalBondId(index as u32),
                    node,
                    target: ExternalTarget2D {
                        kind: ExternalTargetKind::World,
                        token: ExternalTargetToken(0),
                    },
                    anchor: fracture_core::Vec2::new(0.0, -WALL_WORLD_HEIGHT * 0.5),
                    normal: fracture_core::Vec2::new(0.0, -1.0),
                    health: 1.20,
                    effective_length: VOXEL_SIZE,
                    tension_limit: 0.040,
                    compression_limit: 0.040,
                    shear_limit: 0.040,
                })
                .with_body_policy(StaticAnchorBodyPolicy::Fixed),
            )
            .expect("fracture wall static anchor should connect");
    }
}

fn add_bridge_static_anchors(world: &mut FxRapierWorld2D) {
    let anchored_nodes = support_nodes_touching_x(world, BRIDGE_FAMILY, 0);
    for (index, node) in anchored_nodes.into_iter().enumerate() {
        world
            .connect_static_anchor(
                BRIDGE_FAMILY,
                StaticAnchorConnectionDesc::new(StaticAnchorDesc {
                    id: ExternalBondId(index as u32),
                    node,
                    target: ExternalTarget2D {
                        kind: ExternalTargetKind::World,
                        token: ExternalTargetToken(1),
                    },
                    anchor: fracture_core::Vec2::new(-BRIDGE_WORLD_WIDTH * 0.5, 0.0),
                    normal: fracture_core::Vec2::new(-1.0, 0.0),
                    health: 100.0,
                    effective_length: VOXEL_SIZE,
                    tension_limit: 100.0,
                    compression_limit: 100.0,
                    shear_limit: 100.0,
                })
                .with_body_policy(StaticAnchorBodyPolicy::Fixed),
            )
            .expect("fracture bridge static anchor should connect");
    }
}

fn support_nodes_touching_y(
    world: &FxRapierWorld2D,
    family: FxFamilyId,
    y: u32,
) -> Vec<SupportNodeId> {
    let Some(family) = world.family(family) else {
        return Vec::new();
    };
    family
        .asset()
        .support_nodes()
        .iter()
        .filter(|node| node.voxels.iter().any(|voxel| voxel.y == y))
        .map(|node| node.id)
        .collect()
}

fn support_nodes_touching_x(
    world: &FxRapierWorld2D,
    family: FxFamilyId,
    x: u32,
) -> Vec<SupportNodeId> {
    let Some(family) = world.family(family) else {
        return Vec::new();
    };
    family
        .asset()
        .support_nodes()
        .iter()
        .filter(|node| node.voxels.iter().any(|voxel| voxel.x == x))
        .map(|node| node.id)
        .collect()
}

fn wall_asset() -> AuthoredVoxelAsset {
    debug_assert!(
        (VOXEL_SIZE - VIEWPORT_REFERENCE_WORLD_HEIGHT / VIEWPORT_REFERENCE_HEIGHT as f32).abs()
            < f32::EPSILON
    );

    let mut input = rect_voxel_input(WALL_VOXEL_WIDTH, WALL_VOXEL_HEIGHT, 7);
    input.default_bond_health = 1.60;
    input.default_tension_limit = 0.045;
    input.default_shear_limit = 0.045;

    author_demo_asset(input, natural_wall_cluster_policy(), "fracture wall")
}

fn bridge_asset() -> AuthoredVoxelAsset {
    let mut input = rect_voxel_input(BRIDGE_VOXEL_WIDTH, BRIDGE_VOXEL_HEIGHT, 6);
    input.default_bond_health = 8.0;
    input.default_tension_limit = 0.40;
    input.default_shear_limit = 0.40;
    author_demo_asset(
        input,
        VoxelClusterPolicy::structural_beam(VoxelClusterAxis::X, 16, 2),
        "fracture bridge",
    )
}

fn joint_pull_asset() -> AuthoredVoxelAsset {
    let mut input = rect_voxel_input(JOINT_VOXEL_WIDTH, JOINT_VOXEL_HEIGHT, 7);
    input.default_bond_health = 0.16;
    input.default_tension_limit = 0.004;
    input.default_shear_limit = 0.004;
    author_demo_asset(
        input,
        VoxelClusterPolicy::structural_beam(VoxelClusterAxis::X, 8, 2),
        "fracture joint pull shape",
    )
}

fn collision_asset(contact_material: u16) -> AuthoredVoxelAsset {
    let mut input = rect_voxel_input(
        COLLIDER_VOXEL_WIDTH,
        COLLIDER_VOXEL_HEIGHT,
        contact_material,
    );
    input.default_bond_health = 0.12;
    input.default_tension_limit = 0.003;
    input.default_shear_limit = 0.003;
    author_demo_asset(
        input,
        VoxelClusterPolicy::brittle_isotropic(6, 36),
        "fracture collision shape",
    )
}

fn rect_voxel_input(width: u32, height: u32, contact_material: u16) -> VoxelAuthoringInput {
    let cell_count = (width * height) as usize;
    let occupancy = vec![true; cell_count];
    let fracture_material = vec![1; cell_count];
    let contact_material = vec![contact_material; cell_count];
    let external_id = (0..cell_count as u32).collect::<Vec<_>>();

    VoxelAuthoringInput::new(
        width,
        height,
        VOXEL_SIZE,
        occupancy,
        fracture_material,
        contact_material,
        external_id,
    )
}

fn author_demo_asset(
    input: VoxelAuthoringInput,
    cluster_policy: VoxelClusterPolicy,
    name: &'static str,
) -> AuthoredVoxelAsset {
    let options = VoxelAuthoringOptions {
        hierarchy_policy: VoxelHierarchyPolicy::ParentChunksByMaterial,
        ..VoxelAuthoringOptions::default()
    }
    .with_material_rule(1, cluster_policy);

    author_voxel_asset_with_options(input, options)
        .unwrap_or_else(|_| panic!("{name} should author"))
}

fn natural_wall_cluster_policy() -> VoxelClusterPolicy {
    VoxelClusterPolicy {
        mode: VoxelClusterMode::NaturalVoronoi(
            NaturalVoronoi::generated(56, 0xC0A5_2026)
                .with_noise(0xF2AC_7A11, 850)
                .with_field(
                    NaturalVoronoiClusterField::new(GridCoord::new(2, WALL_VOXEL_HEIGHT / 2), 42)
                        .with_extra_seeds(34, 0x1A9E_1197)
                        .with_distance_bias(640, 160),
                )
                .with_field(
                    NaturalVoronoiClusterField::new(
                        GridCoord::new(WALL_VOXEL_WIDTH - 1, WALL_VOXEL_HEIGHT / 2),
                        58,
                    )
                    .with_extra_seeds(16, 0xB01D_5105)
                    .with_distance_bias(768, 96),
                ),
        ),
    }
}

fn add_projectile(world: &mut FxRapierWorld2D) {
    let body = world.insert_rigid_body(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(-16.0, 0.0))
            .linvel(Vector::new(52.0, 10.0))
            .ccd_enabled(true)
            .additional_mass(700.0),
    );
    world.insert_collider_with_parent(
        ColliderBuilder::ball(0.45)
            .density(50.0)
            .friction(0.4)
            .restitution(0.05)
            .build(),
        body,
    );
}

fn add_floor(world: &mut FxRapierWorld2D, translation: Vector, half_width: f32) {
    let floor = world.insert_rigid_body(RigidBodyBuilder::fixed().translation(translation));
    world.insert_collider_with_parent(ColliderBuilder::cuboid(half_width, 0.12).build(), floor);
}

fn add_fixed_box(
    world: &mut FxRapierWorld2D,
    translation: Vector,
    half_extents: Vector,
    friction: f32,
) -> RigidBodyHandle {
    let body = world.insert_rigid_body(RigidBodyBuilder::fixed().translation(translation));
    world.insert_collider_with_parent(
        ColliderBuilder::cuboid(half_extents.x, half_extents.y)
            .friction(friction)
            .restitution(0.0)
            .build(),
        body,
    );
    body
}

fn place_actor_body(
    world: &mut FxRapierWorld2D,
    family: FxFamilyId,
    actor: FxActorId,
    translation: Vector,
) {
    place_actor_body_with_motion(world, family, actor, translation, Vector::ZERO, 0.0);
}

fn place_actor_body_with_motion(
    world: &mut FxRapierWorld2D,
    family: FxFamilyId,
    actor: FxActorId,
    translation: Vector,
    linvel: Vector,
    angvel: f32,
) {
    let handles = world
        .actor_handles(family, actor)
        .expect("fracture actor should have physics handles");
    let body = world
        .rigid_bodies_mut()
        .get_mut(handles.body)
        .expect("fracture actor body should exist");
    body.set_position(Pose::from_translation(translation), true);
    body.set_linvel(linvel, true);
    body.set_angvel(angvel, true);
    body.enable_ccd(true);
}

fn apply_bridge_gravity_load(
    world: &mut FxRapierWorld2D,
    load_scale: f32,
) -> Result<fracture_rapier::FxStepWithDiagnostics, fracture_rapier::FxRapierError> {
    let Some(family) = world.family(BRIDGE_FAMILY) else {
        return world.apply_fracture_commands_to_family(BRIDGE_FAMILY, &[]);
    };

    let stress_inputs = family
        .asset()
        .support_nodes()
        .iter()
        .filter(|node| !node.voxels.iter().any(|voxel| voxel.x == 0))
        .enumerate()
        .map(|(index, node)| {
            let center_x = node
                .voxels
                .iter()
                .map(|voxel| voxel.x as f32 + 0.5)
                .sum::<f32>()
                / node.voxels.len().max(1) as f32;
            let lever = (center_x / BRIDGE_VOXEL_WIDTH as f32).max(0.15);
            StressInput {
                order_key: DeterministicOrderKey::new(
                    world.tick(),
                    30,
                    BRIDGE_FAMILY,
                    FxActorId(0),
                    CommandId(index as u32),
                ),
                actor: FxActorId(0),
                node: node.id,
                force: fracture_core::Vec2::new(0.0, GRAVITY.y * lever * load_scale.max(0.0)),
                source: DamageSource::Stress,
            }
        })
        .collect::<Vec<_>>();

    let solver = StressSolver2D::new(bridge_stress_settings());
    let commands = solver.generate(family, &stress_inputs);
    world.apply_fracture_commands_to_family(BRIDGE_FAMILY, &commands)
}

fn bridge_stress_settings() -> StressSettings {
    StressSettings {
        damage_per_overload: 0.45,
        fracture_energy_budget: 2.5,
        beam_bending_moment_scale: 0.08,
        section_aggregation_max_bonds: 8,
        section_axis_dot_min: 0.92,
        max_fractures_per_frame: 8,
        ..StressSettings::default()
    }
}

fn mirror_fracture_world(world: &FxRapierWorld2D, physics: &mut PhysicsState) {
    physics.bodies = world.rigid_bodies().clone();
    physics.colliders = world.colliders().clone();
    physics.impulse_joints = world.impulse_joints().clone();
    physics.multibody_joints = world.multibody_joints().clone();
    physics.islands = world.islands().clone();
    physics.broad_phase = world.broad_phase().clone();
    physics.narrow_phase = world.narrow_phase().clone();
    physics.ccd_solver = world.ccd_solver().clone();
    physics.pipeline = PhysicsPipeline::new();
    physics.gravity = world.gravity();
}

fn color_initial_bodies(
    testbed: &mut Testbed,
    world: &FxRapierWorld2D,
    destructible_families: &[FxFamilyId],
) {
    let destructible = destructible_body_handles(world, destructible_families);
    for (handle, body) in world.rigid_bodies().iter() {
        let color = body_color(body, destructible.contains(&handle));
        testbed.set_initial_body_color(handle, color);
    }
}

fn rebuild_render_nodes(
    graphics: &mut TestbedGraphics,
    physics: &PhysicsState,
    world: &FxRapierWorld2D,
    destructible_families: &[FxFamilyId],
) {
    let destructible = destructible_body_handles(world, destructible_families);
    graphics.graphics.clear();
    for (handle, body) in physics.bodies.iter() {
        graphics.graphics.add_body_colliders_with_color(
            graphics.window,
            handle,
            &physics.bodies,
            &physics.colliders,
            body_color(body, destructible.contains(&handle)),
        );
    }
}

fn destructible_body_handles(
    world: &FxRapierWorld2D,
    destructible_families: &[FxFamilyId],
) -> Vec<RigidBodyHandle> {
    let mut out = Vec::new();
    for family_id in destructible_families {
        let Some(family) = world.family(*family_id) else {
            continue;
        };
        for (actor, _) in family.actors() {
            if let Some(handles) = world.actor_handles(*family_id, *actor) {
                out.push(handles.body);
            }
        }
    }
    out
}

fn body_color(body: &RigidBody, is_destructible: bool) -> Color {
    if is_destructible {
        Color::new(0.86, 0.36, 0.22, 1.0)
    } else if !body.is_dynamic() {
        Color::new(0.58, 0.54, 0.50, 1.0)
    } else {
        Color::new(0.18, 0.27, 0.34, 1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeSet;

    #[test]
    fn high_speed_wall_contact_impulse_splits_anchor_fixed_wall() {
        let mut world = build_high_speed_wall_world();
        let handles = world
            .actor_handles(IMPACT_FAMILY, FxActorId(0))
            .expect("initial wall actor should have physics handles");
        assert_eq!(
            world.rigid_bodies()[handles.body].body_type(),
            RigidBodyType::Fixed
        );
        assert_wall_anchors_are_on_bottom_edge(&world);

        let mut split_events = 0;
        let mut stress_inputs = 0;
        let mut fracture_events = 0;

        for _ in 0..120 {
            let step = world.step().expect("wall demo fracture step should run");
            stress_inputs += step.stress_inputs.len();
            fracture_events += step.fracture_events.len();
            split_events += step.split_events.len();
            if split_events > 0 {
                break;
            }
        }

        assert!(stress_inputs > 0);
        assert!(fracture_events > 0);
        assert!(split_events > 0);
        assert!(world.family(IMPACT_FAMILY).unwrap().actor_count() > 1);
    }

    #[test]
    fn bridge_gravity_stress_splits_static_anchor_cantilever() {
        let mut world = build_bridge_collapse_world();
        let handles = world
            .actor_handles(BRIDGE_FAMILY, FxActorId(0))
            .expect("initial bridge actor should have physics handles");
        assert_eq!(
            world.rigid_bodies()[handles.body].body_type(),
            RigidBodyType::Fixed
        );

        let mut split_events = 0;
        let mut fracture_events = 0;
        let mut first_split_fractures = 0;
        let mut first_split_fragments = 0;
        let mut first_structural_break_max_root_distance = None;
        for i in 0..120 {
            let step = apply_bridge_gravity_load(&mut world, 0.25 + i as f32 * 0.20)
                .expect("bridge gravity fracture should apply");
            fracture_events += step.report.fracture_events.len();
            split_events += step.report.split_events.len();
            if split_events > 0 {
                let family = world
                    .family(BRIDGE_FAMILY)
                    .expect("bridge family should exist after split");
                first_split_fractures = step.report.fracture_events.len();
                first_split_fragments = step.report.split_events[0].fragments.len();
                first_structural_break_max_root_distance = Some(
                    step.report
                        .fracture_events
                        .iter()
                        .filter_map(|event| match event.target {
                            FractureTarget::Bond(bond_id) => {
                                let bond = family.asset().bond(bond_id)?;
                                let node_a_min_x = family
                                    .asset()
                                    .node(bond.node_a)?
                                    .voxels
                                    .iter()
                                    .map(|voxel| voxel.x)
                                    .min()?;
                                let node_b_min_x = family
                                    .asset()
                                    .node(bond.node_b)?
                                    .voxels
                                    .iter()
                                    .map(|voxel| voxel.x)
                                    .min()?;
                                Some(node_a_min_x.min(node_b_min_x))
                            }
                            _ => None,
                        })
                        .max()
                        .expect("first bridge split should include structural bond fractures"),
                );
                break;
            }
        }

        assert!(fracture_events > 0);
        assert!(split_events > 0);
        assert!(first_split_fractures <= 8);
        assert!(first_split_fragments <= 3);
        assert!(
            first_structural_break_max_root_distance.unwrap() <= 24,
            "first structural bridge split should occur near the anchored root section"
        );
        assert!(world.family(BRIDGE_FAMILY).unwrap().actor_count() > 1);
    }

    #[test]
    fn joint_pull_feedback_splits_voxel_shape() {
        let mut world = build_joint_pull_world();
        let mut split_events = 0;
        let mut joint_inputs = 0;

        for _ in 0..24 {
            let step = world.step().expect("joint demo fracture step should run");
            joint_inputs += step
                .stress_inputs
                .iter()
                .filter(|input| input.source == DamageSource::JointFeedback)
                .count();
            split_events += step.split_events.len();
            if split_events > 0 {
                break;
            }
        }

        assert!(joint_inputs > 0);
        assert!(split_events > 0);
        assert!(world.family(JOINT_FAMILY).unwrap().actor_count() > 1);
        assert_external_joint_attached_to_anchor_node(&world);

        for _ in 0..120 {
            world
                .step()
                .expect("joint demo should keep stepping after fracture");
        }
        assert!(
            !world.impulse_joints().is_empty(),
            "joint pull demo should not release the joint to hide post-fracture issues"
        );
    }

    #[test]
    fn joint_pull_survives_testbed_mirror_step_order() {
        let mut fracture_world = build_joint_pull_world();
        let mut physics = PhysicsState::new();
        mirror_fracture_world(&fracture_world, &mut physics);

        let mut saw_split = false;
        for _ in 0..180 {
            physics.pipeline.step(
                physics.gravity,
                &physics.integration_parameters,
                &mut physics.islands,
                &mut physics.broad_phase,
                &mut physics.narrow_phase,
                &mut physics.bodies,
                &mut physics.colliders,
                &mut physics.impulse_joints,
                &mut physics.multibody_joints,
                &mut physics.ccd_solver,
                &*physics.hooks,
                &(),
            );

            let step = fracture_world
                .step()
                .expect("joint demo fracture world should step after testbed physics");
            saw_split |= !step.split_events.is_empty();
            mirror_fracture_world(&fracture_world, &mut physics);
        }

        assert!(saw_split);
        assert_external_joint_attached_to_anchor_node(&fracture_world);
    }

    #[test]
    fn voxel_vs_voxel_impact_splits_both_shapes() {
        let mut world = build_voxel_collision_world();
        let mut split_families = BTreeSet::new();
        let mut contact_families = BTreeSet::new();

        for _ in 0..80 {
            let step = world.step().expect("voxel collision demo should step");
            for input in &step.contact_impulses {
                if input.impulse.normal_impulse > 0.0 {
                    contact_families.insert(input.stress.order_key.family_id);
                }
            }
            for event in &step.split_events {
                split_families.insert(event.family);
            }
            if split_families.contains(&COLLISION_LEFT_FAMILY)
                && split_families.contains(&COLLISION_RIGHT_FAMILY)
            {
                break;
            }
        }

        assert!(contact_families.contains(&COLLISION_LEFT_FAMILY));
        assert!(contact_families.contains(&COLLISION_RIGHT_FAMILY));
        assert!(split_families.contains(&COLLISION_LEFT_FAMILY));
        assert!(split_families.contains(&COLLISION_RIGHT_FAMILY));
        assert!(world.family(COLLISION_LEFT_FAMILY).unwrap().actor_count() > 1);
        assert!(world.family(COLLISION_RIGHT_FAMILY).unwrap().actor_count() > 1);
    }

    #[test]
    fn fracture2_voxel_collision_demo_queues_direct_damage_field() {
        let mut world = build_voxel_collision_world();

        queue_voxel_collision_fracture_field(&mut world);
        let step = world
            .step()
            .expect("voxel collision queued fracture field should step");

        assert!(
            step.fracture_field_effects
                .iter()
                .any(|effect| effect.family == COLLISION_LEFT_FAMILY)
        );
        assert!(!step.fracture_events.is_empty());
        assert!(!step.split_events.is_empty());
        assert!(world.family(COLLISION_LEFT_FAMILY).unwrap().actor_count() > 1);
    }

    #[test]
    fn fracture2_demo_snapshot_restores_fracture_runtime_state() {
        let mut world = build_high_speed_wall_world();
        for _ in 0..8 {
            world
                .step()
                .expect("fracture demo should step before snapshot");
        }
        let saved_tick = world.tick();
        let snapshot = encode_demo_snapshot(&world, DemoDriver::PhysicsOnly)
            .expect("fracture demo snapshot should encode");

        for _ in 0..120 {
            let step = world
                .step()
                .expect("fracture demo should keep stepping after snapshot");
            if !step.split_events.is_empty() {
                break;
            }
        }
        assert!(world.family(IMPACT_FAMILY).unwrap().actor_count() > 1);

        let (mut restored, driver) =
            decode_demo_snapshot(&snapshot).expect("fracture demo snapshot should restore");
        assert_eq!(driver, DemoDriver::PhysicsOnly);
        assert_eq!(restored.tick(), saved_tick);
        assert_eq!(restored.family(IMPACT_FAMILY).unwrap().actor_count(), 1);

        let mut restored_split = false;
        for _ in 0..120 {
            let step = restored
                .step()
                .expect("restored fracture demo should keep stepping");
            restored_split |= !step.split_events.is_empty();
            if restored_split {
                break;
            }
        }
        assert!(restored_split);
    }

    #[test]
    fn fracture2_demo_snapshot_preserves_script_driver_state() {
        let world = build_bridge_collapse_world();
        let driver = DemoDriver::BridgeGravity {
            apply_after_tick: 45,
            load_scale: 0.65,
            load_step: 0.20,
            fractured: true,
        };
        let snapshot =
            encode_demo_snapshot(&world, driver).expect("fracture demo snapshot should encode");
        let (_, restored_driver) =
            decode_demo_snapshot(&snapshot).expect("fracture demo snapshot should restore");

        assert_eq!(restored_driver, driver);
    }

    fn assert_wall_anchors_are_on_bottom_edge(world: &FxRapierWorld2D) {
        let family = world
            .family(IMPACT_FAMILY)
            .expect("wall family should exist");
        let anchors = family.external_bonds().collect::<Vec<_>>();
        assert!(!anchors.is_empty());
        for (_, bond) in anchors {
            let node = family
                .asset()
                .node(bond.node)
                .expect("anchor node should exist");
            assert!(
                node.voxels.iter().any(|voxel| voxel.y == 0),
                "wall anchor {:?} should touch bottom edge, got voxels {:?}",
                bond.node,
                node.voxels
            );
        }
    }

    fn assert_external_joint_attached_to_anchor_node(world: &FxRapierWorld2D) {
        assert!(
            !world.impulse_joints().is_empty(),
            "joint pull demo must keep the external joint after fracture"
        );

        let anchor_node = joint_anchor_node(world);
        let family = world.family(JOINT_FAMILY).unwrap();
        let owner = family
            .node_owner(anchor_node)
            .expect("joint anchor node should still be owned after split");
        let handles = world
            .actor_handles(JOINT_FAMILY, owner)
            .expect("joint anchor fragment should have physics handles");
        let joint_migrated = world
            .impulse_joints()
            .iter()
            .any(|(_, joint)| joint.body1 == handles.body || joint.body2 == handles.body);

        assert!(
            joint_migrated,
            "joint pull demo must migrate the external joint to the fragment owning the joint anchor support node"
        );
    }

    fn joint_anchor_node(world: &FxRapierWorld2D) -> SupportNodeId {
        let family = world.family(JOINT_FAMILY).unwrap();
        let asset = family.asset();
        let full_shape_local_com = fracture_core::Vec2::new(
            JOINT_VOXEL_WIDTH as f32 * VOXEL_SIZE * 0.5,
            JOINT_VOXEL_HEIGHT as f32 * VOXEL_SIZE * 0.5,
        );
        let local_anchor = Vector::new(0.9, 4.75) - Vector::new(-1.4, 4.2);
        let anchor_in_asset = fracture_core::Vec2::new(
            full_shape_local_com.x + local_anchor.x,
            full_shape_local_com.y + local_anchor.y,
        );

        asset
            .support_nodes()
            .iter()
            .flat_map(|node| {
                node.voxels.iter().map(move |voxel| {
                    let center = voxel.center(asset.voxel_size());
                    let delta = center - anchor_in_asset;
                    (node.id, delta.dot(delta))
                })
            })
            .min_by(|(_, a), (_, b)| a.total_cmp(b))
            .map(|(node, _)| node)
            .expect("joint pull asset should have support nodes")
    }
}
