use std::collections::BTreeSet;

use fracture_core::{
    BondId, CommandId, CompressionDamageMode2D, ConnectionError, ConnectionId, DamageSource,
    DeterministicOrderKey, DynamicConnectionPolicy, DynamicStructuralBondDesc, ExternalBondId,
    ExternalTarget2D, ExternalTargetKind, ExternalTargetToken, FractureCommand, FractureTarget,
    FxActorId, FxFamilyId, StaticAnchorDesc, StressInput, StressSettings, SupportNodeId, Vec2,
    snapshot::SnapshotMode,
};
use fracture_voxel::{VoxelAuthoringInput, author_voxel_asset};
use rapier2d::prelude::*;

#[cfg(feature = "deterministic-replay")]
use crate::FxRapierReplayCommand;
use crate::contact_map::{ContactPairSide, collider_key, map_contact_pair};
use crate::snapshot::{
    PrestressBaselineTargetSnapshot, decode_world_snapshot, encode_world_snapshot,
};
use crate::{
    ActorPhysicsHandles, ColliderLodSettings, ContactMaterialProperties,
    DynamicStructuralConnectionDesc, FractureField2D, FractureFieldMode, FxRapierError,
    FxRapierSnapshotError, FxRapierWorld2D, QuickImpactAction, QuickImpactSettings,
    StaticAnchorBodyPolicy, StaticAnchorConnectionDesc,
};

#[cfg(feature = "deterministic-replay")]
fn set_deterministic_replay_mode(world: &mut FxRapierWorld2D) {
    world.set_snapshot_mode(SnapshotMode::Deterministic);
    world.set_lod_settings(ColliderLodSettings::disabled());
}

fn rewrite_snapshot_checksum(bytes: &mut [u8]) {
    let mut hash = 0xcbf2_9ce4_8422_2325u64;
    for byte in &bytes[34..] {
        hash ^= u64::from(*byte);
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    }
    bytes[26..34].copy_from_slice(&hash.to_le_bytes());
}

fn replace_first_f32_bits(bytes: &mut [u8], from: f32, to_bits: u32) {
    let from = from.to_bits().to_le_bytes();
    let to = to_bits.to_le_bytes();
    let offset = bytes
        .windows(from.len())
        .position(|window| window == from)
        .expect("expected f32 marker in snapshot bytes");
    bytes[offset..offset + from.len()].copy_from_slice(&to);
    rewrite_snapshot_checksum(bytes);
}

fn two_node_asset(contact_material: u16) -> fracture_voxel::AuthoredVoxelAsset {
    two_node_multi_material_asset(contact_material, contact_material)
}

fn two_node_multi_material_asset(
    left_material: u16,
    right_material: u16,
) -> fracture_voxel::AuthoredVoxelAsset {
    let mut input = VoxelAuthoringInput::new(
        2,
        1,
        1.0,
        vec![true, true],
        vec![1, 2],
        vec![left_material, right_material],
        vec![0, 1],
    );
    input.support_node_hint = Some(vec![Some(0), Some(1)]);
    input.default_bond_health = 1.0;
    input.default_tension_limit = 0.01;
    input.default_shear_limit = 0.01;
    author_voxel_asset(input).unwrap()
}

fn l_shaped_asset() -> fracture_voxel::AuthoredVoxelAsset {
    let mut input = VoxelAuthoringInput::new(
        2,
        2,
        1.0,
        vec![true, false, true, true],
        vec![1, 0, 1, 1],
        vec![7, 0, 7, 8],
        vec![0, 0, 1, 2],
    );
    input.support_node_hint = Some(vec![Some(0), None, Some(0), Some(1)]);
    author_voxel_asset(input).unwrap()
}

fn single_node_asset(contact_material: u16) -> fracture_voxel::AuthoredVoxelAsset {
    author_voxel_asset(VoxelAuthoringInput::new(
        1,
        1,
        1.0,
        vec![true],
        vec![1],
        vec![contact_material],
        vec![0],
    ))
    .unwrap()
}

fn two_voxel_single_node_asset(contact_material: u16) -> fracture_voxel::AuthoredVoxelAsset {
    let mut input = VoxelAuthoringInput::new(
        2,
        1,
        1.0,
        vec![true, true],
        vec![1, 1],
        vec![contact_material, contact_material],
        vec![0, 0],
    );
    input.support_node_hint = Some(vec![Some(0), Some(0)]);
    author_voxel_asset(input).unwrap()
}

fn disconnected_two_node_asset() -> fracture_voxel::AuthoredVoxelAsset {
    let mut input = VoxelAuthoringInput::new(
        3,
        1,
        1.0,
        vec![true, false, true],
        vec![1, 0, 1],
        vec![5, 0, 5],
        vec![0, 0, 1],
    );
    input.support_node_hint = Some(vec![Some(0), None, Some(1)]);
    author_voxel_asset(input).unwrap()
}

fn four_node_line_asset() -> fracture_voxel::AuthoredVoxelAsset {
    let mut input = VoxelAuthoringInput::new(
        4,
        1,
        1.0,
        vec![true, true, true, true],
        vec![1, 1, 1, 1],
        vec![5, 5, 5, 5],
        vec![0, 1, 2, 3],
    );
    input.support_node_hint = Some(vec![Some(0), Some(1), Some(2), Some(3)]);
    input.default_bond_health = 1.0;
    author_voxel_asset(input).unwrap()
}

fn static_anchor_desc(id: u32, node: u32) -> StaticAnchorDesc {
    StaticAnchorDesc {
        id: ExternalBondId(id),
        node: SupportNodeId(node),
        target: ExternalTarget2D {
            kind: ExternalTargetKind::World,
            token: ExternalTargetToken(0),
        },
        anchor: Vec2::new(node as f32 + 0.5, 0.5),
        normal: Vec2::new(1.0, 0.0),
        health: 1.0,
        effective_length: 1.0,
        tension_limit: 0.01,
        compression_limit: 0.01,
        shear_limit: 0.01,
    }
}

fn dynamic_graph_only_desc(id: u32, node_a: u32, node_b: u32) -> DynamicStructuralBondDesc {
    DynamicStructuralBondDesc {
        id: ConnectionId(id),
        node_a: SupportNodeId(node_a),
        node_b: SupportNodeId(node_b),
        centroid: Vec2::new((node_a + node_b) as f32 * 0.5, 0.5),
        normal: Vec2::new(1.0, 0.0),
        health: 1.0,
        effective_length: 1.0,
        tension_limit: 0.01,
        compression_limit: 0.01,
        shear_limit: 0.01,
    }
}

fn break_bond_command(
    tick: u64,
    family: FxFamilyId,
    actor: FxActorId,
    bond: BondId,
) -> FractureCommand {
    FractureCommand {
        order_key: DeterministicOrderKey::new(tick, 0, family, actor, CommandId(tick as u32)),
        actor,
        target: FractureTarget::Bond(bond),
        health_loss: 2.0,
        effective_length_loss: 2.0,
        source: DamageSource::Script,
    }
}

fn break_external_bond_command(
    tick: u64,
    family: FxFamilyId,
    actor: FxActorId,
    bond: ExternalBondId,
) -> FractureCommand {
    FractureCommand {
        order_key: DeterministicOrderKey::new(tick, 0, family, actor, CommandId(tick as u32)),
        actor,
        target: FractureTarget::ExternalBond(bond),
        health_loss: 2.0,
        effective_length_loss: 2.0,
        source: DamageSource::Script,
    }
}

fn add_fixed_box(
    world: &mut FxRapierWorld2D,
    translation: Vector,
    half_extents: Vector,
    friction: f32,
) -> (RigidBodyHandle, ColliderHandle) {
    let body = world.insert_rigid_body(RigidBodyBuilder::fixed().translation(translation));
    let collider = world.insert_collider_with_parent(
        ColliderBuilder::cuboid(half_extents.x, half_extents.y)
            .friction(friction)
            .build(),
        body,
    );
    (body, collider)
}

fn overlapping_side_contact_world() -> (FxRapierWorld2D, ColliderHandle, ColliderHandle) {
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(Vector::ZERO);
    world
        .add_destructible(FxFamilyId(1), single_node_asset(7))
        .unwrap();
    let destructible = world.actor_handles(FxFamilyId(1), FxActorId(0)).unwrap();
    let (_, ordinary) = add_fixed_box(
        &mut world,
        Vector::new(1.0, 0.5),
        Vector::new(0.5, 0.5),
        1.0,
    );
    (world, destructible.collider, ordinary)
}

fn quick_impact_settings(
    static_soften: f32,
    static_suppress: f32,
    dynamic_soften: f32,
    dynamic_suppress: f32,
) -> QuickImpactSettings {
    QuickImpactSettings {
        enabled: true,
        static_soften_impulse_threshold: static_soften,
        static_suppress_impulse_threshold: static_suppress,
        dynamic_soften_impulse_threshold: dynamic_soften,
        dynamic_suppress_impulse_threshold: dynamic_suppress,
        penetration_impulse_scale: 0.0,
        stress_force_scale: 1.0,
        softened_friction_scale: 0.2,
        softened_restitution_scale: 0.0,
        ..QuickImpactSettings::default()
    }
}

fn quick_impact_wall_world(
    settings: QuickImpactSettings,
    speed: f32,
) -> (FxRapierWorld2D, ActorPhysicsHandles) {
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(Vector::ZERO);
    world.integration_parameters_mut().dt = 1.0 / 60.0;
    world.set_stress_settings(StressSettings {
        damage_per_overload: 2.0,
        max_iterations: 2,
        ..StressSettings::default()
    });
    world.set_quick_impact_settings(settings);
    world.set_material_impact_hardness(7, 1.0);
    world
        .add_destructible(FxFamilyId(1), two_node_asset(7))
        .unwrap();
    let handles = world.actor_handles(FxFamilyId(1), FxActorId(0)).unwrap();
    world
        .rigid_bodies_mut()
        .get_mut(handles.body)
        .unwrap()
        .set_linvel(Vector::new(speed, 0.0), true);
    add_fixed_box(
        &mut world,
        Vector::new(2.0, 0.5),
        Vector::new(0.5, 0.5),
        1.0,
    );
    (world, handles)
}

fn quick_impact_dynamic_world(
    settings: QuickImpactSettings,
    speed: f32,
) -> (FxRapierWorld2D, ActorPhysicsHandles, ActorPhysicsHandles) {
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(Vector::ZERO);
    world.integration_parameters_mut().dt = 1.0 / 60.0;
    world.set_stress_settings(StressSettings {
        damage_per_overload: 2.0,
        max_iterations: 2,
        ..StressSettings::default()
    });
    world.set_quick_impact_settings(settings);
    world
        .add_destructible(FxFamilyId(1), two_node_asset(7))
        .unwrap();
    world
        .add_destructible(FxFamilyId(2), two_node_asset(7))
        .unwrap();
    let a = world.actor_handles(FxFamilyId(1), FxActorId(0)).unwrap();
    let b = world.actor_handles(FxFamilyId(2), FxActorId(0)).unwrap();
    {
        let body = world.rigid_bodies_mut().get_mut(a.body).unwrap();
        body.set_position(Pose::from_translation(Vector::new(1.0, 0.5)), true);
        body.set_linvel(Vector::new(speed, 0.0), true);
    }
    {
        let body = world.rigid_bodies_mut().get_mut(b.body).unwrap();
        body.set_position(Pose::from_translation(Vector::new(2.5, 0.5)), true);
        body.set_linvel(Vector::new(-speed, 0.0), true);
    }
    (world, a, b)
}

#[test]
fn static_anchor_marks_actor_fixed_or_kinematic() {
    let family = FxFamilyId(1);

    let mut default_world = FxRapierWorld2D::new();
    default_world
        .add_destructible(family, single_node_asset(7))
        .unwrap();
    let default_handles = default_world.actor_handles(family, FxActorId(0)).unwrap();
    assert!(
        default_world
            .rigid_bodies()
            .get(default_handles.body)
            .unwrap()
            .is_dynamic()
    );
    default_world
        .connect_static_anchor(
            family,
            StaticAnchorConnectionDesc::new(static_anchor_desc(1, 0)),
        )
        .unwrap();
    assert!(
        default_world
            .rigid_bodies()
            .get(default_handles.body)
            .unwrap()
            .is_dynamic(),
        "default static-anchor policy must preserve the dynamic body type"
    );

    let mut fixed_world = FxRapierWorld2D::new();
    fixed_world
        .add_destructible(family, single_node_asset(7))
        .unwrap();
    let fixed_handles = fixed_world.actor_handles(family, FxActorId(0)).unwrap();
    fixed_world
        .connect_static_anchor(
            family,
            StaticAnchorConnectionDesc::new(static_anchor_desc(2, 0))
                .with_body_policy(StaticAnchorBodyPolicy::Fixed),
        )
        .unwrap();
    assert!(
        fixed_world
            .rigid_bodies()
            .get(fixed_handles.body)
            .unwrap()
            .is_fixed()
    );

    let mut kinematic_world = FxRapierWorld2D::new();
    kinematic_world
        .add_destructible(family, single_node_asset(7))
        .unwrap();
    let kinematic_handles = kinematic_world.actor_handles(family, FxActorId(0)).unwrap();
    kinematic_world
        .connect_static_anchor(
            family,
            StaticAnchorConnectionDesc::new(static_anchor_desc(3, 0))
                .with_body_policy(StaticAnchorBodyPolicy::KinematicVelocityBased),
        )
        .unwrap();
    assert!(
        kinematic_world
            .rigid_bodies()
            .get(kinematic_handles.body)
            .unwrap()
            .is_kinematic()
    );
}

#[test]
fn static_anchor_policy_moves_to_split_child() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(Vector::ZERO);
    world
        .add_destructible(family, four_node_line_asset())
        .unwrap();
    world
        .connect_static_anchor(
            family,
            StaticAnchorConnectionDesc::new(static_anchor_desc(9, 3))
                .with_body_policy(StaticAnchorBodyPolicy::Fixed),
        )
        .unwrap();
    let before = world.actor_handles(family, FxActorId(0)).unwrap();
    assert!(world.rigid_bodies().get(before.body).unwrap().is_fixed());

    let split = world
        .fracture_and_sync_for_test(
            family,
            &[break_bond_command(0, family, FxActorId(0), BondId(2))],
        )
        .unwrap();

    assert_eq!(split.len(), 1);
    assert_eq!(split[0].created_children, vec![FxActorId(1)]);
    let kept = world.actor_handles(family, FxActorId(0)).unwrap();
    let child = world.actor_handles(family, FxActorId(1)).unwrap();
    assert!(
        world.rigid_bodies().get(kept.body).unwrap().is_dynamic(),
        "unanchored kept fragment must not keep stale fixed policy"
    );
    assert!(
        world.rigid_bodies().get(child.body).unwrap().is_fixed(),
        "anchored child fragment must receive the live anchor body policy"
    );
}

#[test]
fn static_anchor_grouping_prevents_spurious_split() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(Vector::ZERO);
    world
        .add_destructible(family, four_node_line_asset())
        .unwrap();
    world
        .connect_static_anchor(
            family,
            StaticAnchorConnectionDesc::new(static_anchor_desc(21, 0))
                .with_body_policy(StaticAnchorBodyPolicy::Fixed),
        )
        .unwrap();
    world
        .connect_static_anchor(
            family,
            StaticAnchorConnectionDesc::new(static_anchor_desc(22, 3))
                .with_body_policy(StaticAnchorBodyPolicy::Fixed),
        )
        .unwrap();
    let actor = world.actor_handles(family, FxActorId(0)).unwrap();
    assert!(world.rigid_bodies().get(actor.body).unwrap().is_fixed());

    let split = world
        .fracture_and_sync_for_test(
            family,
            &[break_bond_command(0, family, FxActorId(0), BondId(1))],
        )
        .unwrap();

    assert!(
        split.is_empty(),
        "fragments attached to the same live static target must remain one split island"
    );
    assert_eq!(world.family(family).unwrap().actor_count(), 1);
    let actor = world.actor_handles(family, FxActorId(0)).unwrap();
    assert!(
        world.rigid_bodies().get(actor.body).unwrap().is_fixed(),
        "no-split anchored actor must retain the requested static-anchor body policy"
    );
}

#[test]
fn broken_static_anchor_clears_body_policy() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(Vector::ZERO);
    world
        .add_destructible(family, single_node_asset(7))
        .unwrap();
    world
        .connect_static_anchor(
            family,
            StaticAnchorConnectionDesc::new(static_anchor_desc(11, 0))
                .with_body_policy(StaticAnchorBodyPolicy::Fixed),
        )
        .unwrap();
    let actor = world.actor_handles(family, FxActorId(0)).unwrap();
    assert!(world.rigid_bodies().get(actor.body).unwrap().is_fixed());

    let split = world
        .fracture_and_sync_for_test(
            family,
            &[break_external_bond_command(
                0,
                family,
                FxActorId(0),
                ExternalBondId(11),
            )],
        )
        .unwrap();

    assert!(split.is_empty());
    assert!(
        world.rigid_bodies().get(actor.body).unwrap().is_dynamic(),
        "broken external bond must release the temporary anchor body policy"
    );
}

#[test]
fn dynamic_bond_graph_only() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    world
        .add_destructible(family, disconnected_two_node_asset())
        .unwrap();
    let actor0 = world.actor_handles(family, FxActorId(0)).unwrap();
    let actor1 = world.actor_handles(family, FxActorId(1)).unwrap();
    let before_joints = world.impulse_joints().len();

    let connection = world
        .connect_dynamic_structural_bond(
            family,
            DynamicStructuralConnectionDesc::graph_only(dynamic_graph_only_desc(4, 0, 1)),
        )
        .unwrap();

    assert_eq!(connection, ConnectionId(4));
    assert_eq!(world.impulse_joints().len(), before_joints);
    assert_eq!(world.actor_handles(family, FxActorId(0)), Some(actor0));
    assert_eq!(world.actor_handles(family, FxActorId(1)), Some(actor1));
    assert_eq!(world.family(family).unwrap().actor_count(), 2);
    assert_eq!(
        world
            .family(family)
            .unwrap()
            .dynamic_structural_bond(connection)
            .unwrap()
            .policy,
        DynamicConnectionPolicy::GraphOnly
    );
}

#[test]
fn dynamic_bond_custom_hard_constraint_is_future_error() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    world
        .add_destructible(family, disconnected_two_node_asset())
        .unwrap();
    let before_joints = world.impulse_joints().len();
    let before_digest = world.family(family).unwrap().deterministic_state_digest();

    let err = world
        .connect_dynamic_structural_bond(
            family,
            DynamicStructuralConnectionDesc::custom_hard_constraint(dynamic_graph_only_desc(
                4, 0, 1,
            )),
        )
        .unwrap_err();

    assert_eq!(
        err,
        FxRapierError::UnsupportedConnectionPolicy(DynamicConnectionPolicy::CustomHardConstraint)
    );
    assert_eq!(world.impulse_joints().len(), before_joints);
    assert_eq!(
        world.family(family).unwrap().deterministic_state_digest(),
        before_digest
    );
    assert_eq!(
        world
            .family(family)
            .unwrap()
            .dynamic_structural_bonds()
            .count(),
        0
    );
}

#[test]
fn dynamic_merge_conserves_mass_com_velocity() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    world
        .add_destructible(family, disconnected_two_node_asset())
        .unwrap();
    world
        .connect_dynamic_structural_bond(
            family,
            DynamicStructuralConnectionDesc::graph_only(dynamic_graph_only_desc(21, 0, 1)),
        )
        .unwrap();
    let actor0 = world.actor_handles(family, FxActorId(0)).unwrap();
    let actor1 = world.actor_handles(family, FxActorId(1)).unwrap();
    {
        let body0 = world.rigid_bodies_mut().get_mut(actor0.body).unwrap();
        body0.set_position(Pose::from_translation(Vector::new(-4.0, 2.0)), true);
        body0.set_linvel(Vector::new(3.0, -1.0), true);
        body0.set_angvel(0.25, true);

        let body1 = world.rigid_bodies_mut().get_mut(actor1.body).unwrap();
        body1.set_position(Pose::from_translation(Vector::new(6.0, -2.0)), true);
        body1.set_linvel(Vector::new(-1.0, 5.0), true);
        body1.set_angvel(-0.5, true);
    }
    let mass0 = world.rigid_bodies().get(actor0.body).unwrap().mass();
    let mass1 = world.rigid_bodies().get(actor1.body).unwrap().mass();
    let vel0 = world.rigid_bodies().get(actor0.body).unwrap().linvel();
    let vel1 = world.rigid_bodies().get(actor1.body).unwrap().linvel();
    let com0 = world
        .rigid_bodies()
        .get(actor0.body)
        .unwrap()
        .center_of_mass();
    let com1 = world
        .rigid_bodies()
        .get(actor1.body)
        .unwrap()
        .center_of_mass();
    let expected_velocity = (vel0 * mass0 + vel1 * mass1) / (mass0 + mass1);
    let expected_com = (com0 * mass0 + com1 * mass1) / (mass0 + mass1);
    let before_joints = world.impulse_joints().len();

    let result = world
        .merge_actors(family, FxActorId(1), FxActorId(0))
        .unwrap();

    assert_eq!(result.kept_actor, FxActorId(0));
    assert_eq!(result.removed_actor, FxActorId(1));
    assert_eq!(world.impulse_joints().len(), before_joints);
    assert!(world.actor_handles(family, FxActorId(1)).is_none());
    assert!(world.rigid_bodies().get(actor1.body).is_none());
    assert!(world.colliders().get(actor1.collider).is_none());
    let merged = world.actor_handles(family, FxActorId(0)).unwrap();
    assert_eq!(merged.body, actor0.body);
    assert_ne!(merged.collider, actor0.collider);
    let merged_body = world.rigid_bodies().get(merged.body).unwrap();
    assert_vector_close(merged_body.center_of_mass(), expected_com);
    assert_vector_close(merged_body.linvel(), expected_velocity);
    assert_scalar_close(merged_body.mass(), mass0 + mass1);
    assert_eq!(world.family(family).unwrap().actor_count(), 1);
    assert!(!world.family(family).unwrap().is_dirty(FxActorId(0)));
}

#[test]
fn dynamic_merge_requires_graph_connection_no_side_effects() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    world
        .add_destructible(family, disconnected_two_node_asset())
        .unwrap();
    let actor0 = world.actor_handles(family, FxActorId(0)).unwrap();
    let actor1 = world.actor_handles(family, FxActorId(1)).unwrap();
    let before_digest = world.family(family).unwrap().deterministic_state_digest();
    let before_joints = world.impulse_joints().len();

    let err = world
        .merge_actors(family, FxActorId(0), FxActorId(1))
        .unwrap_err();

    assert_eq!(
        err,
        FxRapierError::Connection(ConnectionError::MissingMergeConnection {
            actor_a: FxActorId(0),
            actor_b: FxActorId(1),
        })
    );
    assert_eq!(
        world.family(family).unwrap().deterministic_state_digest(),
        before_digest
    );
    assert_eq!(world.impulse_joints().len(), before_joints);
    assert_eq!(world.actor_handles(family, FxActorId(0)), Some(actor0));
    assert_eq!(world.actor_handles(family, FxActorId(1)), Some(actor1));
    assert!(world.rigid_bodies().get(actor0.body).is_some());
    assert!(world.rigid_bodies().get(actor1.body).is_some());
    assert!(world.colliders().get(actor0.collider).is_some());
    assert!(world.colliders().get(actor1.collider).is_some());
    assert_eq!(world.family(family).unwrap().actor_count(), 2);
}

#[test]
fn contact_mapping_pair_order() {
    let (mut world, destructible, ordinary) = overlapping_side_contact_world();
    world.step().unwrap();

    let pair = world
        .narrow_phase()
        .contact_pair(ordinary, destructible)
        .expect("real narrow-phase contact pair");
    let registry = world.contact_registry_snapshot();
    let mapping = map_contact_pair(pair, &registry).expect("destructible side is mapped");

    assert_eq!(mapping.destructible_collider, destructible);
    assert_eq!(mapping.other_collider, ordinary);
    assert_eq!(
        mapping.side,
        if pair.collider1 == destructible {
            ContactPairSide::Collider1
        } else {
            ContactPairSide::Collider2
        }
    );
}

#[test]
fn tracked_impulse_readback() {
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(Vector::new(0.0, -9.81));
    world
        .add_destructible(FxFamilyId(1), two_node_multi_material_asset(7, 8))
        .unwrap();
    add_fixed_box(
        &mut world,
        Vector::new(0.5, -0.05),
        Vector::new(0.45, 0.1),
        1.0,
    );
    add_fixed_box(
        &mut world,
        Vector::new(1.5, -0.05),
        Vector::new(0.45, 0.1),
        1.0,
    );

    let mut hits = Vec::new();
    for _ in 0..12 {
        let step = world.step_with_diagnostics().unwrap();
        assert_eq!(
            step.diagnostics.contact_impulse_readback_miss_count, 0,
            "tracked contact impulses must be sourced through the pre-solver cache"
        );
        hits = step
            .report
            .contact_impulses
            .into_iter()
            .filter(|input| input.impulse.normal_impulse > 0.0)
            .collect();
        let nodes = hits
            .iter()
            .map(|input| input.stress.node)
            .collect::<BTreeSet<_>>();
        let materials = hits
            .iter()
            .filter_map(|input| input.impulse.voxel.map(|voxel| voxel.contact_material))
            .collect::<BTreeSet<_>>();
        if nodes.contains(&SupportNodeId(0))
            && nodes.contains(&SupportNodeId(1))
            && materials.contains(&7)
            && materials.contains(&8)
        {
            break;
        }
    }
    assert!(
        hits.iter()
            .all(|input| input.impulse.source_tracked_geometric_contact)
    );
    assert!(
        hits.iter()
            .all(|input| input.impulse.source_pre_solver_cache)
    );
    assert!(hits.iter().all(|input| !input.impulse.used_fallback));
    assert!(hits.iter().all(|input| input.impulse.voxel.is_some()));
    assert!(
        hits.iter()
            .any(|input| input.stress.node == SupportNodeId(0)
                && input.impulse.voxel.unwrap().contact_material == 7)
    );
    assert!(
        hits.iter()
            .any(|input| input.stress.node == SupportNodeId(1)
                && input.impulse.voxel.unwrap().contact_material == 8)
    );
}

#[test]
fn contact_hook_material() {
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(Vector::ZERO);
    world
        .add_destructible(FxFamilyId(1), two_node_multi_material_asset(7, 8))
        .unwrap();
    let destructible = world.actor_handles(FxFamilyId(1), FxActorId(0)).unwrap();
    add_fixed_box(
        &mut world,
        Vector::new(0.5, -0.05),
        Vector::new(0.45, 0.1),
        1.0,
    );
    add_fixed_box(
        &mut world,
        Vector::new(1.5, -0.05),
        Vector::new(0.45, 0.1),
        1.0,
    );
    world.set_contact_material_properties(
        7,
        ContactMaterialProperties {
            friction: 0.21,
            restitution: 0.17,
        },
    );
    world.set_contact_material_properties(
        8,
        ContactMaterialProperties {
            friction: 0.82,
            restitution: 0.38,
        },
    );
    world.step().unwrap();

    let observations = world.drain_contact_hook_observations();
    let destructible_observations = observations
        .iter()
        .filter(|obs| {
            obs.collider1 == destructible.collider || obs.collider2 == destructible.collider
        })
        .collect::<Vec<_>>();
    assert!(
        destructible_observations
            .iter()
            .all(|obs| obs.before_solver_contacts == obs.after_solver_contacts)
    );
    assert!(
        destructible_observations
            .iter()
            .all(|obs| !obs.used_fallback)
    );
    assert!(
        destructible_observations
            .iter()
            .any(|obs| { obs.material == 7 && obs.friction == 0.21 && obs.restitution == 0.17 })
    );
    assert!(
        destructible_observations
            .iter()
            .any(|obs| { obs.material == 8 && obs.friction == 0.82 && obs.restitution == 0.38 })
    );

    let has_modified_solver_contact = world
        .narrow_phase()
        .contact_pairs_with(destructible.collider)
        .flat_map(|pair| &pair.manifolds)
        .flat_map(|manifold| &manifold.data.solver_contacts)
        .filter(|contact| {
            (contact.friction == 0.21 && contact.restitution == 0.17)
                || (contact.friction == 0.82 && contact.restitution == 0.38)
        })
        .count();
    assert!(has_modified_solver_contact >= 2);
}

#[test]
fn quick_impact_suppression_generates_quick_stress_without_post_solve_double_count() {
    let settings = quick_impact_settings(0.01, 0.02, 1000.0, 2000.0);
    let (mut world, _) = quick_impact_wall_world(settings, 8.0);

    let step = world.step_with_diagnostics().unwrap();

    assert!(
        step.report
            .quick_impacts
            .iter()
            .any(|input| input.impact.estimate.action == QuickImpactAction::Suppress)
    );
    assert!(
        step.report
            .stress_inputs
            .iter()
            .any(|input| input.source == DamageSource::ContactImpulse
                && input.order_key.source_priority == 5)
    );
    assert!(
        step.report.contact_impulses.is_empty(),
        "suppressed contacts must not also feed post-solve contact impulse stress"
    );
}

#[test]
fn quick_impact_softening_is_observable_without_post_solve_double_count() {
    let settings = quick_impact_settings(0.01, 1000.0, 1000.0, 2000.0);
    let (mut world, handles) = quick_impact_wall_world(settings, 8.0);
    world.set_contact_material_properties(
        7,
        ContactMaterialProperties {
            friction: 0.8,
            restitution: 0.5,
        },
    );

    let step = world.step_with_diagnostics().unwrap();
    let observations = world.drain_contact_hook_observations();

    assert!(
        step.report
            .quick_impacts
            .iter()
            .any(|input| input.impact.estimate.action == QuickImpactAction::Soften)
    );
    assert!(
        observations
            .iter()
            .filter(|obs| obs.destructible_collider == handles.collider)
            .any(|obs| {
                obs.quick_impact
                    .is_some_and(|impact| impact.action == QuickImpactAction::Soften)
                    && obs.after_solver_contacts > 0
            })
    );
    assert!(
        step.report.contact_impulses.is_empty(),
        "softened quick-impact contacts must not also feed post-solve contact impulse stress"
    );
}

#[test]
fn quick_impact_small_debris_does_not_suppress() {
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(Vector::ZERO);
    world.set_quick_impact_settings(quick_impact_settings(0.01, 0.02, 1000.0, 2000.0));
    world
        .add_destructible(FxFamilyId(1), single_node_asset(7))
        .unwrap();
    let handles = world.actor_handles(FxFamilyId(1), FxActorId(0)).unwrap();
    world
        .rigid_bodies_mut()
        .get_mut(handles.body)
        .unwrap()
        .set_linvel(Vector::new(8.0, 0.0), true);
    add_fixed_box(
        &mut world,
        Vector::new(1.0, 0.5),
        Vector::new(0.5, 0.5),
        1.0,
    );

    let step = world.step_with_diagnostics().unwrap();
    let observations = world.drain_contact_hook_observations();

    assert!(step.report.quick_impacts.is_empty());
    assert!(
        observations
            .iter()
            .filter(|obs| obs.destructible_collider == handles.collider)
            .all(|obs| obs.before_solver_contacts == obs.after_solver_contacts)
    );
}

#[test]
fn quick_impact_mixed_small_debris_pair_preserves_solver_contacts_and_readback() {
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(Vector::ZERO);
    world.integration_parameters_mut().dt = 1.0 / 60.0;
    world.set_stress_settings(StressSettings {
        damage_per_overload: 2.0,
        max_iterations: 2,
        ..StressSettings::default()
    });
    world.set_quick_impact_settings(quick_impact_settings(0.01, 0.02, 0.01, 0.02));
    world.set_material_impact_hardness(7, 1.0);
    world
        .add_destructible(FxFamilyId(1), two_node_asset(7))
        .unwrap();
    world
        .add_destructible(FxFamilyId(2), single_node_asset(7))
        .unwrap();
    let large = world.actor_handles(FxFamilyId(1), FxActorId(0)).unwrap();
    let small = world.actor_handles(FxFamilyId(2), FxActorId(0)).unwrap();
    {
        let body = world.rigid_bodies_mut().get_mut(large.body).unwrap();
        body.set_position(Pose::from_translation(Vector::new(1.0, 0.5)), true);
        body.set_linvel(Vector::new(8.0, 0.0), true);
    }
    {
        let body = world.rigid_bodies_mut().get_mut(small.body).unwrap();
        body.set_position(Pose::from_translation(Vector::new(2.0, 0.5)), true);
        body.set_linvel(Vector::new(-8.0, 0.0), true);
    }

    let step = world.step_with_diagnostics().unwrap();
    let observations = world.drain_contact_hook_observations();
    let pair_observations = observations
        .iter()
        .filter(|obs| {
            (obs.collider1 == large.collider && obs.collider2 == small.collider)
                || (obs.collider1 == small.collider && obs.collider2 == large.collider)
        })
        .collect::<Vec<_>>();
    let impulse_families = step
        .report
        .contact_impulses
        .iter()
        .map(|input| input.stress.order_key.family_id)
        .collect::<BTreeSet<_>>();

    assert!(
        !pair_observations.is_empty(),
        "mixed destructible pair must reach the contact hook"
    );
    assert!(step.report.quick_impacts.is_empty());
    assert!(
        pair_observations
            .iter()
            .all(|obs| obs.quick_impact.is_none()
                && obs.before_solver_contacts == obs.after_solver_contacts),
        "any small-debris side must disable pair-level quick-impact suppression/softening"
    );
    assert_eq!(step.diagnostics.contact_impulse_readback_miss_count, 0);
    assert!(impulse_families.contains(&FxFamilyId(1)));
    assert!(impulse_families.contains(&FxFamilyId(2)));
}

#[test]
fn quick_impact_dynamic_opponent_uses_higher_threshold_than_static() {
    let settings = quick_impact_settings(0.01, 0.02, 1000.0, 2000.0);
    let (mut static_world, _) = quick_impact_wall_world(settings, 8.0);
    let static_step = static_world.step_with_diagnostics().unwrap();

    let (mut dynamic_world, _, _) = quick_impact_dynamic_world(settings, 8.0);
    let dynamic_step = dynamic_world.step_with_diagnostics().unwrap();

    assert!(
        static_step.report.quick_impacts.iter().any(|input| !input
            .impact
            .estimate
            .dynamic_opponent
            && input.impact.estimate.action == QuickImpactAction::Suppress)
    );
    assert!(
        dynamic_step.report.quick_impacts.is_empty(),
        "same-speed destructible-vs-destructible contact should stay below the higher dynamic threshold"
    );
}

#[test]
fn quick_impact_destructible_vs_destructible_is_generic() {
    let settings = quick_impact_settings(0.01, 0.02, 0.01, 0.02);
    let (mut world, _, _) = quick_impact_dynamic_world(settings, 8.0);

    let step = world.step_with_diagnostics().unwrap();
    let families = step
        .report
        .quick_impacts
        .iter()
        .map(|input| input.stress.order_key.family_id)
        .collect::<BTreeSet<_>>();

    assert!(families.contains(&FxFamilyId(1)));
    assert!(families.contains(&FxFamilyId(2)));
}

#[test]
fn fracture_field_stress_injection_can_split() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(Vector::ZERO);
    world.set_stress_settings(StressSettings {
        damage_per_overload: 2.0,
        max_iterations: 2,
        ..StressSettings::default()
    });
    world.add_destructible(family, two_node_asset(7)).unwrap();
    world.queue_fracture_field(
        FractureField2D::stress(Vec2::new(0.5, 0.5), 0.25, Vec2::new(50.0, 0.0))
            .with_family(family),
    );

    let step = world.step_with_diagnostics().unwrap();

    assert!(
        step.report
            .fracture_field_effects
            .iter()
            .any(|effect| effect.mode == FractureFieldMode::Stress)
    );
    assert!(!step.report.fracture_events.is_empty());
    assert!(!step.report.split_events.is_empty());
}

#[test]
fn fracture_field_direct_damage_can_split_locally() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(Vector::ZERO);
    world.add_destructible(family, two_node_asset(7)).unwrap();
    world.queue_fracture_field(
        FractureField2D::direct_damage(Vec2::new(1.5, 0.5), 0.25, 2.0).with_family(family),
    );

    let step = world.step_with_diagnostics().unwrap();

    assert_eq!(step.report.fracture_field_effects.len(), 1);
    assert_eq!(step.report.fracture_events.len(), 1);
    assert_eq!(step.report.fracture_events[0].source, DamageSource::Script);
    assert_eq!(step.report.split_events.len(), 1);
}

#[test]
fn fracture_field_direct_damage_does_not_leak_after_later_unknown_family_error() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(Vector::ZERO);
    world.add_destructible(family, two_node_asset(7)).unwrap();
    world.queue_fracture_field(
        FractureField2D::direct_damage(Vec2::new(1.5, 0.5), 0.25, 2.0).with_family(family),
    );
    world.queue_fracture_field(
        FractureField2D::direct_damage(Vec2::new(1.5, 0.5), 0.25, 2.0).with_family(FxFamilyId(99)),
    );

    assert!(matches!(
        world.step_with_diagnostics(),
        Err(FxRapierError::UnknownFamily(FxFamilyId(99)))
    ));

    let step = world.step_with_diagnostics().unwrap();

    assert!(step.report.fracture_events.is_empty());
    assert!(step.report.split_events.is_empty());
    assert_eq!(world.family(family).unwrap().actor_count(), 1);
}

#[test]
fn fracture_field_direct_damage_bypasses_stress_cap_and_diagnostics() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(Vector::ZERO);
    world.set_stress_settings(StressSettings {
        max_fractures_per_frame: 0,
        ..StressSettings::default()
    });
    world.add_destructible(family, two_node_asset(7)).unwrap();
    world.queue_fracture_field(
        FractureField2D::direct_damage(Vec2::new(1.5, 0.5), 0.25, 2.0).with_family(family),
    );

    let step = world.step_with_diagnostics().unwrap();

    assert_eq!(step.report.fracture_events.len(), 1);
    assert_eq!(step.report.split_events.len(), 1);
    assert_eq!(step.diagnostics.global_stress_cap.input_count, 0);
    assert_eq!(
        step.diagnostics
            .global_stress_cap
            .generated_commands_before_cap,
        0
    );
    assert_eq!(
        step.diagnostics
            .global_stress_cap
            .generated_commands_after_cap,
        0
    );
    assert_eq!(step.diagnostics.global_stress_cap.frame_cap, 0);
}

#[test]
fn fracture_field_radius_and_family_filter_apply() {
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(Vector::ZERO);
    world
        .add_destructible(FxFamilyId(1), two_node_asset(7))
        .unwrap();
    world
        .add_destructible(FxFamilyId(2), two_node_asset(7))
        .unwrap();
    let family2 = world.actor_handles(FxFamilyId(2), FxActorId(0)).unwrap();
    world
        .rigid_bodies_mut()
        .get_mut(family2.body)
        .unwrap()
        .set_position(Pose::from_translation(Vector::new(5.0, 0.5)), true);
    world.queue_fracture_field(
        FractureField2D::direct_damage(Vec2::new(5.5, 0.5), 0.25, 2.0).with_family(FxFamilyId(2)),
    );

    let step = world.step_with_diagnostics().unwrap();

    assert_eq!(step.report.fracture_field_effects.len(), 1);
    assert!(
        step.report
            .fracture_field_effects
            .iter()
            .all(|effect| effect.family == FxFamilyId(2) && effect.node == SupportNodeId(1))
    );
    assert!(
        step.report
            .fracture_events
            .iter()
            .all(|event| event.family == FxFamilyId(2))
    );
    assert_eq!(world.family(FxFamilyId(1)).unwrap().actor_count(), 1);
    assert_eq!(world.family(FxFamilyId(2)).unwrap().actor_count(), 2);
}

#[test]
fn non_rectangular_per_voxel_collider_no_aabb_fill() {
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(Vector::ZERO);
    world
        .add_destructible(FxFamilyId(1), l_shaped_asset())
        .unwrap();
    let destructible = world.actor_handles(FxFamilyId(1), FxActorId(0)).unwrap();
    let (_, hole_probe) = add_fixed_box(
        &mut world,
        Vector::new(1.5, 0.5),
        Vector::new(0.2, 0.2),
        0.5,
    );

    world.step().unwrap();
    let active_contact = world
        .narrow_phase()
        .contact_pair(destructible.collider, hole_probe)
        .is_some_and(|pair| pair.has_any_active_contact());
    assert!(
        !active_contact,
        "missing grid cell must remain empty, not collide as an actor AABB"
    );
}

#[test]
fn small_debris_lod_is_default_on() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    assert_eq!(
        ColliderLodSettings::default(),
        ColliderLodSettings::small_debris_box(4, 1)
    );
    assert_eq!(
        world.lod_settings(),
        ColliderLodSettings::small_debris_box(4, 1)
    );
    world
        .add_destructible(family, two_voxel_single_node_asset(7))
        .unwrap();

    let handles = world.actor_handles(family, FxActorId(0)).unwrap();
    let registry = world.contact_registry_snapshot();
    let voxels = registry
        .collider_voxels
        .get(&collider_key(handles.collider))
        .unwrap();

    assert_eq!(voxels.len(), 1);
    assert_eq!(voxels[0].coord, fracture_core::GridCoord::new(0, 0));
    assert_eq!(voxels[0].node, SupportNodeId(0));
    assert_eq!(voxels[0].contact_material, 7);
    assert!(
        world
            .colliders()
            .get(handles.collider)
            .unwrap()
            .shape()
            .as_cuboid()
            .is_some()
    );
}

#[test]
fn small_debris_lod_can_be_disabled() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    world.set_lod_settings(ColliderLodSettings::disabled());
    world
        .add_destructible(family, two_voxel_single_node_asset(7))
        .unwrap();

    let handles = world.actor_handles(family, FxActorId(0)).unwrap();
    let registry = world.contact_registry_snapshot();
    let voxels = registry
        .collider_voxels
        .get(&collider_key(handles.collider))
        .unwrap();

    assert_eq!(voxels.len(), 2);
    assert_eq!(voxels[0].coord, fracture_core::GridCoord::new(0, 0));
    assert_eq!(voxels[1].coord, fracture_core::GridCoord::new(1, 0));
    assert_eq!(voxels[0].node, SupportNodeId(0));
    assert_eq!(voxels[1].node, SupportNodeId(0));
    assert_eq!(voxels[0].contact_material, 7);
    assert_eq!(voxels[1].contact_material, 7);
    assert!(
        world
            .colliders()
            .get(handles.collider)
            .unwrap()
            .shape()
            .as_compound()
            .is_some()
    );
}

#[test]
fn small_debris_lod_requires_one_node_threshold() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    world.set_lod_settings(ColliderLodSettings::small_debris_box(4, 4));
    world
        .add_destructible(family, two_voxel_single_node_asset(7))
        .unwrap();

    let handles = world.actor_handles(family, FxActorId(0)).unwrap();
    let registry = world.contact_registry_snapshot();
    let voxels = registry
        .collider_voxels
        .get(&collider_key(handles.collider))
        .unwrap();

    assert_eq!(voxels.len(), 2);
    assert!(
        world
            .colliders()
            .get(handles.collider)
            .unwrap()
            .shape()
            .as_compound()
            .is_some()
    );
}

#[test]
fn multi_node_actors_remain_voxel_compounds() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    world.add_destructible(family, two_node_asset(7)).unwrap();

    let handles = world.actor_handles(family, FxActorId(0)).unwrap();
    let registry = world.contact_registry_snapshot();
    let voxels = registry
        .collider_voxels
        .get(&collider_key(handles.collider))
        .unwrap();

    assert_eq!(voxels.len(), 2);
    assert!(
        world
            .colliders()
            .get(handles.collider)
            .unwrap()
            .shape()
            .as_compound()
            .is_some()
    );
}

#[test]
fn performance_budget_report_is_observable_and_validation_only() {
    let mut world = FxRapierWorld2D::new();
    for id in 0..=100 {
        world
            .add_destructible(FxFamilyId(id), single_node_asset(7))
            .unwrap();
    }

    let report = world.performance_budget_report();

    assert_eq!(report.occupied_voxels, 101);
    assert_eq!(report.support_nodes, 101);
    assert_eq!(report.active_bodies, 101);
    assert!(!report.within_budget());
    assert!(world.validate_performance_budget().is_err());
    assert_eq!(world.rigid_bodies().len(), 101);
}

#[test]
fn step_diagnostics_include_budget_when_no_fracture_occurs() {
    let mut world = FxRapierWorld2D::new();
    world
        .add_destructible(FxFamilyId(1), two_node_asset(7))
        .unwrap();

    let step = world.step_with_diagnostics().unwrap();
    let budget = step.diagnostics.budget.unwrap();

    assert!(step.report.split_events.is_empty());
    assert_eq!(budget.occupied_voxels, 2);
    assert_eq!(budget.support_nodes, 2);
    assert_eq!(budget.active_bodies, 1);
    assert!(budget.within_budget());
}

#[test]
fn stress_frame_cap_is_global_across_families() {
    let family_a = FxFamilyId(1);
    let family_b = FxFamilyId(2);
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(Vector::ZERO);
    world.set_stress_settings(StressSettings {
        damage_per_overload: 2.0,
        max_fractures_per_frame: 1,
        ..StressSettings::default()
    });
    world.add_destructible(family_a, two_node_asset(7)).unwrap();
    world.add_destructible(family_b, two_node_asset(7)).unwrap();

    let step = world
        .step_with_stress_inputs_for_test(vec![
            StressInput {
                order_key: DeterministicOrderKey::new(0, 1, family_a, FxActorId(0), CommandId(0)),
                actor: FxActorId(0),
                node: SupportNodeId(0),
                force: Vec2::new(10.0, 0.0),
                source: DamageSource::Stress,
            },
            StressInput {
                order_key: DeterministicOrderKey::new(0, 1, family_b, FxActorId(0), CommandId(0)),
                actor: FxActorId(0),
                node: SupportNodeId(0),
                force: Vec2::new(10.0, 0.0),
                source: DamageSource::Stress,
            },
        ])
        .unwrap();

    assert_eq!(step.report.fracture_events.len(), 1);
    assert_eq!(step.report.split_events.len(), 1);
    assert_eq!(step.report.fracture_events[0].family, family_a);
    assert_eq!(step.diagnostics.global_stress_cap.input_count, 2);
    assert_eq!(step.diagnostics.global_stress_cap.family_count, 2);
    assert_eq!(
        step.diagnostics
            .global_stress_cap
            .generated_commands_before_cap,
        2
    );
    assert_eq!(
        step.diagnostics
            .global_stress_cap
            .generated_commands_after_cap,
        1
    );
    assert_eq!(step.diagnostics.global_stress_cap.frame_cap, 1);
    assert_eq!(
        step.diagnostics
            .stress_profiles
            .iter()
            .map(|profile| profile.generated_commands_after_cap)
            .sum::<usize>(),
        1
    );
}

#[test]
fn stress_gravity_static_anchor_without_contact_or_joint_input() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(Vector::new(0.0, -9.81));
    world.set_stress_settings(StressSettings {
        damage_per_overload: 1.0,
        max_iterations: 1,
        ..StressSettings::default()
    });
    world
        .add_destructible(family, single_node_asset(7))
        .unwrap();
    world
        .connect_static_anchor(
            family,
            StaticAnchorConnectionDesc::new(static_anchor_desc(77, 0))
                .with_body_policy(StaticAnchorBodyPolicy::Fixed),
        )
        .unwrap();

    let step = world.step_with_diagnostics().unwrap();

    assert!(step.report.contact_impulses.is_empty());
    assert!(step.report.joint_feedback.is_empty());
    assert!(step.report.stress_inputs.is_empty());
    assert!(step.report.fracture_events.is_empty());
    assert!(step.report.split_events.is_empty());
    assert_eq!(world.prestress_baseline_count_for_test(), 1);

    let step = world.step_with_diagnostics().unwrap();

    assert!(step.report.contact_impulses.is_empty());
    assert!(step.report.joint_feedback.is_empty());
    assert!(step.report.stress_inputs.is_empty());
    assert!(step.report.fracture_events.is_empty());
    assert!(step.report.split_events.is_empty());
    assert_eq!(world.prestress_baseline_count_for_test(), 1);
}

#[test]
fn prestress_baseline_extra_test_input_still_fractures() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(Vector::new(0.0, -1.0));
    world.set_stress_settings(StressSettings {
        damage_per_overload: 1.0,
        max_iterations: 1,
        ..StressSettings::default()
    });
    world
        .add_destructible(family, single_node_asset(7))
        .unwrap();
    let anchor = world
        .connect_static_anchor(
            family,
            StaticAnchorConnectionDesc::new(static_anchor_desc(77, 0))
                .with_body_policy(StaticAnchorBodyPolicy::Fixed),
        )
        .unwrap();

    let initial = world.step_with_diagnostics().unwrap();

    assert!(initial.report.fracture_events.is_empty());
    assert!(initial.report.split_events.is_empty());
    assert_eq!(world.prestress_baseline_count_for_test(), 1);

    let step = world
        .step_with_stress_inputs_for_test(vec![StressInput {
            order_key: DeterministicOrderKey::new(
                world.tick(),
                1,
                family,
                FxActorId(0),
                CommandId(1),
            ),
            actor: FxActorId(0),
            node: SupportNodeId(0),
            force: Vec2::new(0.0, -1.0),
            source: DamageSource::ContactImpulse,
        }])
        .unwrap();

    assert_eq!(step.report.fracture_events.len(), 1);
    assert_eq!(
        step.report.fracture_events[0].target,
        FractureTarget::ExternalBond(anchor)
    );
    assert_eq!(
        step.report.fracture_events[0].source,
        DamageSource::ContactImpulse
    );
    assert_eq!(world.prestress_baseline_count_for_test(), 0);
}

#[test]
fn prestress_baseline_gravity_delta_fractures() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(Vector::new(0.0, -1.0));
    world.set_stress_settings(StressSettings {
        damage_per_overload: 1.0,
        max_iterations: 1,
        ..StressSettings::default()
    });
    world
        .add_destructible(family, single_node_asset(7))
        .unwrap();
    let anchor = world
        .connect_static_anchor(
            family,
            StaticAnchorConnectionDesc::new(static_anchor_desc(77, 0))
                .with_body_policy(StaticAnchorBodyPolicy::Fixed),
        )
        .unwrap();

    let initial = world.step_with_diagnostics().unwrap();

    assert!(initial.report.fracture_events.is_empty());
    assert!(initial.report.split_events.is_empty());
    assert_eq!(world.prestress_baseline_count_for_test(), 1);

    world.set_gravity(Vector::new(0.0, -2.0));
    let step = world.step_with_diagnostics().unwrap();

    assert!(step.report.stress_inputs.is_empty());
    assert_eq!(step.report.fracture_events.len(), 1);
    assert_eq!(
        step.report.fracture_events[0].target,
        FractureTarget::ExternalBond(anchor)
    );
    assert_eq!(step.report.fracture_events[0].source, DamageSource::Stress);
    assert_eq!(world.prestress_baseline_count_for_test(), 0);
}

#[test]
fn prestress_baseline_invalidates_after_static_anchor_change() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(Vector::new(0.0, -1.0));
    world.set_stress_settings(StressSettings {
        damage_per_overload: 1.0,
        max_iterations: 1,
        ..StressSettings::default()
    });
    world
        .add_destructible(family, single_node_asset(7))
        .unwrap();
    world
        .connect_static_anchor(
            family,
            StaticAnchorConnectionDesc::new(static_anchor_desc(77, 0))
                .with_body_policy(StaticAnchorBodyPolicy::Fixed),
        )
        .unwrap();

    let initial = world.step_with_diagnostics().unwrap();

    assert!(initial.report.fracture_events.is_empty());
    assert_eq!(world.prestress_baseline_count_for_test(), 1);
    let first_signature = world
        .prestress_baseline_signature_for_test(family)
        .expect("baseline captured");

    world
        .connect_static_anchor(
            family,
            StaticAnchorConnectionDesc::new(static_anchor_desc(78, 0))
                .with_body_policy(StaticAnchorBodyPolicy::Fixed),
        )
        .unwrap();

    assert_eq!(world.prestress_baseline_count_for_test(), 0);
    assert_eq!(world.prestress_baseline_signature_for_test(family), None);

    let recaptured = world.step_with_diagnostics().unwrap();

    assert!(recaptured.report.fracture_events.is_empty());
    assert!(recaptured.report.split_events.is_empty());
    assert_eq!(world.prestress_baseline_count_for_test(), 1);
    assert_ne!(
        world
            .prestress_baseline_signature_for_test(family)
            .expect("baseline recaptured"),
        first_signature
    );
}

#[test]
fn prestress_baseline_invalidates_after_stress_settings_change() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(Vector::new(0.0, -1.0));
    world.set_stress_settings(StressSettings {
        damage_per_overload: 1.0,
        max_iterations: 1,
        ..StressSettings::default()
    });
    world
        .add_destructible(family, single_node_asset(7))
        .unwrap();
    world
        .connect_static_anchor(
            family,
            StaticAnchorConnectionDesc::new(static_anchor_desc(77, 0))
                .with_body_policy(StaticAnchorBodyPolicy::Fixed),
        )
        .unwrap();

    let initial = world.step_with_diagnostics().unwrap();

    assert!(initial.report.fracture_events.is_empty());
    assert!(initial.report.split_events.is_empty());
    assert_eq!(world.prestress_baseline_count_for_test(), 1);

    world.set_stress_settings(StressSettings {
        damage_per_overload: 1.0,
        enable_gravity: false,
        max_iterations: 1,
        ..StressSettings::default()
    });

    assert_eq!(world.prestress_baseline_count_for_test(), 0);
    assert_eq!(world.prestress_baseline_signature_for_test(family), None);

    let recaptured = world.step_with_diagnostics().unwrap();

    assert!(recaptured.report.fracture_events.is_empty());
    assert!(recaptured.report.split_events.is_empty());
    assert_eq!(world.prestress_baseline_count_for_test(), 1);
    assert_eq!(world.stress_settings().enable_gravity, false);
}

fn world_with_prestress_baseline() -> (FxFamilyId, FxRapierWorld2D) {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    world.set_snapshot_mode(SnapshotMode::Normal);
    world.set_gravity(Vector::new(0.0, -1.0));
    world.set_stress_settings(StressSettings {
        damage_per_overload: 1.0,
        max_iterations: 1,
        ..StressSettings::default()
    });
    world
        .add_destructible(family, single_node_asset(7))
        .unwrap();
    world
        .connect_static_anchor(
            family,
            StaticAnchorConnectionDesc::new(static_anchor_desc(77, 0))
                .with_body_policy(StaticAnchorBodyPolicy::Fixed),
        )
        .unwrap();
    let initial = world.step_with_diagnostics().unwrap();
    assert!(initial.report.fracture_events.is_empty());
    assert!(initial.report.split_events.is_empty());
    assert_eq!(world.prestress_baseline_count_for_test(), 1);
    (family, world)
}

fn snapshot_with_prestress_baseline() -> crate::snapshot::FxRapierWorldSnapshot {
    let (_, world) = world_with_prestress_baseline();
    let (_, snapshot) = decode_world_snapshot(&world.snapshot().unwrap()).unwrap();
    assert_eq!(snapshot.prestress_baselines.len(), 1);
    snapshot
}

#[test]
fn snapshot_restore_prestress_baseline() {
    let (family, world) = world_with_prestress_baseline();
    let signature = world
        .prestress_baseline_signature_for_test(family)
        .expect("baseline captured");
    let load_count = world
        .prestress_baseline_load_count_for_test(family)
        .expect("baseline captured");

    let mut restored = FxRapierWorld2D::restore_snapshot(&world.snapshot().unwrap()).unwrap();

    assert_eq!(restored.prestress_baseline_count_for_test(), 1);
    assert_eq!(
        restored.prestress_baseline_signature_for_test(family),
        Some(signature)
    );
    assert_eq!(
        restored.prestress_baseline_load_count_for_test(family),
        Some(load_count)
    );

    let step = restored.step_with_diagnostics().unwrap();

    assert!(step.report.stress_inputs.is_empty());
    assert!(step.report.fracture_events.is_empty());
    assert!(step.report.split_events.is_empty());
    assert_eq!(restored.prestress_baseline_count_for_test(), 1);
    assert_eq!(
        restored.prestress_baseline_signature_for_test(family),
        Some(signature)
    );
    assert_eq!(
        restored.prestress_baseline_load_count_for_test(family),
        Some(load_count)
    );
}

#[test]
fn snapshot_restore_rejects_duplicate_prestress_baseline_family() {
    let mut snapshot = snapshot_with_prestress_baseline();
    snapshot
        .prestress_baselines
        .push(snapshot.prestress_baselines[0].clone());

    assert!(matches!(
        FxRapierWorld2D::restore_snapshot(&encode_world_snapshot(&snapshot)),
        Err(FxRapierError::Snapshot(
            FxRapierSnapshotError::StateMismatch("duplicate prestress baseline family")
        ))
    ));
}

#[test]
fn snapshot_restore_rejects_unknown_prestress_baseline_family() {
    let mut snapshot = snapshot_with_prestress_baseline();
    snapshot.prestress_baselines[0].family = FxFamilyId(99);

    assert!(matches!(
        FxRapierWorld2D::restore_snapshot(&encode_world_snapshot(&snapshot)),
        Err(FxRapierError::Snapshot(
            FxRapierSnapshotError::StateMismatch("prestress baseline unknown family")
        ))
    ));
}

#[test]
fn snapshot_restore_rejects_non_finite_prestress_baseline_gravity() {
    let mut snapshot = snapshot_with_prestress_baseline();
    snapshot.prestress_baselines[0].gravity[0] = 12_345.0;
    let mut bytes = encode_world_snapshot(&snapshot);
    replace_first_f32_bits(&mut bytes, 12_345.0, f32::NAN.to_bits());

    assert!(matches!(
        FxRapierWorld2D::restore_snapshot(&bytes),
        Err(FxRapierError::Snapshot(
            FxRapierSnapshotError::InvalidValue("prestress_baseline.gravity")
        ))
    ));
}

#[test]
fn snapshot_restore_rejects_non_finite_prestress_baseline_load() {
    let mut snapshot = snapshot_with_prestress_baseline();
    snapshot.prestress_baselines[0].loads[0].node_a_force[0] = 12_345.0;
    let mut bytes = encode_world_snapshot(&snapshot);
    replace_first_f32_bits(&mut bytes, 12_345.0, f32::NAN.to_bits());

    assert!(matches!(
        FxRapierWorld2D::restore_snapshot(&bytes),
        Err(FxRapierError::Snapshot(
            FxRapierSnapshotError::InvalidValue("prestress_baseline.node_a_force")
        ))
    ));
}

#[test]
fn snapshot_restore_rejects_prestress_baseline_topology_mismatch() {
    let mut snapshot = snapshot_with_prestress_baseline();
    snapshot.prestress_baselines[0].topology_signature ^= 1;

    assert!(matches!(
        FxRapierWorld2D::restore_snapshot(&encode_world_snapshot(&snapshot)),
        Err(FxRapierError::Snapshot(
            FxRapierSnapshotError::StateMismatch("prestress baseline topology mismatch")
        ))
    ));
}

#[test]
fn snapshot_restore_rejects_duplicate_prestress_baseline_target() {
    let mut snapshot = snapshot_with_prestress_baseline();
    let duplicate = snapshot.prestress_baselines[0].loads[0];
    snapshot.prestress_baselines[0].loads.push(duplicate);

    assert!(matches!(
        FxRapierWorld2D::restore_snapshot(&encode_world_snapshot(&snapshot)),
        Err(FxRapierError::Snapshot(
            FxRapierSnapshotError::StateMismatch("duplicate prestress baseline target")
        ))
    ));
}

#[test]
fn snapshot_restore_rejects_unknown_prestress_baseline_target() {
    let mut snapshot = snapshot_with_prestress_baseline();
    snapshot.prestress_baselines[0].loads[0].target =
        PrestressBaselineTargetSnapshot::ExternalBond(ExternalBondId(999));

    assert!(matches!(
        FxRapierWorld2D::restore_snapshot(&encode_world_snapshot(&snapshot)),
        Err(FxRapierError::Snapshot(
            FxRapierSnapshotError::StateMismatch("prestress baseline unknown target")
        ))
    ));
}

#[test]
fn snapshot_restore_rejects_missing_prestress_baseline_target() {
    let mut snapshot = snapshot_with_prestress_baseline();
    snapshot.prestress_baselines[0].loads.clear();

    assert!(matches!(
        FxRapierWorld2D::restore_snapshot(&encode_world_snapshot(&snapshot)),
        Err(FxRapierError::Snapshot(
            FxRapierSnapshotError::StateMismatch("prestress baseline missing target")
        ))
    ));
}

#[test]
fn joint_feedback_stress() {
    let mut world = FxRapierWorld2D::new();
    world
        .add_destructible(FxFamilyId(1), single_node_asset(3))
        .unwrap();
    let destructible = world.actor_handles(FxFamilyId(1), FxActorId(0)).unwrap();
    let (anchor, _) = add_fixed_box(
        &mut world,
        Vector::new(0.5, 1.5),
        Vector::new(0.1, 0.1),
        0.5,
    );
    world.insert_impulse_joint(destructible.body, anchor, FixedJointBuilder::new(), true);

    let mut feedback = None;
    for _ in 0..8 {
        let report = world.step().unwrap();
        feedback = report
            .joint_feedback
            .into_iter()
            .find(|feedback| feedback.impulse_magnitude > 0.0);
        if feedback.is_some() {
            break;
        }
    }
    let feedback = feedback.expect("nonzero impulse joint feedback after real step");
    assert_eq!(feedback.destructible.actor, FxActorId(0));
    assert!(feedback.impulse_magnitude > 0.0);
    assert_eq!(
        feedback.stress.source,
        fracture_core::DamageSource::JointFeedback
    );
}

#[test]
fn same_step_split_sync() {
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(Vector::ZERO);
    world.set_stress_settings(StressSettings {
        damage_per_overload: 2.0,
        max_fractures_per_frame: u16::MAX,
        ..StressSettings::default()
    });
    world
        .add_destructible(FxFamilyId(1), two_node_asset(5))
        .unwrap();
    let destructible = world.actor_handles(FxFamilyId(1), FxActorId(0)).unwrap();
    {
        let body = world.rigid_bodies_mut().get_mut(destructible.body).unwrap();
        body.set_position(
            Pose::from_parts(Vector::new(4.0, 3.0), Rotation::new(0.25)),
            true,
        );
        body.set_linvel(Vector::new(1.25, -6.0), true);
        body.set_angvel(1.5, true);
        body.enable_ccd(true);
    }
    let parent_body_before = destructible.body;
    add_fixed_box(
        &mut world,
        Vector::new(4.48, 2.55),
        Vector::new(0.45, 0.1),
        0.5,
    );

    let mut report = None;
    for _ in 0..8 {
        let step = world.step_with_diagnostics().unwrap();
        if !step.report.split_events.is_empty() {
            report = Some(step);
            break;
        }
    }
    let report = report.expect("same-step stress split");
    assert_eq!(report.report.split_events.len(), 1);
    assert_eq!(report.diagnostics.contact_impulse_readback_miss_count, 0);
    assert!(report.diagnostics.budget.unwrap().within_budget());
    assert!(
        report
            .diagnostics
            .stress_profiles
            .iter()
            .any(|profile| profile.input_count > 0 && profile.generated_commands_after_cap > 0)
    );
    assert_eq!(report.diagnostics.physics_sync.rebuilt_colliders, 1);
    assert_eq!(report.diagnostics.physics_sync.created_actor_bodies, 1);
    assert_eq!(report.diagnostics.physics_sync.removed_actor_bodies, 0);
    let event = &report.report.split_events[0];
    assert_eq!(event.kept_actor, FxActorId(0));
    assert_eq!(event.created_children, vec![FxActorId(1)]);

    let kept = world.actor_handles(FxFamilyId(1), event.kept_actor);
    let child = world.actor_handles(FxFamilyId(1), FxActorId(1));
    assert!(
        kept.is_some(),
        "kept actor has Rapier handles before return"
    );
    assert!(
        child.is_some(),
        "child actor has Rapier handles before return"
    );
    assert!(world.rigid_bodies().get(kept.unwrap().body).is_some());
    assert!(world.colliders().get(kept.unwrap().collider).is_some());
    assert!(world.rigid_bodies().get(child.unwrap().body).is_some());
    assert!(world.colliders().get(child.unwrap().collider).is_some());

    let kept = kept.unwrap();
    let child = child.unwrap();
    assert_eq!(kept.body, parent_body_before);
    let kept_body = world.rigid_bodies().get(kept.body).unwrap();
    let child_body = world.rigid_bodies().get(child.body).unwrap();
    assert!(kept_body.translation().x > 3.0);
    assert!(kept_body.linvel().length() > 0.0);
    assert!(kept_body.angvel().abs() > 0.0);
    assert!(kept_body.is_ccd_enabled());
    assert_ne!(kept.collider, destructible.collider);
    assert_ne!(child.body, kept.body);
    assert!((child_body.translation() - kept_body.translation()).length() > 0.2);
    assert!(child_body.linvel().length() > 0.0);
    assert!(child_body.angvel().abs() > 0.0);
}

#[test]
fn split_remaps_impulse_joint_endpoint_to_child_fragment() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(Vector::ZERO);
    world
        .add_destructible(family, four_node_line_asset())
        .unwrap();
    let parent = world.actor_handles(family, FxActorId(0)).unwrap();
    let anchor =
        world.insert_rigid_body(RigidBodyBuilder::fixed().translation(Vector::new(3.5, 0.5)));
    let old_joint = world.insert_impulse_joint(
        parent.body,
        anchor,
        FixedJointBuilder::new()
            .local_anchor1(Vector::new(1.5, 0.0))
            .local_anchor2(Vector::ZERO),
        true,
    );

    let report = world
        .apply_fracture_commands_to_family(
            family,
            &[break_bond_command(0, family, FxActorId(0), BondId(2))],
        )
        .unwrap();
    let split = &report.report.split_events;

    assert_eq!(split.len(), 1);
    assert_eq!(split[0].created_children, vec![FxActorId(1)]);
    assert_eq!(report.report.impulse_joint_handle_replacements.len(), 1);
    assert_eq!(
        report
            .diagnostics
            .physics_sync
            .impulse_joint_handle_replacements,
        report.report.impulse_joint_handle_replacements
    );
    let replacement = report.report.impulse_joint_handle_replacements[0];
    assert_eq!(replacement.old, old_joint);
    assert!(!world.impulse_joints().contains(old_joint));
    assert!(world.impulse_joints().contains(replacement.new));
    let kept = world.actor_handles(family, FxActorId(0)).unwrap();
    let child = world.actor_handles(family, FxActorId(1)).unwrap();
    assert_eq!(kept.body, parent.body);
    let joints = world.impulse_joints().iter().collect::<Vec<_>>();
    assert_eq!(joints.len(), 1);
    let (new_joint, joint) = joints[0];
    assert_eq!(new_joint, replacement.new);
    assert_eq!(joint.body1, child.body);
    assert_eq!(joint.body2, anchor);
    assert_ne!(joint.body1, kept.body);
    assert_vector_close(joint.data.local_anchor1(), Vector::ZERO);
    assert_vector_close(joint.data.local_anchor2(), Vector::ZERO);
}

#[test]
fn split_child_inherits_actual_parent_snapshot_in_multi_actor_family() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(Vector::ZERO);
    world
        .add_destructible(family, four_node_line_asset())
        .unwrap();

    let first_split = world
        .fracture_and_sync_for_test(
            family,
            &[break_bond_command(0, family, FxActorId(0), BondId(1))],
        )
        .unwrap();
    assert_eq!(first_split.len(), 1);
    assert_eq!(first_split[0].created_children, vec![FxActorId(1)]);

    let actor0 = world.actor_handles(family, FxActorId(0)).unwrap();
    let actor1 = world.actor_handles(family, FxActorId(1)).unwrap();
    {
        let body0 = world.rigid_bodies_mut().get_mut(actor0.body).unwrap();
        body0.set_position(
            Pose::from_parts(Vector::new(-12.0, 4.0), Rotation::new(0.35)),
            true,
        );
        body0.set_linvel(Vector::new(1.0, 2.0), true);
        body0.set_angvel(0.75, true);
        body0.enable_ccd(false);

        let body1 = world.rigid_bodies_mut().get_mut(actor1.body).unwrap();
        body1.set_position(
            Pose::from_parts(Vector::new(23.0, -7.0), Rotation::new(-0.6)),
            true,
        );
        body1.set_linvel(Vector::new(-4.0, 3.5), true);
        body1.set_angvel(-2.25, true);
        body1.enable_ccd(true);
    }
    let parent1_position = *world.rigid_bodies().get(actor1.body).unwrap().position();
    let parent1_linvel = world.rigid_bodies().get(actor1.body).unwrap().linvel();
    let parent1_angvel = world.rigid_bodies().get(actor1.body).unwrap().angvel();

    let second_split = world
        .fracture_and_sync_for_test(
            family,
            &[break_bond_command(1, family, FxActorId(1), BondId(2))],
        )
        .unwrap();
    assert_eq!(second_split.len(), 1);
    assert_eq!(second_split[0].parent_actor, FxActorId(1));
    assert_eq!(second_split[0].created_children, vec![FxActorId(2)]);

    let kept1 = world.actor_handles(family, FxActorId(1)).unwrap();
    let child2 = world.actor_handles(family, FxActorId(2)).unwrap();
    assert_eq!(kept1.body, actor1.body);
    let kept1_body = world.rigid_bodies().get(kept1.body).unwrap();
    assert_vector_close(kept1_body.translation(), parent1_position.translation);

    let child2_body = world.rigid_bodies().get(child2.body).unwrap();
    let expected_child2_translation =
        parent1_position.translation + parent1_position.rotation * Vector::new(0.5, 0.0);
    assert_vector_close(child2_body.translation(), expected_child2_translation);
    assert_vector_close(child2_body.linvel(), parent1_linvel);
    assert!((child2_body.angvel() - parent1_angvel).abs() < 0.0001);
    assert!(child2_body.is_ccd_enabled());

    let actor0_position = *world.rigid_bodies().get(actor0.body).unwrap().position();
    let actor0_linvel = world.rigid_bodies().get(actor0.body).unwrap().linvel();
    let actor0_angvel = world.rigid_bodies().get(actor0.body).unwrap().angvel();
    let third_split = world
        .fracture_and_sync_for_test(
            family,
            &[break_bond_command(2, family, FxActorId(0), BondId(0))],
        )
        .unwrap();
    assert_eq!(third_split.len(), 1);
    assert_eq!(third_split[0].parent_actor, FxActorId(0));
    assert_eq!(third_split[0].created_children, vec![FxActorId(3)]);

    let kept0 = world.actor_handles(family, FxActorId(0)).unwrap();
    let child3 = world.actor_handles(family, FxActorId(3)).unwrap();
    assert_eq!(kept0.body, actor0.body);
    let kept0_body = world.rigid_bodies().get(kept0.body).unwrap();
    assert_vector_close(kept0_body.translation(), actor0_position.translation);

    let child3_body = world.rigid_bodies().get(child3.body).unwrap();
    let expected_child3_translation =
        actor0_position.translation + actor0_position.rotation * Vector::new(-0.5, 0.0);
    assert_vector_close(child3_body.translation(), expected_child3_translation);
    assert_vector_close(child3_body.linvel(), actor0_linvel);
    assert!((child3_body.angvel() - actor0_angvel).abs() < 0.0001);
    assert!(!child3_body.is_ccd_enabled());
}

#[test]
fn dirty_sync_report_skips_untouched_actor_colliders() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    world.set_lod_settings(ColliderLodSettings::small_debris_box(4, 1));
    world
        .add_destructible(family, four_node_line_asset())
        .unwrap();
    world
        .fracture_and_sync_for_test(
            family,
            &[break_bond_command(0, family, FxActorId(0), BondId(1))],
        )
        .unwrap();
    let untouched_before = world.actor_handles(family, FxActorId(1)).unwrap();

    let (split, sync) = world
        .fracture_and_sync_report_for_test(
            family,
            &[break_bond_command(1, family, FxActorId(0), BondId(0))],
        )
        .unwrap();
    let untouched_after = world.actor_handles(family, FxActorId(1)).unwrap();

    assert_eq!(split.len(), 1);
    assert_eq!(sync.rebuilt_colliders, 1);
    assert_eq!(sync.created_actor_bodies, 1);
    assert_eq!(sync.removed_actor_bodies, 0);
    assert_eq!(sync.untouched_actor_count, 1);
    assert_eq!(sync.primitive_lod_replacements, 2);
    assert_eq!(untouched_after.body, untouched_before.body);
    assert_eq!(untouched_after.collider, untouched_before.collider);
}

#[test]
fn split_child_missing_parent_snapshot_returns_explicit_error() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(Vector::ZERO);
    world.add_destructible(family, two_node_asset(5)).unwrap();

    let err = world
        .fracture_and_sync_without_parent_snapshot_for_test(
            family,
            &[break_bond_command(0, family, FxActorId(0), BondId(0))],
            FxActorId(0),
        )
        .unwrap_err();

    assert_eq!(
        err,
        FxRapierError::MissingSplitParentSnapshot {
            family,
            parent: FxActorId(0),
            child: FxActorId(1),
        }
    );
    assert!(
        world.actor_handles(family, FxActorId(1)).is_none(),
        "split child must not be placed through the generic no-inherited-snapshot path"
    );
}

#[test]
fn adapter_multi_family_split_events_are_reported_in_family_order() {
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(Vector::ZERO);
    world
        .add_destructible(FxFamilyId(20), two_node_asset(5))
        .unwrap();
    world
        .add_destructible(FxFamilyId(10), two_node_asset(5))
        .unwrap();

    assert_eq!(
        world.family_ids_for_test(),
        vec![FxFamilyId(10), FxFamilyId(20)]
    );

    let split_events = world
        .fracture_all_and_sync_for_test(&[
            (
                FxFamilyId(20),
                vec![break_bond_command(
                    0,
                    FxFamilyId(20),
                    FxActorId(0),
                    BondId(0),
                )],
            ),
            (
                FxFamilyId(10),
                vec![break_bond_command(
                    0,
                    FxFamilyId(10),
                    FxActorId(0),
                    BondId(0),
                )],
            ),
        ])
        .unwrap();

    assert_eq!(
        split_events
            .iter()
            .map(|event| event.family)
            .collect::<Vec<_>>(),
        vec![FxFamilyId(10), FxFamilyId(20)]
    );
}

#[test]
fn destructible_vs_destructible_generates_two_readbacks() {
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(Vector::ZERO);
    world
        .add_destructible(FxFamilyId(1), single_node_asset(7))
        .unwrap();
    world
        .add_destructible(FxFamilyId(2), single_node_asset(8))
        .unwrap();
    let a = world.actor_handles(FxFamilyId(1), FxActorId(0)).unwrap();
    let b = world.actor_handles(FxFamilyId(2), FxActorId(0)).unwrap();
    world
        .rigid_bodies_mut()
        .get_mut(a.body)
        .unwrap()
        .set_linvel(Vector::new(2.0, 0.0), true);
    world
        .rigid_bodies_mut()
        .get_mut(b.body)
        .unwrap()
        .set_position(Pose::from_translation(Vector::new(0.85, 0.5)), true);
    world
        .rigid_bodies_mut()
        .get_mut(b.body)
        .unwrap()
        .set_linvel(Vector::new(-2.0, 0.0), true);

    let mut records = Vec::new();
    for _ in 0..8 {
        let report = world.step().unwrap();
        records = report
            .contact_impulses
            .into_iter()
            .filter(|input| input.impulse.normal_impulse > 0.0)
            .collect();
        let families = records
            .iter()
            .map(|input| input.stress.order_key.family_id)
            .collect::<BTreeSet<_>>();
        if families.contains(&FxFamilyId(1)) && families.contains(&FxFamilyId(2)) {
            break;
        }
    }
    assert!(records.iter().all(|input| !input.impulse.used_fallback));
    assert!(
        records
            .iter()
            .any(|input| input.stress.order_key.family_id == FxFamilyId(1))
    );
    assert!(
        records
            .iter()
            .any(|input| input.stress.order_key.family_id == FxFamilyId(2))
    );
}

#[cfg(feature = "deterministic-replay")]
#[test]
fn deterministic_snapshot_restores_lod_disabled() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    world.set_snapshot_mode(SnapshotMode::Deterministic);
    world
        .add_destructible(family, two_voxel_single_node_asset(7))
        .unwrap();

    assert_eq!(world.lod_settings(), ColliderLodSettings::disabled());
    let restored = FxRapierWorld2D::restore_snapshot(&world.snapshot().unwrap()).unwrap();
    assert_eq!(restored.snapshot_mode(), SnapshotMode::Deterministic);
    assert_eq!(restored.lod_settings(), ColliderLodSettings::disabled());
}

#[cfg(feature = "deterministic-replay")]
#[test]
fn deterministic_snapshot_rejects_enabled_lod() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    world.set_snapshot_mode(SnapshotMode::Deterministic);
    world.set_lod_settings(ColliderLodSettings::small_debris_box(4, 1));
    world
        .add_destructible(family, two_voxel_single_node_asset(7))
        .unwrap();

    assert!(matches!(
        world.snapshot(),
        Err(FxRapierError::Snapshot(
            FxRapierSnapshotError::InvalidValue("deterministic lod")
        ))
    ));
}

#[cfg(feature = "deterministic-replay")]
#[test]
fn rapier_checkpoint_restores_into_real_world() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    set_deterministic_replay_mode(&mut world);
    world.set_gravity(Vector::new(0.0, -1.0));
    world.integration_parameters_mut().dt = 1.0 / 120.0;
    world.integration_parameters_mut().min_ccd_dt = 1.0 / 3600.0;
    world.integration_parameters_mut().warmstart_coefficient = 0.73;
    world.integration_parameters_mut().num_solver_iterations = 6;
    world.integration_parameters_mut().max_ccd_substeps = 2;
    world.set_contact_material_properties(
        5,
        ContactMaterialProperties {
            friction: 0.75,
            restitution: 0.25,
        },
    );
    world
        .add_destructible(family, four_node_line_asset())
        .unwrap();
    world
        .connect_static_anchor(
            family,
            StaticAnchorConnectionDesc::new(static_anchor_desc(7, 0))
                .with_body_policy(StaticAnchorBodyPolicy::Fixed),
        )
        .unwrap();
    let handles = world.actor_handles(family, FxActorId(0)).unwrap();
    {
        let body = world.rigid_bodies_mut().get_mut(handles.body).unwrap();
        body.set_position(Pose::from_translation(Vector::new(3.0, 4.0)), true);
        body.set_linvel(Vector::new(1.0, -2.0), true);
        body.set_angvel(0.5, true);
        body.enable_ccd(true);
        body.set_soft_ccd_prediction(0.125);
    }

    let digest = world.family(family).unwrap().deterministic_state_digest();
    let bytes = world.snapshot().unwrap();
    let mut restored = FxRapierWorld2D::restore_snapshot(&bytes).unwrap();
    assert_eq!(
        restored
            .family(family)
            .unwrap()
            .deterministic_state_digest(),
        digest
    );
    let restored_handles = restored.actor_handles(family, FxActorId(0)).unwrap();
    let restored_body = restored.rigid_bodies().get(restored_handles.body).unwrap();
    assert!(restored_body.is_fixed());
    assert!(restored_body.is_ccd_enabled());
    assert_scalar_close(restored_body.soft_ccd_prediction(), 0.125);
    assert_scalar_close(restored.integration_parameters().dt, 1.0 / 120.0);
    assert_scalar_close(restored.integration_parameters().min_ccd_dt, 1.0 / 3600.0);
    assert_scalar_close(
        restored.integration_parameters().warmstart_coefficient,
        0.73,
    );
    assert_eq!(restored.integration_parameters().num_solver_iterations, 6);
    assert_eq!(restored.integration_parameters().max_ccd_substeps, 2);
    restored.step().unwrap();
}

#[cfg(feature = "deterministic-replay")]
#[test]
fn deterministic_replay_after_split_checkpoint_identical() {
    let family = FxFamilyId(1);
    let mut original = FxRapierWorld2D::new();
    set_deterministic_replay_mode(&mut original);
    original.set_gravity(Vector::ZERO);
    original.integration_parameters_mut().dt = 1.0 / 120.0;
    original.integration_parameters_mut().min_ccd_dt = 1.0 / 4800.0;
    original.integration_parameters_mut().warmstart_coefficient = 0.7;
    original.integration_parameters_mut().num_solver_iterations = 7;
    original.integration_parameters_mut().max_ccd_substeps = 3;
    original
        .add_destructible(family, four_node_line_asset())
        .unwrap();
    original
        .fracture_and_sync_for_test(
            family,
            &[break_bond_command(0, family, FxActorId(0), BondId(0))],
        )
        .unwrap();
    for _ in 0..10 {
        original.step().unwrap();
    }
    let checkpoint = original.snapshot().unwrap();
    let mut restored = FxRapierWorld2D::restore_snapshot(&checkpoint).unwrap();
    assert_scalar_close(restored.integration_parameters().dt, 1.0 / 120.0);
    assert_scalar_close(restored.integration_parameters().min_ccd_dt, 1.0 / 4800.0);
    assert_scalar_close(restored.integration_parameters().warmstart_coefficient, 0.7);
    assert_eq!(restored.integration_parameters().num_solver_iterations, 7);
    assert_eq!(restored.integration_parameters().max_ccd_substeps, 3);
    let commands = vec![FxRapierReplayCommand {
        tick: 25,
        stable_order: 0,
        family,
        command: break_bond_command(25, family, FxActorId(0), BondId(2)),
    }];

    let original_trace = original.run_replay_trace(1000, &commands).unwrap();
    let restored_trace = restored.run_replay_trace(1000, &commands).unwrap();
    assert_eq!(original_trace, restored_trace);
    assert!(
        original_trace
            .ticks
            .iter()
            .flat_map(|tick| tick.split_events.iter())
            .any(|event| !event.created_children.is_empty())
    );
    assert_eq!(
        original
            .family(family)
            .unwrap()
            .deterministic_state_digest(),
        restored
            .family(family)
            .unwrap()
            .deterministic_state_digest()
    );
    assert_eq!(
        original.sorted_actor_body_trace(),
        restored.sorted_actor_body_trace()
    );
}

#[test]
fn snapshot_restore_full_rapier_public_bodies_colliders_joints() {
    let mut world = FxRapierWorld2D::new();
    world.set_snapshot_mode(SnapshotMode::Normal);
    world
        .add_destructible(FxFamilyId(1), single_node_asset(7))
        .unwrap();
    let managed = world.actor_handles(FxFamilyId(1), FxActorId(0)).unwrap();
    let (public_body, public_collider) = add_fixed_box(
        &mut world,
        Vector::new(4.0, 0.0),
        Vector::new(0.5, 0.5),
        1.0,
    );
    let joint =
        world.insert_impulse_joint(managed.body, public_body, FixedJointBuilder::new(), true);

    let restored = FxRapierWorld2D::restore_snapshot(&world.snapshot().unwrap()).unwrap();
    assert!(restored.rigid_bodies().get(public_body).is_some());
    assert!(restored.colliders().get(public_collider).is_some());
    assert!(restored.impulse_joints().get(joint).is_some());
    assert!(
        restored
            .actor_handles(FxFamilyId(1), FxActorId(0))
            .is_some()
    );
}

#[test]
fn snapshot_restore_runtime_lod_keeps_primitive_metadata() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    world.set_snapshot_mode(SnapshotMode::Normal);
    world.set_lod_settings(ColliderLodSettings::small_debris_box(4, 1));
    world
        .add_destructible(family, two_voxel_single_node_asset(7))
        .unwrap();
    let handles = world.actor_handles(family, FxActorId(0)).unwrap();
    assert_eq!(
        world
            .contact_registry_snapshot()
            .collider_voxels
            .get(&collider_key(handles.collider))
            .unwrap()
            .len(),
        1
    );

    let mut restored = FxRapierWorld2D::restore_snapshot(&world.snapshot().unwrap()).unwrap();
    assert_eq!(
        restored.lod_settings(),
        ColliderLodSettings::small_debris_box(4, 1)
    );
    let restored_handles = restored.actor_handles(family, FxActorId(0)).unwrap();
    assert_eq!(
        restored
            .contact_registry_snapshot()
            .collider_voxels
            .get(&collider_key(restored_handles.collider))
            .unwrap()
            .len(),
        1
    );
    restored.step().unwrap();
}

#[test]
fn snapshot_restore_full_integration_parameters() {
    let mut world = FxRapierWorld2D::new();
    world.set_snapshot_mode(SnapshotMode::Normal);
    world.integration_parameters_mut().dt = 1.0 / 123.0;
    world.integration_parameters_mut().min_ccd_dt = 1.0 / 9876.0;
    world
        .integration_parameters_mut()
        .contact_softness
        .natural_frequency = 42.0;
    world
        .integration_parameters_mut()
        .contact_softness
        .damping_ratio = 0.37;
    world.integration_parameters_mut().warmstart_coefficient = 0.61;
    world.integration_parameters_mut().length_unit = 2.5;
    world
        .integration_parameters_mut()
        .normalized_allowed_linear_error = 0.004;
    world
        .integration_parameters_mut()
        .normalized_max_corrective_velocity = 12.5;
    world
        .integration_parameters_mut()
        .normalized_prediction_distance = 0.071;
    world.integration_parameters_mut().num_solver_iterations = 9;
    world
        .integration_parameters_mut()
        .num_internal_pgs_iterations = 4;
    world
        .integration_parameters_mut()
        .num_internal_stabilization_iterations = 3;
    world.integration_parameters_mut().min_island_size = 8;
    world.integration_parameters_mut().max_ccd_substeps = 5;
    world
        .add_destructible(FxFamilyId(1), single_node_asset(7))
        .unwrap();

    let restored = FxRapierWorld2D::restore_snapshot(&world.snapshot().unwrap()).unwrap();
    assert_eq!(
        restored.integration_parameters(),
        world.integration_parameters()
    );
}

#[test]
fn snapshot_restore_stress_settings() {
    let mut world = FxRapierWorld2D::new();
    world.set_snapshot_mode(SnapshotMode::Normal);
    world.set_stress_settings(StressSettings {
        tension_limit_scale: 0.5,
        shear_limit_scale: 0.25,
        compression_limit_scale: 0.75,
        compression_damage_mode: CompressionDamageMode2D::Break,
        damage_per_overload: 3.0,
        max_fractures_per_frame: 2,
        max_iterations: 6,
        convergence_epsilon: 0.125,
        enable_gravity: false,
    });
    world
        .add_destructible(FxFamilyId(1), two_node_asset(7))
        .unwrap();

    let mut restored = FxRapierWorld2D::restore_snapshot(&world.snapshot().unwrap()).unwrap();
    assert_eq!(restored.stress_settings(), world.stress_settings());
    restored.step().unwrap();
}

#[test]
fn snapshot_restore_quick_impact_settings_and_hardness_round_trip() {
    let settings = quick_impact_settings(0.01, 0.02, 0.01, 0.02);
    let (world, _) = quick_impact_wall_world(settings, 8.0);
    world.set_material_impact_hardness(7, 3.0);
    let (_, snapshot) = decode_world_snapshot(&world.snapshot().unwrap()).unwrap();

    assert_eq!(snapshot.quick_impact_settings, settings);
    assert_eq!(snapshot.material_impact_hardness, vec![(7, 3.0)]);

    let mut restored =
        FxRapierWorld2D::restore_snapshot(&encode_world_snapshot(&snapshot)).unwrap();
    assert_eq!(restored.quick_impact_settings(), settings);
    assert_scalar_close(restored.material_impact_hardness(7), 3.0);

    let step = restored.step_with_diagnostics().unwrap();

    assert!(
        step.report
            .quick_impacts
            .iter()
            .any(|input| input.impact.estimate.hardness == 3.0)
    );
}

#[test]
fn fracture_field_snapshot_restore_preserves_queued_direct_damage() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(Vector::ZERO);
    world.add_destructible(family, two_node_asset(7)).unwrap();

    let field = FractureField2D::direct_damage(Vec2::new(1.5, 0.5), 0.25, 2.0)
        .with_family(family)
        .with_effective_length_loss(0.75)
        .with_source(DamageSource::ContactImpulse);
    world.queue_fracture_field(field);

    let (_, snapshot) = decode_world_snapshot(&world.snapshot().unwrap()).unwrap();
    assert_eq!(snapshot.queued_fracture_fields, vec![field]);

    let mut restored =
        FxRapierWorld2D::restore_snapshot(&encode_world_snapshot(&snapshot)).unwrap();
    let step = restored.step_with_diagnostics().unwrap();

    assert_eq!(step.report.fracture_field_effects.len(), 1);
    assert_eq!(step.report.fracture_field_effects[0].family, family);
    assert_eq!(
        step.report.fracture_field_effects[0].mode,
        FractureFieldMode::DirectDamage
    );
    assert_eq!(step.report.fracture_events.len(), 1);
    assert_eq!(
        step.report.fracture_events[0].source,
        DamageSource::ContactImpulse
    );
    assert_eq!(step.report.split_events.len(), 1);
}

#[test]
fn fracture_field_snapshot_restore_rejects_invalid_queued_field() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(Vector::ZERO);
    world.add_destructible(family, two_node_asset(7)).unwrap();
    world.queue_fracture_field(
        FractureField2D::direct_damage(Vec2::new(1.5, 0.5), 0.25, 2.0).with_family(family),
    );
    let (_, mut snapshot) = decode_world_snapshot(&world.snapshot().unwrap()).unwrap();
    snapshot.queued_fracture_fields[0].radius = -1.0;

    assert!(matches!(
        FxRapierWorld2D::restore_snapshot(&encode_world_snapshot(&snapshot)),
        Err(FxRapierError::Snapshot(
            FxRapierSnapshotError::InvalidValue("queued fracture field")
        ))
    ));
}

#[test]
fn snapshot_restore_rejects_invalid_quick_impact_settings() {
    let mut world = FxRapierWorld2D::new();
    world.set_snapshot_mode(SnapshotMode::Normal);
    world
        .add_destructible(FxFamilyId(1), two_node_asset(7))
        .unwrap();
    let (_, mut snapshot) = decode_world_snapshot(&world.snapshot().unwrap()).unwrap();
    snapshot.quick_impact_settings.softened_friction_scale = 1.5;

    assert!(matches!(
        FxRapierWorld2D::restore_snapshot(&encode_world_snapshot(&snapshot)),
        Err(FxRapierError::Snapshot(
            FxRapierSnapshotError::InvalidValue("quick impact settings")
        ))
    ));
}

#[test]
fn snapshot_restore_rejects_invalid_material_impact_hardness() {
    let mut world = FxRapierWorld2D::new();
    world.set_snapshot_mode(SnapshotMode::Normal);
    world
        .add_destructible(FxFamilyId(1), two_node_asset(7))
        .unwrap();
    let (_, mut snapshot) = decode_world_snapshot(&world.snapshot().unwrap()).unwrap();
    snapshot.material_impact_hardness.push((7, -0.1));

    assert!(matches!(
        FxRapierWorld2D::restore_snapshot(&encode_world_snapshot(&snapshot)),
        Err(FxRapierError::Snapshot(
            FxRapierSnapshotError::InvalidValue("material impact hardness")
        ))
    ));
}

#[test]
fn snapshot_restore_rejects_invalid_compression_limit_scale() {
    let mut world = FxRapierWorld2D::new();
    world.set_snapshot_mode(SnapshotMode::Normal);
    world
        .add_destructible(FxFamilyId(1), two_node_asset(7))
        .unwrap();
    let (_, mut snapshot) = decode_world_snapshot(&world.snapshot().unwrap()).unwrap();
    snapshot.stress.compression_limit_scale = -0.5;

    assert!(matches!(
        FxRapierWorld2D::restore_snapshot(&encode_world_snapshot(&snapshot)),
        Err(FxRapierError::Snapshot(
            FxRapierSnapshotError::InvalidValue("stress.compression_limit_scale")
        ))
    ));
}

#[test]
fn snapshot_restore_rejects_invalid_compression_damage_mode() {
    let mut world = FxRapierWorld2D::new();
    world.set_snapshot_mode(SnapshotMode::Normal);
    world
        .add_destructible(FxFamilyId(1), two_node_asset(7))
        .unwrap();
    let mut bytes = world.snapshot().unwrap();
    const HEADER_LEN: usize = 34;
    const STRESS_PAYLOAD_OFFSET: usize = 8 + 8 + (9 * 4) + (5 * 4);
    const COMPRESSION_MODE_STRESS_OFFSET: usize = 4 + 4 + 4;
    bytes[HEADER_LEN + STRESS_PAYLOAD_OFFSET + COMPRESSION_MODE_STRESS_OFFSET] = 99;
    rewrite_snapshot_checksum(&mut bytes);

    assert!(matches!(
        FxRapierWorld2D::restore_snapshot(&bytes),
        Err(FxRapierError::Snapshot(
            FxRapierSnapshotError::InvalidValue("stress.compression_damage_mode")
        ))
    ));
}

#[test]
fn snapshot_restore_rejects_duplicate_family() {
    let mut world = FxRapierWorld2D::new();
    world.set_snapshot_mode(SnapshotMode::Normal);
    world
        .add_destructible(FxFamilyId(1), single_node_asset(7))
        .unwrap();
    let (_, mut snapshot) = decode_world_snapshot(&world.snapshot().unwrap()).unwrap();
    snapshot.families.push(snapshot.families[0].clone());

    assert!(matches!(
        FxRapierWorld2D::restore_snapshot(&encode_world_snapshot(&snapshot)),
        Err(FxRapierError::Snapshot(
            FxRapierSnapshotError::StateMismatch("duplicate family")
        ))
    ));
}

#[test]
fn snapshot_restore_rejects_stale_body_mapping() {
    let mut world = FxRapierWorld2D::new();
    world.set_snapshot_mode(SnapshotMode::Normal);
    world
        .add_destructible(FxFamilyId(1), single_node_asset(7))
        .unwrap();
    let (_, mut snapshot) = decode_world_snapshot(&world.snapshot().unwrap()).unwrap();
    snapshot.actor_physics[0].body_handle = (123_456, 0);

    assert!(matches!(
        FxRapierWorld2D::restore_snapshot(&encode_world_snapshot(&snapshot)),
        Err(FxRapierError::Snapshot(
            FxRapierSnapshotError::StateMismatch("actor references missing body")
        ))
    ));
}

#[test]
fn snapshot_restore_rejects_stale_collider_mapping() {
    let mut world = FxRapierWorld2D::new();
    world.set_snapshot_mode(SnapshotMode::Normal);
    world
        .add_destructible(FxFamilyId(1), single_node_asset(7))
        .unwrap();
    let (_, mut snapshot) = decode_world_snapshot(&world.snapshot().unwrap()).unwrap();
    snapshot.actor_physics[0].collider_handle = (123_456, 0);

    assert!(matches!(
        FxRapierWorld2D::restore_snapshot(&encode_world_snapshot(&snapshot)),
        Err(FxRapierError::Snapshot(
            FxRapierSnapshotError::StateMismatch("actor references missing collider")
        ))
    ));
}

#[test]
fn snapshot_restore_rejects_collider_parent_mismatch() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    world.set_snapshot_mode(SnapshotMode::Normal);
    world
        .add_destructible(family, single_node_asset(7))
        .unwrap();
    let (_, public_collider) = add_fixed_box(
        &mut world,
        Vector::new(4.0, 0.0),
        Vector::new(0.5, 0.5),
        1.0,
    );
    let (_, mut snapshot) = decode_world_snapshot(&world.snapshot().unwrap()).unwrap();
    let public_key = public_collider.into_raw_parts();
    snapshot.actor_physics[0].collider_handle = public_key;
    snapshot.collider_actors[0].collider_handle = public_key;
    snapshot.collider_voxels[0].collider_handle = public_key;

    assert!(matches!(
        FxRapierWorld2D::restore_snapshot(&encode_world_snapshot(&snapshot)),
        Err(FxRapierError::Snapshot(
            FxRapierSnapshotError::StateMismatch("actor collider parent mismatch")
        ))
    ));
}

#[test]
fn snapshot_restore_rejects_duplicate_collider_voxel_subshape() {
    let mut world = FxRapierWorld2D::new();
    world.set_snapshot_mode(SnapshotMode::Normal);
    world
        .add_destructible(FxFamilyId(1), single_node_asset(7))
        .unwrap();
    let (_, mut snapshot) = decode_world_snapshot(&world.snapshot().unwrap()).unwrap();
    let duplicate = snapshot.collider_voxels[0].voxels[0].clone();
    snapshot.collider_voxels[0].voxels.push(duplicate);

    assert!(matches!(
        FxRapierWorld2D::restore_snapshot(&encode_world_snapshot(&snapshot)),
        Err(FxRapierError::Snapshot(
            FxRapierSnapshotError::StateMismatch("collider voxel metadata mismatch")
        ))
    ));
}

#[test]
fn snapshot_restore_rejects_unknown_collider_voxel_node() {
    let mut world = FxRapierWorld2D::new();
    world.set_snapshot_mode(SnapshotMode::Normal);
    world
        .add_destructible(FxFamilyId(1), single_node_asset(7))
        .unwrap();
    let (_, mut snapshot) = decode_world_snapshot(&world.snapshot().unwrap()).unwrap();
    snapshot.collider_voxels[0].voxels[0].node = SupportNodeId(99);

    assert!(matches!(
        FxRapierWorld2D::restore_snapshot(&encode_world_snapshot(&snapshot)),
        Err(FxRapierError::Snapshot(
            FxRapierSnapshotError::StateMismatch("collider voxel metadata mismatch")
        ))
    ));
}

#[test]
fn snapshot_restore_rejects_wrong_collider_voxel_material() {
    let mut world = FxRapierWorld2D::new();
    world.set_snapshot_mode(SnapshotMode::Normal);
    world
        .add_destructible(FxFamilyId(1), single_node_asset(7))
        .unwrap();
    let (_, mut snapshot) = decode_world_snapshot(&world.snapshot().unwrap()).unwrap();
    snapshot.collider_voxels[0].voxels[0].contact_material = 8;

    assert!(matches!(
        FxRapierWorld2D::restore_snapshot(&encode_world_snapshot(&snapshot)),
        Err(FxRapierError::Snapshot(
            FxRapierSnapshotError::StateMismatch("collider voxel metadata mismatch")
        ))
    ));
}

#[test]
fn snapshot_restore_rejects_missing_static_anchor_baseline() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    world.set_snapshot_mode(SnapshotMode::Normal);
    world
        .add_destructible(family, single_node_asset(7))
        .unwrap();
    world
        .connect_static_anchor(
            family,
            StaticAnchorConnectionDesc::new(static_anchor_desc(7, 0))
                .with_body_policy(StaticAnchorBodyPolicy::Fixed),
        )
        .unwrap();
    let (_, mut snapshot) = decode_world_snapshot(&world.snapshot().unwrap()).unwrap();
    snapshot.static_anchor_body_baselines.clear();

    assert!(matches!(
        FxRapierWorld2D::restore_snapshot(&encode_world_snapshot(&snapshot)),
        Err(FxRapierError::Snapshot(
            FxRapierSnapshotError::StateMismatch(
                "static anchor applied policy/baseline key mismatch"
            )
        ))
    ));
}

#[test]
fn snapshot_restore_rejects_stale_static_anchor_baseline() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    world.set_snapshot_mode(SnapshotMode::Normal);
    world
        .add_destructible(family, single_node_asset(7))
        .unwrap();
    world
        .connect_static_anchor(
            family,
            StaticAnchorConnectionDesc::new(static_anchor_desc(7, 0))
                .with_body_policy(StaticAnchorBodyPolicy::Fixed),
        )
        .unwrap();
    let (_, mut snapshot) = decode_world_snapshot(&world.snapshot().unwrap()).unwrap();
    snapshot.applied_static_anchor_policies.clear();

    assert!(matches!(
        FxRapierWorld2D::restore_snapshot(&encode_world_snapshot(&snapshot)),
        Err(FxRapierError::Snapshot(
            FxRapierSnapshotError::StateMismatch(
                "static anchor applied policy/baseline key mismatch"
            )
        ))
    ));
}

#[test]
fn snapshot_restore_rejects_static_anchor_body_type_mismatch() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    world.set_snapshot_mode(SnapshotMode::Normal);
    world
        .add_destructible(family, single_node_asset(7))
        .unwrap();
    world
        .connect_static_anchor(
            family,
            StaticAnchorConnectionDesc::new(static_anchor_desc(7, 0))
                .with_body_policy(StaticAnchorBodyPolicy::Fixed),
        )
        .unwrap();
    let (_, mut snapshot) = decode_world_snapshot(&world.snapshot().unwrap()).unwrap();
    snapshot.applied_static_anchor_policies[0].policy =
        StaticAnchorBodyPolicy::KinematicVelocityBased;

    assert!(matches!(
        FxRapierWorld2D::restore_snapshot(&encode_world_snapshot(&snapshot)),
        Err(FxRapierError::Snapshot(
            FxRapierSnapshotError::StateMismatch("static anchor applied body type mismatch")
        ))
    ));
}

#[cfg(feature = "deterministic-replay")]
#[test]
fn snapshot_restore_active_contact_uses_cold_start_contract() {
    let mut world = FxRapierWorld2D::new();
    set_deterministic_replay_mode(&mut world);
    world.set_gravity(Vector::ZERO);
    world
        .add_destructible(FxFamilyId(1), single_node_asset(7))
        .unwrap();
    world
        .add_destructible(FxFamilyId(2), single_node_asset(7))
        .unwrap();
    let b = world.actor_handles(FxFamilyId(2), FxActorId(0)).unwrap();
    world
        .rigid_bodies_mut()
        .get_mut(b.body)
        .unwrap()
        .set_position(Pose::from_translation(Vector::new(0.5, 0.0)), true);

    world.step().unwrap();

    let bytes = world.snapshot().unwrap();
    let mut restored = FxRapierWorld2D::restore_snapshot(&bytes).unwrap();
    assert_eq!(restored.snapshot_mode(), SnapshotMode::Deterministic);
    assert_eq!(restored.tick(), world.tick());
    restored.step().unwrap();
    assert!(
        restored
            .actor_handles(FxFamilyId(1), FxActorId(0))
            .is_some()
    );
    assert!(
        restored
            .actor_handles(FxFamilyId(2), FxActorId(0))
            .is_some()
    );
}

fn assert_vector_close(actual: Vector, expected: Vector) {
    assert!(
        (actual - expected).length() < 0.0001,
        "actual={actual:?} expected={expected:?}"
    );
}

fn assert_scalar_close(actual: f32, expected: f32) {
    assert!(
        (actual - expected).abs() < 0.0001,
        "actual={actual:?} expected={expected:?}"
    );
}

#[cfg(feature = "deterministic-replay")]
#[test]
fn snapshot_restore() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    set_deterministic_replay_mode(&mut world);
    world.set_gravity(Vector::ZERO);
    world.integration_parameters_mut().dt = 1.0 / 120.0;
    world.set_contact_material_properties(
        5,
        ContactMaterialProperties {
            friction: 0.25,
            restitution: 0.1,
        },
    );
    world
        .add_destructible(family, four_node_line_asset())
        .unwrap();
    world
        .connect_static_anchor(
            family,
            StaticAnchorConnectionDesc::new(static_anchor_desc(33, 0))
                .with_body_policy(StaticAnchorBodyPolicy::Fixed),
        )
        .unwrap();
    world
        .fracture_and_sync_for_test(
            family,
            &[break_bond_command(0, family, FxActorId(0), BondId(2))],
        )
        .unwrap();
    let child = world.actor_handles(family, FxActorId(1)).unwrap();
    {
        let body = world.rigid_bodies_mut().get_mut(child.body).unwrap();
        body.set_position(
            Pose::from_parts(Vector::new(3.0, 4.0), Rotation::new(0.4)),
            true,
        );
        body.set_linvel(Vector::new(1.0, -2.0), true);
        body.set_angvel(0.75, true);
        body.enable_ccd(true);
    }

    let bytes = world.snapshot().unwrap();
    assert_eq!(&bytes[0..4], b"RFXS");
    let restored = FxRapierWorld2D::restore_snapshot(&bytes).unwrap();
    assert_eq!(restored.snapshot_mode(), SnapshotMode::Deterministic);
    assert_eq!(restored.tick(), world.tick());
    assert_eq!(
        restored
            .family(family)
            .unwrap()
            .deterministic_state_digest(),
        world.family(family).unwrap().deterministic_state_digest()
    );
    let logical_body = |world: &FxRapierWorld2D| {
        world
            .sorted_actor_body_trace()
            .into_iter()
            .map(|body| {
                (
                    body.family,
                    body.actor,
                    body.translation,
                    body.rotation_angle,
                    body.linvel,
                    body.angvel,
                    body.body_type,
                    body.sleeping,
                )
            })
            .collect::<Vec<_>>()
    };
    assert_eq!(logical_body(&restored), logical_body(&world));
}

#[cfg(feature = "deterministic-replay")]
#[test]
fn replay_split_order() {
    let family = FxFamilyId(1);
    let mut a = FxRapierWorld2D::new();
    let mut b = FxRapierWorld2D::new();
    set_deterministic_replay_mode(&mut a);
    set_deterministic_replay_mode(&mut b);
    a.set_gravity(Vector::ZERO);
    b.set_gravity(Vector::ZERO);
    a.add_destructible(family, four_node_line_asset()).unwrap();
    b.add_destructible(family, four_node_line_asset()).unwrap();
    let commands = vec![
        FxRapierReplayCommand {
            tick: 0,
            stable_order: 2,
            family,
            command: break_bond_command(0, family, FxActorId(0), BondId(2)),
        },
        FxRapierReplayCommand {
            tick: 0,
            stable_order: 1,
            family,
            command: break_bond_command(0, family, FxActorId(0), BondId(0)),
        },
    ];
    let reversed = vec![commands[1].clone(), commands[0].clone()];

    let a_report = a.apply_replay_tick(0, &commands).unwrap();
    let b_report = b.apply_replay_tick(0, &reversed).unwrap();
    assert_eq!(a_report.split_events, b_report.split_events);
    assert_eq!(a.sorted_actor_body_trace(), b.sorted_actor_body_trace());
    assert_eq!(
        a.family(family).unwrap().deterministic_state_digest(),
        b.family(family).unwrap().deterministic_state_digest()
    );
}

#[cfg(feature = "deterministic-replay")]
#[test]
fn replay_rejects_enabled_lod() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    world.set_snapshot_mode(SnapshotMode::Deterministic);
    world.set_lod_settings(ColliderLodSettings::small_debris_box(4, 1));
    world.set_gravity(Vector::ZERO);
    world
        .add_destructible(family, four_node_line_asset())
        .unwrap();
    let command = FxRapierReplayCommand {
        tick: 0,
        stable_order: 0,
        family,
        command: break_bond_command(0, family, FxActorId(0), BondId(0)),
    };

    assert!(matches!(
        world.apply_replay_tick(0, &[command]),
        Err(FxRapierError::Snapshot(
            FxRapierSnapshotError::InvalidValue("deterministic lod")
        ))
    ));
}

#[cfg(feature = "deterministic-replay")]
#[test]
fn replay_accepts_public_deterministic_mode_setup() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    world.set_snapshot_mode(SnapshotMode::Deterministic);
    world.set_gravity(Vector::ZERO);
    world
        .add_destructible(family, four_node_line_asset())
        .unwrap();
    let command = FxRapierReplayCommand {
        tick: 0,
        stable_order: 0,
        family,
        command: break_bond_command(0, family, FxActorId(0), BondId(0)),
    };

    let report = world.apply_replay_tick(0, &[command]).unwrap();
    assert_eq!(report.split_events.len(), 1);
    assert_eq!(world.snapshot_mode(), SnapshotMode::Deterministic);
    assert_eq!(world.lod_settings(), ColliderLodSettings::disabled());
    world.snapshot().unwrap();
}

#[cfg(feature = "deterministic-replay")]
#[test]
fn replay_rejects_duplicate_ambiguous_keys() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    set_deterministic_replay_mode(&mut world);
    world.set_gravity(Vector::ZERO);
    world
        .add_destructible(family, four_node_line_asset())
        .unwrap();
    let commands = vec![
        FxRapierReplayCommand {
            tick: 0,
            stable_order: 7,
            family,
            command: break_bond_command(0, family, FxActorId(0), BondId(0)),
        },
        FxRapierReplayCommand {
            tick: 0,
            stable_order: 7,
            family,
            command: break_bond_command(0, family, FxActorId(0), BondId(0)),
        },
    ];

    assert!(matches!(
        world.apply_replay_tick(0, &commands),
        Err(FxRapierError::DuplicateReplayKey {
            tick: 0,
            stable_order: 7
        })
    ));
}

#[cfg(feature = "deterministic-replay")]
#[test]
fn replay_rejects_unknown_family_commands() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    set_deterministic_replay_mode(&mut world);
    world.set_gravity(Vector::ZERO);
    world.add_destructible(family, two_node_asset(5)).unwrap();
    let command = FxRapierReplayCommand {
        tick: 0,
        stable_order: 0,
        family: FxFamilyId(99),
        command: break_bond_command(0, FxFamilyId(99), FxActorId(0), BondId(0)),
    };

    assert!(matches!(
        world.apply_replay_tick(0, &[command]),
        Err(FxRapierError::UnknownReplayFamily(FxFamilyId(99)))
    ));
}

#[test]
fn normal_mode_not_assert_exact() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    world.set_snapshot_mode(SnapshotMode::Normal);
    world.set_gravity(Vector::ZERO);
    world.add_destructible(family, two_node_asset(5)).unwrap();
    let bytes = world.snapshot().unwrap();
    assert_eq!(&bytes[0..4], b"RFXS");
    let restored = FxRapierWorld2D::restore_snapshot(&bytes).unwrap();
    assert_eq!(restored.snapshot_mode(), SnapshotMode::Normal);
    assert!(restored.actor_handles(family, FxActorId(0)).is_some());
    let mut restored = restored;
    assert!(matches!(
        restored.run_replay_trace(restored.tick() + 1, &[]),
        Err(FxRapierError::ReplayRequiresDeterministicMode)
    ));
}

#[cfg(not(feature = "deterministic-replay"))]
#[test]
fn deterministic_mode_requires_feature_for_snapshot_and_replay() {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    world.set_snapshot_mode(SnapshotMode::Deterministic);
    world.add_destructible(family, two_node_asset(5)).unwrap();

    assert!(matches!(
        world.snapshot(),
        Err(FxRapierError::Snapshot(
            FxRapierSnapshotError::DeterministicReplayFeatureRequired
        ))
    ));
    assert!(matches!(
        world.apply_replay_tick(0, &[]),
        Err(FxRapierError::Snapshot(
            FxRapierSnapshotError::DeterministicReplayFeatureRequired
        ))
    ));
    assert!(matches!(
        world.run_replay_trace(world.tick(), &[]),
        Err(FxRapierError::Snapshot(
            FxRapierSnapshotError::DeterministicReplayFeatureRequired
        ))
    ));
}

#[cfg(feature = "deterministic-replay")]
#[test]
fn deterministic_1000_tick_small_scene_replay_identical() {
    fn build() -> FxRapierWorld2D {
        let mut world = FxRapierWorld2D::new();
        set_deterministic_replay_mode(&mut world);
        world.set_gravity(Vector::ZERO);
        world.integration_parameters_mut().dt = 1.0 / 60.0;
        world
            .add_destructible(FxFamilyId(1), four_node_line_asset())
            .unwrap();
        world
    }

    let family = FxFamilyId(1);
    let commands = vec![
        FxRapierReplayCommand {
            tick: 10,
            stable_order: 1,
            family,
            command: break_bond_command(10, family, FxActorId(0), BondId(0)),
        },
        FxRapierReplayCommand {
            tick: 500,
            stable_order: 2,
            family,
            command: break_bond_command(500, family, FxActorId(0), BondId(2)),
        },
    ];
    let reversed = vec![commands[1].clone(), commands[0].clone()];
    let mut a = build();
    let mut b = build();
    let trace_a = a.run_replay_trace(1000, &commands).unwrap();
    let trace_b = b.run_replay_trace(1000, &reversed).unwrap();
    assert_eq!(trace_a, trace_b);
    assert_eq!(a.snapshot().unwrap(), b.snapshot().unwrap());
}

#[cfg(feature = "deterministic-replay")]
#[test]
fn deterministic_replay_live_contact_identical() {
    fn build() -> FxRapierWorld2D {
        let mut world = FxRapierWorld2D::new();
        set_deterministic_replay_mode(&mut world);
        world.set_gravity(Vector::ZERO);
        world
            .add_destructible(FxFamilyId(1), single_node_asset(7))
            .unwrap();
        world
            .add_destructible(FxFamilyId(2), single_node_asset(7))
            .unwrap();
        let b = world.actor_handles(FxFamilyId(2), FxActorId(0)).unwrap();
        world
            .rigid_bodies_mut()
            .get_mut(b.body)
            .unwrap()
            .set_position(Pose::from_translation(Vector::new(0.5, 0.0)), true);
        world
    }

    let mut a = build();
    let mut b = build();
    assert_eq!(
        a.run_replay_trace(16, &[]).unwrap(),
        b.run_replay_trace(16, &[]).unwrap()
    );
}

#[cfg(feature = "deterministic-replay")]
#[test]
fn deterministic_static_anchor_after_restore_identical() {
    let family = FxFamilyId(1);
    let mut original = FxRapierWorld2D::new();
    set_deterministic_replay_mode(&mut original);
    original.set_gravity(Vector::ZERO);
    original
        .add_destructible(family, two_node_asset(7))
        .unwrap();
    original
        .connect_static_anchor(
            family,
            StaticAnchorConnectionDesc::new(static_anchor_desc(44, 0))
                .with_body_policy(StaticAnchorBodyPolicy::Fixed),
        )
        .unwrap();
    let mut restored = FxRapierWorld2D::restore_snapshot(&original.snapshot().unwrap()).unwrap();

    assert_eq!(
        original.run_replay_trace(24, &[]).unwrap(),
        restored.run_replay_trace(24, &[]).unwrap()
    );
}

#[cfg(feature = "deterministic-replay")]
#[test]
fn deterministic_joint_feedback_after_restore_identical() {
    let mut original = FxRapierWorld2D::new();
    set_deterministic_replay_mode(&mut original);
    original.set_gravity(Vector::new(0.0, -9.81));
    original
        .add_destructible(FxFamilyId(1), single_node_asset(7))
        .unwrap();
    let destructible = original.actor_handles(FxFamilyId(1), FxActorId(0)).unwrap();
    let anchor =
        original.insert_rigid_body(RigidBodyBuilder::fixed().translation(Vector::new(0.0, 1.0)));
    original.insert_impulse_joint(destructible.body, anchor, FixedJointBuilder::new(), true);
    let mut restored = FxRapierWorld2D::restore_snapshot(&original.snapshot().unwrap()).unwrap();

    let original_report = original.step().unwrap();
    let restored_report = restored.step().unwrap();
    assert_eq!(
        original_report.joint_feedback,
        restored_report.joint_feedback
    );
}

#[test]
fn deterministic_trace_uses_logical_body_mapping() {
    let family = FxFamilyId(1);
    let mut a = FxRapierWorld2D::new();
    let mut b = FxRapierWorld2D::new();
    a.set_gravity(Vector::ZERO);
    b.set_gravity(Vector::ZERO);
    a.add_destructible(family, single_node_asset(7)).unwrap();
    add_fixed_box(
        &mut b,
        Vector::new(100.0, 100.0),
        Vector::new(0.5, 0.5),
        1.0,
    );
    b.add_destructible(family, single_node_asset(7)).unwrap();

    assert_ne!(
        a.actor_handles(family, FxActorId(0)).unwrap().body,
        b.actor_handles(family, FxActorId(0)).unwrap().body
    );
    assert_eq!(a.sorted_actor_body_trace(), b.sorted_actor_body_trace());
}
