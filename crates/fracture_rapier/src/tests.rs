use std::collections::BTreeSet;

use fracture_core::{
    BondId, CommandId, ConnectionError, ConnectionId, DamageSource, DeterministicOrderKey,
    DynamicConnectionPolicy, DynamicStructuralBondDesc, ExternalBondId, ExternalTarget2D,
    ExternalTargetKind, ExternalTargetToken, FractureCommand, FractureTarget, FxActorId,
    FxFamilyId, StaticAnchorDesc, StressSettings, SupportNodeId, Vec2,
};
use fracture_voxel::{VoxelAuthoringInput, author_voxel_asset};
use rapier2d::prelude::*;

use crate::contact_map::{ContactPairSide, map_contact_pair};
use crate::{
    ContactMaterialProperties, DynamicStructuralConnectionDesc, FxRapierError, FxRapierWorld2D,
    StaticAnchorBodyPolicy, StaticAnchorConnectionDesc,
};

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
        let report = world.step().unwrap();
        hits = report
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
        let step = world.step().unwrap();
        if !step.split_events.is_empty() {
            report = Some(step);
            break;
        }
    }
    let report = report.expect("same-step stress split");
    assert_eq!(report.split_events.len(), 1);
    let event = &report.split_events[0];
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
