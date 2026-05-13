use std::env;
use std::error::Error;
use std::fmt::Write as _;
use std::fs;
use std::path::PathBuf;

use fracture_core::{
    CommandId, DamageSource, DeterministicOrderKey, ExternalBondId, ExternalTarget2D,
    ExternalTargetKind, ExternalTargetToken, FractureCommand, FractureTarget, FxActorId,
    FxFamilyId, GridCoord, StaticAnchorDesc, StressInput, StressProfile, StressSettings,
    StressSolver2D, SupportNodeId, Vec2,
};
use fracture_rapier::{
    FxPerformanceBudgetReport, FxPhysicsSyncReport, FxRapierWorld2D, StaticAnchorBodyPolicy,
    StaticAnchorConnectionDesc,
};
use fracture_voxel::{RuntimeEdit, VoxelAuthoringInput, VoxelRuntime, author_voxel_asset};
use rapier2d::prelude::*;

const IMPACT_ENERGY_MAX_RATIO: f32 = 0.90;

#[derive(Clone, Debug)]
struct HighSpeedImpactDemo {
    normal_impulse_sum: f32,
    stress_input_count: usize,
    contact_stress_input_count: usize,
    fracture_count: usize,
    split_count: usize,
    health_loss: f32,
    effective_length_loss: f32,
    kinetic_estimate_before: f32,
    kinetic_estimate_after: f32,
    max_speed_before: f32,
    max_speed_after: f32,
    kinetic_absorption_ratio: f32,
    absorption_pass_condition: String,
    sync: FxPhysicsSyncReport,
    budget: FxPerformanceBudgetReport,
}

#[derive(Clone, Debug)]
struct BridgeCollapseDemo {
    anchor_policy: StaticAnchorBodyPolicy,
    gravity: Vec2,
    total_gravity_load: Vec2,
    generated_commands: usize,
    profile: StressProfile,
    fracture_count: usize,
    split_count: usize,
    broken_load_bearing_bonds: usize,
    actor_count_before: usize,
    actor_count: usize,
    body_states: Vec<String>,
    sync: FxPhysicsSyncReport,
    budget: FxPerformanceBudgetReport,
}

#[derive(Clone, Debug)]
struct JointPullDemo {
    joint_impulse_magnitude: f32,
    stress_force: Vec2,
    stress_input_count: usize,
    joint_stress_input_count: usize,
    fracture_count: usize,
    split_count: usize,
    child_actor_handles: Vec<String>,
    sync: FxPhysicsSyncReport,
    budget: FxPerformanceBudgetReport,
}

#[derive(Clone, Debug)]
struct RuntimeDigHoleDemo {
    dirty_bbox: String,
    affected_old_nodes: Vec<SupportNodeId>,
    reused_nodes: Vec<SupportNodeId>,
    new_nodes: Vec<SupportNodeId>,
    pre_damage_event_count: usize,
    pre_edit_damaged_state: (SupportNodeId, f32, f32),
    preserved_health_values: Vec<(SupportNodeId, f32, f32)>,
    actor_count_before: usize,
    actor_count_after_edit: usize,
    actor_count_after_split: usize,
    split_count: usize,
    exact_cover_validated: bool,
    unaffected_region_preserved: bool,
}

fn parse_out_arg() -> Result<PathBuf, Box<dyn Error>> {
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--out" => {
                if let Some(path) = args.next() {
                    return Ok(PathBuf::from(path));
                }
                return Err("--out requires a path".into());
            }
            "-h" | "--help" => {
                println!("Usage: phase7_acceptance_demos --out <path>");
                std::process::exit(0);
            }
            other => return Err(format!("unknown argument `{other}`").into()),
        }
    }
    Err("missing required --out <path>".into())
}

fn ensure(condition: bool, message: impl Into<String>) -> Result<(), Box<dyn Error>> {
    if condition {
        Ok(())
    } else {
        Err(message.into().into())
    }
}

fn two_node_asset(contact_material: u16) -> fracture_voxel::AuthoredVoxelAsset {
    let mut input = VoxelAuthoringInput::new(
        2,
        1,
        1.0,
        vec![true, true],
        vec![1, 1],
        vec![contact_material, contact_material],
        vec![0, 1],
    );
    input.support_node_hint = Some(vec![Some(0), Some(1)]);
    input.default_bond_health = 1.0;
    input.default_tension_limit = 0.01;
    input.default_shear_limit = 0.01;
    author_voxel_asset(input).expect("two-node acceptance asset should author")
}

fn four_node_line_asset(contact_material: u16) -> fracture_voxel::AuthoredVoxelAsset {
    let mut input = VoxelAuthoringInput::new(
        4,
        1,
        1.0,
        vec![true, true, true, true],
        vec![1, 1, 1, 1],
        vec![contact_material; 4],
        vec![0, 1, 2, 3],
    );
    input.support_node_hint = Some(vec![Some(0), Some(1), Some(2), Some(3)]);
    input.default_bond_health = 1.0;
    input.default_tension_limit = 0.01;
    input.default_shear_limit = 0.01;
    author_voxel_asset(input).expect("four-node acceptance asset should author")
}

fn three_voxel_single_node_asset() -> fracture_voxel::AuthoredVoxelAsset {
    let mut input = VoxelAuthoringInput::new(
        3,
        1,
        1.0,
        vec![true, true, true],
        vec![1, 1, 1],
        vec![7, 7, 7],
        vec![0, 1, 2],
    );
    input.support_node_hint = Some(vec![Some(0), Some(0), Some(0)]);
    author_voxel_asset(input).expect("runtime edit acceptance asset should author")
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
        health: 100.0,
        effective_length: 1.0,
        tension_limit: 100.0,
        shear_limit: 100.0,
    }
}

fn run_high_speed_impact_demo() -> Result<HighSpeedImpactDemo, Box<dyn Error>> {
    let family = FxFamilyId(1);
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(Vector::ZERO);
    world.integration_parameters_mut().dt = 1.0 / 60.0;
    world.set_stress_settings(StressSettings {
        damage_per_overload: 2.0,
        max_fractures_per_frame: 4,
        ..StressSettings::default()
    });
    world.add_destructible(family, two_node_asset(5))?;

    let destructible = world
        .actor_handles(family, FxActorId(0))
        .ok_or("impact actor handles missing")?;
    let (kinetic_estimate_before, max_speed_before) = {
        let body = world
            .rigid_bodies_mut()
            .get_mut(destructible.body)
            .ok_or("impact body missing")?;
        body.set_position(
            Pose::from_parts(Vector::new(4.0, 3.0), Rotation::new(0.25)),
            true,
        );
        body.set_linvel(Vector::new(1.25, -6.0), true);
        body.set_angvel(1.5, true);
        body.enable_ccd(true);
        (
            0.5 * body.mass() * body.linvel().length_squared(),
            body.linvel().length(),
        )
    };
    add_fixed_box(
        &mut world,
        Vector::new(4.48, 2.55),
        Vector::new(0.45, 0.1),
        0.5,
    );

    let mut accepted = None;
    for _ in 0..8 {
        let step = world.step_with_diagnostics()?;
        let has_contact_impulse = step
            .report
            .contact_impulses
            .iter()
            .any(|input| input.impulse.normal_impulse > 0.0);
        if has_contact_impulse && !step.report.split_events.is_empty() {
            accepted = Some(step);
            break;
        }
    }
    let step = accepted.ok_or("high-speed impact did not produce contact impulse and split")?;
    let budget = step
        .diagnostics
        .budget
        .ok_or("impact diagnostics missing budget")?;
    ensure(
        budget.within_budget(),
        "impact demo exceeded performance budget",
    )?;

    let (kinetic_estimate_after, max_speed_after) = actor_motion_summary(&world, family)?;
    let kinetic_absorption_ratio =
        kinetic_estimate_after / kinetic_estimate_before.max(f32::EPSILON);
    ensure(
        kinetic_absorption_ratio <= IMPACT_ENERGY_MAX_RATIO,
        format!(
            "impact absorption failed: post/pre linear kinetic ratio {kinetic_absorption_ratio:.3} > {IMPACT_ENERGY_MAX_RATIO:.3}"
        ),
    )?;

    let normal_impulse_sum = step
        .report
        .contact_impulses
        .iter()
        .map(|input| input.impulse.normal_impulse)
        .sum::<f32>();
    let contact_stress_input_count = step
        .report
        .stress_inputs
        .iter()
        .filter(|input| input.source == DamageSource::ContactImpulse)
        .count();
    ensure(
        normal_impulse_sum > 0.0,
        "impact normal impulse sum was zero",
    )?;
    ensure(
        contact_stress_input_count > 0,
        "impact demo did not report ContactImpulse stress",
    )?;
    ensure(
        !step.report.fracture_events.is_empty(),
        "impact demo did not fracture",
    )?;
    ensure(
        !step.report.split_events.is_empty(),
        "impact demo did not split",
    )?;
    ensure(
        step.diagnostics.physics_sync.rebuilt_colliders > 0
            && step.diagnostics.physics_sync.created_actor_bodies > 0,
        "impact demo did not sync split physics",
    )?;

    Ok(HighSpeedImpactDemo {
        normal_impulse_sum,
        stress_input_count: step.report.stress_inputs.len(),
        contact_stress_input_count,
        fracture_count: step.report.fracture_events.len(),
        split_count: step.report.split_events.len(),
        health_loss: step
            .report
            .fracture_events
            .iter()
            .map(|event| event.old_health - event.new_health)
            .sum(),
        effective_length_loss: step
            .report
            .fracture_events
            .iter()
            .map(|event| event.old_effective_length - event.new_effective_length)
            .sum(),
        kinetic_estimate_before,
        kinetic_estimate_after,
        max_speed_before,
        max_speed_after,
        kinetic_absorption_ratio,
        absorption_pass_condition: format!(
            "post/pre linear kinetic ratio <= {IMPACT_ENERGY_MAX_RATIO:.2}"
        ),
        sync: step.diagnostics.physics_sync,
        budget,
    })
}

fn run_bridge_collapse_demo() -> Result<BridgeCollapseDemo, Box<dyn Error>> {
    let family = FxFamilyId(2);
    let anchor_policy = StaticAnchorBodyPolicy::Fixed;
    let gravity = Vec2::new(0.0, -9.81);
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(Vector::new(gravity.x, gravity.y));
    world.set_stress_settings(StressSettings {
        damage_per_overload: 2.0,
        max_fractures_per_frame: 4,
        ..StressSettings::default()
    });
    world.add_destructible(family, four_node_line_asset(6))?;
    world.connect_static_anchor(
        family,
        StaticAnchorConnectionDesc::new(static_anchor_desc(1, 0)).with_body_policy(anchor_policy),
    )?;

    let actor_count_before = world
        .family(family)
        .ok_or("bridge family missing before apply")?
        .actor_count();
    let stress_inputs = [1u32, 2, 3]
        .into_iter()
        .map(|node| StressInput {
            order_key: DeterministicOrderKey::new(0, 30, family, FxActorId(0), CommandId(node)),
            actor: FxActorId(0),
            node: SupportNodeId(node),
            force: gravity,
            source: DamageSource::Stress,
        })
        .collect::<Vec<_>>();
    let total_gravity_load = stress_inputs
        .iter()
        .fold(Vec2::ZERO, |acc, input| acc + input.force);

    let solver = StressSolver2D::new(StressSettings {
        damage_per_overload: 2.0,
        max_fractures_per_frame: 4,
        ..StressSettings::default()
    });
    let solve = {
        let family_ref = world.family(family).ok_or("bridge family missing")?;
        solver.generate_with_profile(family_ref, &stress_inputs)
    };
    ensure(
        solve.profile.internal_bonds_tested > 0 || solve.profile.external_bonds_tested > 0,
        "bridge stress profile did not test load-bearing bonds",
    )?;
    ensure(
        !solve.commands.is_empty(),
        "bridge stress solver generated no fracture commands",
    )?;
    let generated_commands = solve.commands.len();

    let step = world.apply_fracture_commands_to_family(family, &solve.commands)?;
    let budget = step
        .diagnostics
        .budget
        .ok_or("bridge diagnostics missing budget")?;
    ensure(
        budget.within_budget(),
        "bridge demo exceeded performance budget",
    )?;

    let family_ref = world
        .family(family)
        .ok_or("bridge family missing after apply")?;
    let actor_count = family_ref.actor_count();
    let broken_load_bearing_bonds = family_ref
        .bond_states()
        .iter()
        .filter(|state| state.is_broken())
        .count()
        + family_ref
            .external_bonds()
            .filter(|(_, bond)| bond.runtime.is_broken())
            .count()
        + family_ref
            .dynamic_structural_bonds()
            .filter(|(_, bond)| bond.runtime.is_broken())
            .count();
    ensure(
        !step.report.fracture_events.is_empty(),
        "bridge demo produced no fracture event",
    )?;
    ensure(
        !step.report.split_events.is_empty() || actor_count > actor_count_before,
        "bridge demo produced fracture but no split/collapse actor increase",
    )?;

    let body_states = family_ref
        .actors()
        .map(|(actor, _)| actor_body_state(&world, family, *actor))
        .collect::<Result<Vec<_>, _>>()?;

    Ok(BridgeCollapseDemo {
        anchor_policy,
        gravity,
        total_gravity_load,
        generated_commands,
        profile: solve.profile,
        fracture_count: step.report.fracture_events.len(),
        split_count: step.report.split_events.len(),
        broken_load_bearing_bonds,
        actor_count_before,
        actor_count,
        body_states,
        sync: step.diagnostics.physics_sync,
        budget,
    })
}

fn run_joint_pull_demo() -> Result<JointPullDemo, Box<dyn Error>> {
    let family = FxFamilyId(3);
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(Vector::new(0.0, -9.81));
    world.integration_parameters_mut().dt = 1.0 / 60.0;
    world.set_stress_settings(StressSettings {
        damage_per_overload: 2.0,
        max_fractures_per_frame: 4,
        ..StressSettings::default()
    });
    world.add_destructible(family, two_node_asset(7))?;
    let destructible = world
        .actor_handles(family, FxActorId(0))
        .ok_or("joint actor handles missing")?;
    let anchor = add_fixed_box(
        &mut world,
        Vector::new(1.0, 1.75),
        Vector::new(0.1, 0.1),
        0.5,
    );
    world.insert_impulse_joint(destructible.body, anchor, FixedJointBuilder::new(), true);

    let mut accepted = None;
    for _ in 0..12 {
        let step = world.step_with_diagnostics()?;
        let has_joint_feedback = step
            .report
            .joint_feedback
            .iter()
            .any(|feedback| feedback.impulse_magnitude > 0.0);
        if has_joint_feedback && !step.report.split_events.is_empty() {
            accepted = Some(step);
            break;
        }
    }
    let step = accepted.ok_or("joint pull did not produce joint feedback and split")?;
    let budget = step
        .diagnostics
        .budget
        .ok_or("joint diagnostics missing budget")?;
    ensure(
        budget.within_budget(),
        "joint demo exceeded performance budget",
    )?;

    let strongest = step
        .report
        .joint_feedback
        .iter()
        .max_by(|a, b| {
            a.impulse_magnitude
                .partial_cmp(&b.impulse_magnitude)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .ok_or("joint feedback missing from accepted step")?;
    let joint_stress_input_count = step
        .report
        .stress_inputs
        .iter()
        .filter(|input| input.source == DamageSource::JointFeedback)
        .count();
    ensure(
        joint_stress_input_count > 0,
        "joint demo did not report JointFeedback stress",
    )?;
    ensure(
        !step.report.fracture_events.is_empty(),
        "joint demo did not fracture",
    )?;
    ensure(
        !step.report.split_events.is_empty(),
        "joint demo did not split",
    )?;
    ensure(
        step.diagnostics.physics_sync.rebuilt_colliders > 0
            && step.diagnostics.physics_sync.created_actor_bodies > 0,
        "joint demo did not sync split physics",
    )?;

    let child_actor_handles = step
        .report
        .split_events
        .iter()
        .flat_map(|event| event.created_children.iter().copied())
        .map(|actor| actor_body_state(&world, family, actor))
        .collect::<Result<Vec<_>, _>>()?;

    Ok(JointPullDemo {
        joint_impulse_magnitude: strongest.impulse_magnitude,
        stress_force: strongest.stress.force,
        stress_input_count: step.report.stress_inputs.len(),
        joint_stress_input_count,
        fracture_count: step.report.fracture_events.len(),
        split_count: step.report.split_events.len(),
        child_actor_handles,
        sync: step.diagnostics.physics_sync,
        budget,
    })
}

fn run_runtime_dig_hole_demo() -> Result<RuntimeDigHoleDemo, Box<dyn Error>> {
    let family = FxFamilyId(4);
    let asset = three_voxel_single_node_asset();
    let mut runtime = VoxelRuntime::instantiate(family, asset);
    let actor_count_before = runtime.family().actor_count();
    let damage = FractureCommand {
        order_key: DeterministicOrderKey::new(1, 1, family, FxActorId(0), CommandId(0)),
        actor: FxActorId(0),
        target: FractureTarget::Node(SupportNodeId(0)),
        health_loss: 0.4,
        effective_length_loss: 0.0,
        source: DamageSource::Script,
    };
    let pre_damage_events = runtime.apply_fracture_commands(&[damage]);
    ensure(
        !pre_damage_events.is_empty(),
        "runtime pre-edit damage produced no event",
    )?;
    let pre_state = runtime
        .family()
        .node_state(SupportNodeId(0))
        .ok_or("runtime damaged node state missing")?;
    let pre_edit_damaged_state = (
        SupportNodeId(0),
        pre_state.health,
        pre_state.accumulated_damage,
    );
    ensure(
        pre_edit_damaged_state.1 < 1.0 && pre_edit_damaged_state.2 > 0.0,
        "runtime pre-edit damage did not create non-default health/damage",
    )?;

    let report = runtime.apply_edit(RuntimeEdit::RemoveVoxels {
        voxels: vec![GridCoord::new(1, 0)],
    })?;
    let actor_count_after_edit = runtime.family().actor_count();
    ensure(
        report.exact_cover_validated,
        "runtime edit did not validate exact cover",
    )?;
    runtime.asset().validate_exact_cover()?;

    let preserved_health_values = runtime
        .family()
        .node_states()
        .filter(|(_, state)| {
            (state.health - 0.3).abs() < 0.0001 && (state.accumulated_damage - 0.2).abs() < 0.0001
        })
        .map(|(node, state)| (node, state.health, state.accumulated_damage))
        .collect::<Vec<_>>();
    ensure(
        preserved_health_values.len() == 2,
        "runtime edit did not preserve non-default damaged node state on both surviving fragments",
    )?;
    ensure(
        preserved_health_values
            .iter()
            .all(|(_, health, damage)| *health < 1.0 && *damage > 0.0),
        "runtime preserved health values reset to pristine defaults",
    )?;

    let split_events = runtime.split_dirty_actors();
    let actor_count_after_split = runtime.family().actor_count();
    ensure(
        !split_events.is_empty(),
        "runtime edit did not split dirty actor",
    )?;
    ensure(
        actor_count_after_split > actor_count_before,
        "runtime edit did not increase actor count after split",
    )?;

    Ok(RuntimeDigHoleDemo {
        dirty_bbox: report
            .dirty_bbox
            .map(|bbox| format!("{:?}..{:?}", bbox.min, bbox.max))
            .unwrap_or_else(|| "none".to_owned()),
        affected_old_nodes: report.affected_old_nodes,
        reused_nodes: report.reused_nodes,
        new_nodes: report.new_nodes,
        pre_damage_event_count: pre_damage_events.len(),
        pre_edit_damaged_state,
        preserved_health_values,
        actor_count_before,
        actor_count_after_edit,
        actor_count_after_split,
        split_count: split_events.len(),
        exact_cover_validated: report.exact_cover_validated,
        unaffected_region_preserved: report.unaffected_region_preserved,
    })
}

fn actor_motion_summary(
    world: &FxRapierWorld2D,
    family: FxFamilyId,
) -> Result<(f32, f32), Box<dyn Error>> {
    let family_ref = world
        .family(family)
        .ok_or("family missing for motion summary")?;
    let mut kinetic = 0.0;
    let mut max_speed = 0.0f32;
    for (actor, _) in family_ref.actors() {
        let handles = world
            .actor_handles(family, *actor)
            .ok_or("actor handles missing for motion summary")?;
        let body = world
            .rigid_bodies()
            .get(handles.body)
            .ok_or("actor body missing for motion summary")?;
        let speed = body.linvel().length();
        kinetic += 0.5 * body.mass() * body.linvel().length_squared();
        max_speed = max_speed.max(speed);
    }
    Ok((kinetic, max_speed))
}

fn actor_body_state(
    world: &FxRapierWorld2D,
    family: FxFamilyId,
    actor: FxActorId,
) -> Result<String, Box<dyn Error>> {
    let handles = world.actor_handles(family, actor).ok_or_else(|| {
        format!(
            "actor handles missing for family {:?} actor {:?}",
            family, actor
        )
    })?;
    let body = world
        .rigid_bodies()
        .get(handles.body)
        .ok_or("actor body missing")?;
    let body_type = if body.is_fixed() {
        "fixed"
    } else if body.is_kinematic() {
        "kinematic"
    } else {
        "dynamic"
    };
    Ok(format!(
        "actor={:?} body={:?} collider={:?} type={} translation=({:.3},{:.3}) linvel=({:.3},{:.3})",
        actor,
        handles.body,
        handles.collider,
        body_type,
        body.translation().x,
        body.translation().y,
        body.linvel().x,
        body.linvel().y
    ))
}

fn markdown(
    impact: &HighSpeedImpactDemo,
    bridge: &BridgeCollapseDemo,
    joint: &JointPullDemo,
    dig: &RuntimeDigHoleDemo,
) -> String {
    let mut out = String::new();
    writeln!(&mut out, "# Phase 7 Acceptance Demos\n").unwrap();

    writeln!(&mut out, "## Demo 1 - High-Speed Impact\n").unwrap();
    writeln!(&mut out, "| Counter | Value |").unwrap();
    writeln!(&mut out, "| --- | ---: |").unwrap();
    writeln!(
        &mut out,
        "| Normal impulse sum | {:.6} |",
        impact.normal_impulse_sum
    )
    .unwrap();
    writeln!(
        &mut out,
        "| Stress inputs | {} |",
        impact.stress_input_count
    )
    .unwrap();
    writeln!(
        &mut out,
        "| ContactImpulse stress inputs | {} |",
        impact.contact_stress_input_count
    )
    .unwrap();
    writeln!(&mut out, "| Fracture events | {} |", impact.fracture_count).unwrap();
    writeln!(&mut out, "| Split events | {} |", impact.split_count).unwrap();
    writeln!(&mut out, "| Health loss | {:.6} |", impact.health_loss).unwrap();
    writeln!(
        &mut out,
        "| Effective-length loss | {:.6} |",
        impact.effective_length_loss
    )
    .unwrap();
    writeln!(
        &mut out,
        "| Linear kinetic before impact | {:.6} |",
        impact.kinetic_estimate_before
    )
    .unwrap();
    writeln!(
        &mut out,
        "| Linear kinetic after impact/split | {:.6} |",
        impact.kinetic_estimate_after
    )
    .unwrap();
    writeln!(
        &mut out,
        "| Max fragment speed before | {:.6} |",
        impact.max_speed_before
    )
    .unwrap();
    writeln!(
        &mut out,
        "| Max fragment speed after | {:.6} |",
        impact.max_speed_after
    )
    .unwrap();
    writeln!(
        &mut out,
        "| Kinetic absorption ratio | {:.6} |",
        impact.kinetic_absorption_ratio
    )
    .unwrap();
    writeln!(
        &mut out,
        "| Absorption pass condition | {} |",
        impact.absorption_pass_condition
    )
    .unwrap();
    write_sync_and_budget(&mut out, &impact.sync, &impact.budget);

    writeln!(
        &mut out,
        "\n## Demo 2 - Bridge Cantilever Gravity Collapse\n"
    )
    .unwrap();
    writeln!(&mut out, "| Counter | Value |").unwrap();
    writeln!(&mut out, "| --- | ---: |").unwrap();
    writeln!(
        &mut out,
        "| Static anchor policy | {:?} |",
        bridge.anchor_policy
    )
    .unwrap();
    writeln!(
        &mut out,
        "| Gravity vector | ({:.3}, {:.3}) |",
        bridge.gravity.x, bridge.gravity.y
    )
    .unwrap();
    writeln!(
        &mut out,
        "| Total generated gravity load | ({:.3}, {:.3}) |",
        bridge.total_gravity_load.x, bridge.total_gravity_load.y
    )
    .unwrap();
    writeln!(
        &mut out,
        "| Generated commands | {} |",
        bridge.generated_commands
    )
    .unwrap();
    writeln!(
        &mut out,
        "| Internal bonds tested | {} |",
        bridge.profile.internal_bonds_tested
    )
    .unwrap();
    writeln!(
        &mut out,
        "| External bonds tested | {} |",
        bridge.profile.external_bonds_tested
    )
    .unwrap();
    writeln!(&mut out, "| Fracture events | {} |", bridge.fracture_count).unwrap();
    writeln!(&mut out, "| Split events | {} |", bridge.split_count).unwrap();
    writeln!(
        &mut out,
        "| Broken load-bearing bonds | {} |",
        bridge.broken_load_bearing_bonds
    )
    .unwrap();
    writeln!(
        &mut out,
        "| Actor count before collapse | {} |",
        bridge.actor_count_before
    )
    .unwrap();
    writeln!(&mut out, "| Final actor count | {} |", bridge.actor_count).unwrap();
    write_sync_and_budget(&mut out, &bridge.sync, &bridge.budget);
    writeln!(&mut out, "\n### Bridge Body State\n").unwrap();
    for state in &bridge.body_states {
        writeln!(&mut out, "- `{state}`").unwrap();
    }

    writeln!(&mut out, "\n## Demo 3 - Joint Pull Fracture\n").unwrap();
    writeln!(&mut out, "| Counter | Value |").unwrap();
    writeln!(&mut out, "| --- | ---: |").unwrap();
    writeln!(
        &mut out,
        "| Joint impulse magnitude | {:.6} |",
        joint.joint_impulse_magnitude
    )
    .unwrap();
    writeln!(
        &mut out,
        "| Stress force | ({:.3}, {:.3}) |",
        joint.stress_force.x, joint.stress_force.y
    )
    .unwrap();
    writeln!(&mut out, "| Stress inputs | {} |", joint.stress_input_count).unwrap();
    writeln!(
        &mut out,
        "| JointFeedback stress inputs | {} |",
        joint.joint_stress_input_count
    )
    .unwrap();
    writeln!(&mut out, "| Fracture events | {} |", joint.fracture_count).unwrap();
    writeln!(&mut out, "| Split events | {} |", joint.split_count).unwrap();
    write_sync_and_budget(&mut out, &joint.sync, &joint.budget);
    writeln!(&mut out, "\n### Joint Child Actor Handles\n").unwrap();
    for state in &joint.child_actor_handles {
        writeln!(&mut out, "- `{state}`").unwrap();
    }

    writeln!(&mut out, "\n## Demo 4 - Runtime Dig-Hole Split\n").unwrap();
    writeln!(&mut out, "| Counter | Value |").unwrap();
    writeln!(&mut out, "| --- | ---: |").unwrap();
    writeln!(&mut out, "| Dirty bbox | `{}` |", dig.dirty_bbox).unwrap();
    writeln!(
        &mut out,
        "| Affected old nodes | `{}` |",
        fmt_debug_list(&dig.affected_old_nodes)
    )
    .unwrap();
    writeln!(
        &mut out,
        "| Reused nodes | `{}` |",
        fmt_debug_list(&dig.reused_nodes)
    )
    .unwrap();
    writeln!(
        &mut out,
        "| New nodes | `{}` |",
        fmt_debug_list(&dig.new_nodes)
    )
    .unwrap();
    writeln!(
        &mut out,
        "| Pre-edit damage events | {} |",
        dig.pre_damage_event_count
    )
    .unwrap();
    writeln!(
        &mut out,
        "| Pre-edit damaged state | `{:?}` |",
        dig.pre_edit_damaged_state
    )
    .unwrap();
    writeln!(
        &mut out,
        "| Preserved non-default health values | `{}` |",
        fmt_debug_list(&dig.preserved_health_values)
    )
    .unwrap();
    writeln!(
        &mut out,
        "| Actor count before edit | {} |",
        dig.actor_count_before
    )
    .unwrap();
    writeln!(
        &mut out,
        "| Actor count after edit | {} |",
        dig.actor_count_after_edit
    )
    .unwrap();
    writeln!(
        &mut out,
        "| Actor count after split | {} |",
        dig.actor_count_after_split
    )
    .unwrap();
    writeln!(&mut out, "| Split events | {} |", dig.split_count).unwrap();
    writeln!(
        &mut out,
        "| Exact cover validated | {} |",
        dig.exact_cover_validated
    )
    .unwrap();
    writeln!(
        &mut out,
        "| Unaffected region preserved | {} |",
        dig.unaffected_region_preserved
    )
    .unwrap();
    out
}

fn write_sync_and_budget(
    out: &mut String,
    sync: &FxPhysicsSyncReport,
    budget: &FxPerformanceBudgetReport,
) {
    writeln!(out, "| Rebuilt colliders | {} |", sync.rebuilt_colliders).unwrap();
    writeln!(
        out,
        "| Created actor bodies | {} |",
        sync.created_actor_bodies
    )
    .unwrap();
    writeln!(
        out,
        "| Removed actor bodies | {} |",
        sync.removed_actor_bodies
    )
    .unwrap();
    writeln!(
        out,
        "| Untouched actor count | {} |",
        sync.untouched_actor_count
    )
    .unwrap();
    writeln!(
        out,
        "| Primitive LOD replacements | {} |",
        sync.primitive_lod_replacements
    )
    .unwrap();
    writeln!(
        out,
        "| Budget occupied/support/bodies | {}/{} / {}/{} / {}/{} |",
        budget.occupied_voxels,
        budget.occupied_voxel_budget,
        budget.support_nodes,
        budget.support_node_budget,
        budget.active_bodies,
        budget.active_body_budget
    )
    .unwrap();
}

fn fmt_debug_list<T: std::fmt::Debug>(items: &[T]) -> String {
    format!("{items:?}")
}

fn main() -> Result<(), Box<dyn Error>> {
    let out = parse_out_arg()?;
    let impact = run_high_speed_impact_demo()?;
    let bridge = run_bridge_collapse_demo()?;
    let joint = run_joint_pull_demo()?;
    let dig = run_runtime_dig_hole_demo()?;

    let report = markdown(&impact, &bridge, &joint, &dig);
    if let Some(parent) = out.parent() {
        fs::create_dir_all(parent)?;
    }
    fs::write(&out, report)?;
    println!(
        "Phase 7 acceptance demos passed: impact energy_ratio={:.6} fractures/splits={}/{}, bridge actors/splits={}->{}/{}, joint impulse/fractures/splits={:.6}/{}/{}, dig damage_events actors/splits={}/{}->{}/{}",
        impact.kinetic_absorption_ratio,
        impact.fracture_count,
        impact.split_count,
        bridge.actor_count_before,
        bridge.actor_count,
        bridge.split_count,
        joint.joint_impulse_magnitude,
        joint.fracture_count,
        joint.split_count,
        dig.pre_damage_event_count,
        dig.actor_count_before,
        dig.actor_count_after_split,
        dig.split_count,
    );
    println!("wrote {}", out.display());
    Ok(())
}
