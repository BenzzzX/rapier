use std::env;
use std::error::Error;
use std::fs;
use std::path::PathBuf;

use fracture_core::{FxActorId, FxFamilyId, StressSettings};
use fracture_rapier::{FxRapierWorld2D, FxStepWithDiagnostics};
use fracture_voxel::{VoxelAuthoringInput, author_voxel_asset};
use rapier2d::prelude::*;

#[derive(Clone, Copy, Debug, Default)]
struct StressProfileTotals {
    input_count: usize,
    actor_count_visited: usize,
    actors_with_input: usize,
    internal_bond_candidates: usize,
    internal_bonds_tested: usize,
    external_bond_candidates: usize,
    external_bonds_tested: usize,
    dynamic_structural_bonds_tested: usize,
    generated_commands_before_cap: usize,
    generated_commands_after_cap: usize,
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
    author_voxel_asset(input).expect("phase6 profile asset should author")
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
                println!("Usage: phase6_profile --out <path>");
                std::process::exit(0);
            }
            other => return Err(format!("unknown argument `{other}`").into()),
        }
    }
    Err("missing required --out <path>".into())
}

fn add_fixed_box(world: &mut FxRapierWorld2D, translation: Vector, half_extents: Vector) {
    let body = world.insert_rigid_body(RigidBodyBuilder::fixed().translation(translation));
    world.insert_collider_with_parent(
        ColliderBuilder::cuboid(half_extents.x, half_extents.y)
            .friction(0.5)
            .build(),
        body,
    );
}

fn place_candidate(world: &mut FxRapierWorld2D, family: FxFamilyId, x: f32) {
    let handles = world
        .actor_handles(family, FxActorId(0))
        .expect("candidate actor should exist");
    let body = world
        .rigid_bodies_mut()
        .get_mut(handles.body)
        .expect("candidate body should exist");
    body.set_position(
        Pose::from_parts(Vector::new(x, 3.0), Rotation::new(0.25)),
        true,
    );
    body.set_linvel(Vector::new(1.25, -6.0), true);
    body.set_angvel(1.5, true);
    body.enable_ccd(true);
    add_fixed_box(world, Vector::new(x + 0.48, 2.55), Vector::new(0.45, 0.1));
}

fn run_profile() -> Result<FxStepWithDiagnostics, Box<dyn Error>> {
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(Vector::ZERO);
    world.set_stress_settings(StressSettings {
        damage_per_overload: 2.0,
        max_fractures_per_frame: 1,
        ..StressSettings::default()
    });

    for id in 1..=99 {
        world
            .add_destructible(FxFamilyId(id), two_node_asset(7))
            .expect("phase6 destructible should add");
    }
    for id in 1..=99 {
        let handles = world
            .actor_handles(FxFamilyId(id), FxActorId(0))
            .expect("phase6 actor should exist");
        let body = world
            .rigid_bodies_mut()
            .get_mut(handles.body)
            .expect("phase6 body should exist");
        body.set_position(
            Pose::from_translation(Vector::new(id as f32 * 6.0, 12.0)),
            true,
        );
    }

    place_candidate(&mut world, FxFamilyId(1), 4.0);
    place_candidate(&mut world, FxFamilyId(2), 10.0);

    let mut last = None;
    for _ in 0..8 {
        let step = world
            .step_with_diagnostics()
            .expect("phase6 step should run");
        let cap = &step.diagnostics.global_stress_cap;
        if cap.generated_commands_before_cap >= 2 || !step.report.split_events.is_empty() {
            if step.report.split_events.is_empty() {
                return Err("phase6 workload generated stress commands but no split events".into());
            }
            return Ok(step);
        }
        last = Some(step);
    }
    let _ = last.ok_or("phase6 profile should run at least one step")?;
    Err("phase6 workload did not produce a split within 8 steps".into())
}

fn markdown(step: &FxStepWithDiagnostics) -> String {
    let budget = step
        .diagnostics
        .budget
        .expect("phase6 diagnostics should include budget");
    let sync = &step.diagnostics.physics_sync;
    let cap = &step.diagnostics.global_stress_cap;
    let profile = stress_profile_totals(step);

    format!(
        "\
# Phase 6 Profile

## Gate Values

| Metric | Observed | Gate | Pass |
| --- | ---: | ---: | --- |
| Occupied voxels | {occupied_voxels} | {occupied_voxel_budget} | {occupied_pass} |
| Support graph nodes | {support_nodes} | {support_node_budget} | {support_pass} |
| Managed destructible active bodies | {active_bodies} | {active_body_budget} | {active_pass} |

## Dirty Rebuild And LOD

| Counter | Value |
| --- | ---: |
| Rebuilt colliders | {rebuilt_colliders} |
| Created actor bodies | {created_actor_bodies} |
| Removed actor bodies | {removed_actor_bodies} |
| Untouched actor count | {untouched_actor_count} |
| Primitive LOD replacements | {primitive_lod_replacements} |

## Global Stress Frame Cap

| Counter | Value |
| --- | ---: |
| Stress inputs | {stress_inputs} |
| Families with stress input | {stress_families} |
| Generated commands before global cap | {before_cap} |
| Generated commands after global cap | {after_cap} |
| Frame cap | {frame_cap} |

## Stress Profile Totals

| Counter | Value |
| --- | ---: |
| Profile input count | {profile_input_count} |
| Actor count visited | {actor_count_visited} |
| Actors with input | {actors_with_input} |
| Internal bond candidates | {internal_bond_candidates} |
| Internal bonds tested | {internal_bonds_tested} |
| External bond candidates | {external_bond_candidates} |
| External bonds tested | {external_bonds_tested} |
| Dynamic structural bonds tested | {dynamic_structural_bonds_tested} |
| Profile generated before cap sum | {profile_before} |
| Profile generated after cap sum | {profile_after} |

## Step Output

| Counter | Value |
| --- | ---: |
| Contact impulses | {contact_impulses} |
| Joint feedback | {joint_feedback} |
| Stress inputs | {step_stress_inputs} |
| Fracture events | {fracture_events} |
| Split events | {split_events} |
",
        occupied_voxels = budget.occupied_voxels,
        occupied_voxel_budget = budget.occupied_voxel_budget,
        occupied_pass = budget.occupied_voxels <= budget.occupied_voxel_budget,
        support_nodes = budget.support_nodes,
        support_node_budget = budget.support_node_budget,
        support_pass = budget.support_nodes <= budget.support_node_budget,
        active_bodies = budget.active_bodies,
        active_body_budget = budget.active_body_budget,
        active_pass = budget.active_bodies <= budget.active_body_budget,
        rebuilt_colliders = sync.rebuilt_colliders,
        created_actor_bodies = sync.created_actor_bodies,
        removed_actor_bodies = sync.removed_actor_bodies,
        untouched_actor_count = sync.untouched_actor_count,
        primitive_lod_replacements = sync.primitive_lod_replacements,
        stress_inputs = cap.input_count,
        stress_families = cap.family_count,
        before_cap = cap.generated_commands_before_cap,
        after_cap = cap.generated_commands_after_cap,
        frame_cap = cap.frame_cap,
        profile_input_count = profile.input_count,
        actor_count_visited = profile.actor_count_visited,
        actors_with_input = profile.actors_with_input,
        internal_bond_candidates = profile.internal_bond_candidates,
        internal_bonds_tested = profile.internal_bonds_tested,
        external_bond_candidates = profile.external_bond_candidates,
        external_bonds_tested = profile.external_bonds_tested,
        dynamic_structural_bonds_tested = profile.dynamic_structural_bonds_tested,
        profile_before = profile.generated_commands_before_cap,
        profile_after = profile.generated_commands_after_cap,
        contact_impulses = step.report.contact_impulses.len(),
        joint_feedback = step.report.joint_feedback.len(),
        step_stress_inputs = step.report.stress_inputs.len(),
        fracture_events = step.report.fracture_events.len(),
        split_events = step.report.split_events.len(),
    )
}

fn stress_profile_totals(step: &FxStepWithDiagnostics) -> StressProfileTotals {
    let mut totals = StressProfileTotals::default();
    for profile in &step.diagnostics.stress_profiles {
        totals.input_count += profile.input_count;
        totals.actor_count_visited += profile.actor_count_visited;
        totals.actors_with_input += profile.actors_with_input;
        totals.internal_bond_candidates += profile.internal_bond_candidates;
        totals.internal_bonds_tested += profile.internal_bonds_tested;
        totals.external_bond_candidates += profile.external_bond_candidates;
        totals.external_bonds_tested += profile.external_bonds_tested;
        totals.dynamic_structural_bonds_tested += profile.dynamic_structural_bonds_tested;
        totals.generated_commands_before_cap += profile.generated_commands_before_cap;
        totals.generated_commands_after_cap += profile.generated_commands_after_cap;
    }
    totals
}

fn main() -> Result<(), Box<dyn Error>> {
    let out = parse_out_arg()?;
    let step = run_profile()?;
    let budget = step
        .diagnostics
        .budget
        .expect("phase6 diagnostics should include budget");
    if !budget.within_budget() {
        return Err(format!(
            "Phase 6 gate failed: occupied_voxels={}/{}, support_nodes={}/{}, active_bodies={}/{}",
            budget.occupied_voxels,
            budget.occupied_voxel_budget,
            budget.support_nodes,
            budget.support_node_budget,
            budget.active_bodies,
            budget.active_body_budget
        )
        .into());
    }
    let report = markdown(&step);
    if let Some(parent) = out.parent() {
        fs::create_dir_all(parent)?;
    }
    fs::write(&out, report)?;
    println!(
        "Phase 6 gate passed: occupied_voxels={}/{}, support_nodes={}/{}, active_bodies={}/{}",
        budget.occupied_voxels,
        budget.occupied_voxel_budget,
        budget.support_nodes,
        budget.support_node_budget,
        budget.active_bodies,
        budget.active_body_budget
    );
    println!("wrote {}", out.display());
    Ok(())
}
