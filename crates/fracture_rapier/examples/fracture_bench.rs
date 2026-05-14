use std::env;
use std::error::Error;
use std::fs;
use std::hint::black_box;
use std::path::PathBuf;
use std::time::{Duration, Instant};

use fracture_core::{
    CommandId, DamageSource, DeterministicOrderKey, FxActorId, FxFamily, FxFamilyId, StressInput,
    StressProfile, StressSettings, StressSolver2D, Vec2,
};
use fracture_rapier::{FxPerformanceBudgetReport, FxPhysicsSyncReport, FxRapierWorld2D};
use fracture_voxel::{
    AuthoredVoxelAsset, NaturalVoronoi, VoxelAuthoringInput, VoxelAuthoringOptions,
    VoxelClusterMode, VoxelClusterPolicy, author_voxel_asset, author_voxel_asset_with_options,
};
use rapier2d::prelude::*;

const STRESS_FAMILY: FxFamilyId = FxFamilyId(1);
const PHASE6_FAMILY_COUNT: u32 = 99;
const WORLD_STRESS_VORONOI_FAMILY_COUNT: u32 = 8;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Scenario {
    AuthorSmall,
    AuthorLarge,
    StressGrid,
    StressVoronoi100,
    WorldStressVoronoi100,
    FractureApplyGrid,
    WorldIdleStep,
    WorldFractureStep,
}

impl Scenario {
    fn all() -> &'static [Self] {
        &[
            Self::StressGrid,
            Self::StressVoronoi100,
            Self::WorldStressVoronoi100,
            Self::FractureApplyGrid,
            Self::WorldFractureStep,
            Self::WorldIdleStep,
            Self::AuthorLarge,
            Self::AuthorSmall,
        ]
    }

    fn name(self) -> &'static str {
        match self {
            Self::AuthorSmall => "author-small-grid",
            Self::AuthorLarge => "author-large-voronoi-100",
            Self::StressGrid => "stress-grid-solver",
            Self::StressVoronoi100 => "stress-voronoi-100",
            Self::WorldStressVoronoi100 => "world-stress-voronoi-100",
            Self::FractureApplyGrid => "fracture-apply-grid",
            Self::WorldIdleStep => "world-idle-step",
            Self::WorldFractureStep => "world-fracture-step",
        }
    }

    fn default_iterations(self) -> usize {
        match self {
            Self::AuthorSmall => 50,
            Self::AuthorLarge => 1,
            Self::StressGrid => 50,
            Self::StressVoronoi100 => 20,
            Self::WorldStressVoronoi100 => 4,
            Self::FractureApplyGrid => 1,
            Self::WorldIdleStep => 32,
            Self::WorldFractureStep => 1,
        }
    }

    fn parse(value: &str) -> Option<Self> {
        match value {
            "author-small" | "author-small-grid" => Some(Self::AuthorSmall),
            "author-large"
            | "author-large-grid"
            | "author-large-voronoi"
            | "author-large-voronoi-100" => Some(Self::AuthorLarge),
            "stress" | "stress-grid" | "stress-grid-solver" => Some(Self::StressGrid),
            "stress-voronoi" | "stress-voronoi-100" => Some(Self::StressVoronoi100),
            "world-stress" | "world-stress-voronoi" | "world-stress-voronoi-100" => {
                Some(Self::WorldStressVoronoi100)
            }
            "fracture-apply" | "fracture-apply-grid" => Some(Self::FractureApplyGrid),
            "world-idle" | "world-idle-step" => Some(Self::WorldIdleStep),
            "world-fracture" | "world-fracture-step" => Some(Self::WorldFractureStep),
            _ => None,
        }
    }
}

#[derive(Clone, Debug)]
struct Args {
    samples: usize,
    warmup: usize,
    iterations_override: Option<usize>,
    scenario: Option<Scenario>,
    out: Option<PathBuf>,
    csv: Option<PathBuf>,
}

impl Default for Args {
    fn default() -> Self {
        Self {
            samples: 30,
            warmup: 5,
            iterations_override: None,
            scenario: None,
            out: None,
            csv: None,
        }
    }
}

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

impl StressProfileTotals {
    fn absorb(&mut self, profile: StressProfile) {
        self.input_count += profile.input_count;
        self.actor_count_visited += profile.actor_count_visited;
        self.actors_with_input += profile.actors_with_input;
        self.internal_bond_candidates += profile.internal_bond_candidates;
        self.internal_bonds_tested += profile.internal_bonds_tested;
        self.external_bond_candidates += profile.external_bond_candidates;
        self.external_bonds_tested += profile.external_bonds_tested;
        self.dynamic_structural_bonds_tested += profile.dynamic_structural_bonds_tested;
        self.generated_commands_before_cap += profile.generated_commands_before_cap;
        self.generated_commands_after_cap += profile.generated_commands_after_cap;
    }
}

#[derive(Clone, Debug, Default)]
struct WorkloadMetrics {
    occupied_voxels: usize,
    support_nodes: usize,
    internal_bonds: usize,
    active_bodies: usize,
    stress_inputs: usize,
    stress_profiles: StressProfileTotals,
    generated_commands: usize,
    fracture_events: usize,
    split_events: usize,
    physics_sync: FxPhysicsSyncReport,
    budget: Option<FxPerformanceBudgetReport>,
    checksum: u64,
}

#[derive(Clone, Copy, Debug)]
struct SampleStats {
    min_ms: f64,
    mean_ms: f64,
    p50_ms: f64,
    p95_ms: f64,
    max_ms: f64,
}

#[derive(Clone, Debug)]
struct BenchResult {
    scenario: Scenario,
    samples: usize,
    warmup: usize,
    iterations_per_sample: usize,
    total_sample_time: Duration,
    stats: SampleStats,
    metrics: WorkloadMetrics,
}

fn parse_args() -> Result<Args, Box<dyn Error>> {
    let mut args = Args::default();
    let mut raw = env::args().skip(1);

    while let Some(arg) = raw.next() {
        match arg.as_str() {
            "--samples" => args.samples = parse_usize_arg("--samples", raw.next())?,
            "--warmup" => args.warmup = parse_usize_arg("--warmup", raw.next())?,
            "--iters" => {
                args.iterations_override = Some(parse_usize_arg("--iters", raw.next())?);
            }
            "--scenario" => {
                let value = raw.next().ok_or("--scenario requires a value")?;
                args.scenario = Some(
                    Scenario::parse(&value).ok_or_else(|| format!("unknown scenario `{value}`"))?,
                );
            }
            "--out" => args.out = Some(PathBuf::from(raw.next().ok_or("--out requires a path")?)),
            "--csv" => args.csv = Some(PathBuf::from(raw.next().ok_or("--csv requires a path")?)),
            "-h" | "--help" => {
                print_help();
                std::process::exit(0);
            }
            other => return Err(format!("unknown argument `{other}`").into()),
        }
    }

    if args.samples == 0 {
        return Err("--samples must be greater than 0".into());
    }

    if matches!(args.iterations_override, Some(0)) {
        return Err("--iters must be greater than 0".into());
    }

    Ok(args)
}

fn parse_usize_arg(name: &str, value: Option<String>) -> Result<usize, Box<dyn Error>> {
    value
        .ok_or_else(|| format!("{name} requires a value"))?
        .parse::<usize>()
        .map_err(|err| format!("invalid {name}: {err}").into())
}

fn print_help() {
    println!(
        "\
Usage: fracture_bench [options]

Options:
  --scenario <name>   One of: stress, stress-voronoi, fracture-apply,
                      world-stress, world-fracture, world-idle,
                      author-small, author-large.
                      author-large is a 100x100 generated NaturalVoronoi policy workload.
                      world-stress steps multiple 100x100 Voronoi families through FxRapierWorld2D.
                      Omit to run all scenarios.
  --samples <n>       Timed samples per scenario. Default: 30.
  --warmup <n>        Untimed warmup samples per scenario. Default: 5.
  --iters <n>         Override iterations per sample for every scenario.
  --out <path>        Write Markdown report.
  --csv <path>        Write CSV summary.
"
    );
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = parse_args()?;
    let scenarios = match args.scenario {
        Some(scenario) => vec![scenario],
        None => Scenario::all().to_vec(),
    };

    let mut results = Vec::new();
    for scenario in scenarios {
        let iterations = args
            .iterations_override
            .unwrap_or_else(|| scenario.default_iterations());
        println!(
            "running {}: samples={}, warmup={}, iters/sample={}",
            scenario.name(),
            args.samples,
            args.warmup,
            iterations
        );
        results.push(run_scenario(
            scenario,
            args.samples,
            args.warmup,
            iterations,
        )?);
    }

    let markdown = render_markdown(&results);
    if let Some(out) = &args.out {
        write_file(out, &markdown)?;
        println!("wrote {}", out.display());
    } else {
        println!("{markdown}");
    }

    if let Some(csv) = &args.csv {
        write_file(csv, &render_csv(&results))?;
        println!("wrote {}", csv.display());
    }

    Ok(())
}

fn write_file(path: &PathBuf, contents: &str) -> Result<(), Box<dyn Error>> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)?;
    }
    fs::write(path, contents)?;
    Ok(())
}

fn run_scenario(
    scenario: Scenario,
    samples: usize,
    warmup: usize,
    iterations: usize,
) -> Result<BenchResult, Box<dyn Error>> {
    match scenario {
        Scenario::AuthorSmall => benchmark(
            scenario,
            samples,
            warmup,
            iterations,
            || Ok(authoring_state(14, 14, 1, 1)),
            |state, iterations| measure_authoring(state, iterations),
        ),
        Scenario::AuthorLarge => benchmark(
            scenario,
            samples,
            warmup,
            iterations,
            || Ok(voronoi_authoring_state_100()),
            |state, iterations| measure_authoring(state, iterations),
        ),
        Scenario::StressGrid => benchmark(
            scenario,
            samples,
            warmup,
            iterations,
            || build_stress_state(14),
            |state, iterations| measure_stress_solver(state, iterations),
        ),
        Scenario::StressVoronoi100 => benchmark(
            scenario,
            samples,
            warmup,
            iterations,
            build_voronoi_stress_state_100,
            |state, iterations| measure_stress_solver(state, iterations),
        ),
        Scenario::WorldStressVoronoi100 => benchmark(
            scenario,
            samples,
            warmup,
            iterations,
            build_world_stress_voronoi_state_100,
            |world, iterations| measure_world_stress_voronoi_step(world, iterations),
        ),
        Scenario::FractureApplyGrid => benchmark(
            scenario,
            samples,
            warmup,
            iterations,
            build_fracture_apply_grid_state,
            |state, iterations| measure_fracture_apply(state, iterations),
        ),
        Scenario::WorldIdleStep => benchmark(
            scenario,
            samples,
            warmup,
            iterations,
            || build_world_idle_state(),
            |world, iterations| measure_world_idle_step(world, iterations),
        ),
        Scenario::WorldFractureStep => benchmark(
            scenario,
            samples,
            warmup,
            iterations,
            || build_phase6_world(),
            |world, iterations| measure_world_fracture_step(world, iterations),
        ),
    }
}

fn benchmark<T, FSetup, FMeasure>(
    scenario: Scenario,
    samples: usize,
    warmup: usize,
    iterations: usize,
    mut setup: FSetup,
    mut measure: FMeasure,
) -> Result<BenchResult, Box<dyn Error>>
where
    FSetup: FnMut() -> Result<T, Box<dyn Error>>,
    FMeasure: FnMut(&mut T, usize) -> Result<WorkloadMetrics, Box<dyn Error>>,
{
    for _ in 0..warmup {
        let mut state = setup()?;
        black_box(measure(&mut state, iterations)?);
    }

    let mut durations = Vec::with_capacity(samples);
    let mut last_metrics = WorkloadMetrics::default();
    let mut total_sample_time = Duration::ZERO;

    for _ in 0..samples {
        let mut state = setup()?;
        let start = Instant::now();
        last_metrics = measure(&mut state, iterations)?;
        let elapsed = start.elapsed();
        black_box(last_metrics.checksum);
        total_sample_time += elapsed;
        durations.push(elapsed.as_secs_f64() * 1000.0 / iterations as f64);
    }

    durations.sort_by(|a, b| a.total_cmp(b));
    let stats = sample_stats(&durations);
    Ok(BenchResult {
        scenario,
        samples,
        warmup,
        iterations_per_sample: iterations,
        total_sample_time,
        stats,
        metrics: last_metrics,
    })
}

fn sample_stats(sorted_ms: &[f64]) -> SampleStats {
    let min_ms = sorted_ms[0];
    let max_ms = sorted_ms[sorted_ms.len() - 1];
    let mean_ms = sorted_ms.iter().sum::<f64>() / sorted_ms.len() as f64;
    SampleStats {
        min_ms,
        mean_ms,
        p50_ms: percentile(sorted_ms, 0.50),
        p95_ms: percentile(sorted_ms, 0.95),
        max_ms,
    }
}

fn percentile(sorted: &[f64], percentile: f64) -> f64 {
    let index = ((sorted.len() - 1) as f64 * percentile).ceil() as usize;
    sorted[index]
}

#[derive(Clone)]
struct AuthoringState {
    input: VoxelAuthoringInput,
    options: VoxelAuthoringOptions,
}

struct StressState {
    solver: StressSolver2D,
    family: FxFamily,
    inputs: Vec<StressInput>,
}

struct FractureApplyState {
    world: FxRapierWorld2D,
    commands: Vec<fracture_core::FractureCommand>,
    occupied_voxels: usize,
    support_nodes: usize,
    internal_bonds: usize,
}

fn authoring_state(width: u32, height: u32, max_extent: u32, max_voxels: usize) -> AuthoringState {
    AuthoringState {
        input: grid_authoring_input(width, height, 1.0, 1),
        options: VoxelAuthoringOptions::default()
            .with_material_rule(1, VoxelClusterPolicy::isotropic(max_extent, max_voxels)),
    }
}

fn voronoi_authoring_state_100() -> AuthoringState {
    AuthoringState {
        input: grid_authoring_input(100, 100, 1.0, 1),
        options: VoxelAuthoringOptions::default().with_material_rule(
            1,
            VoxelClusterPolicy {
                mode: VoxelClusterMode::NaturalVoronoi(
                    NaturalVoronoi::generated(196, 0xF2AC_7100).with_noise(0x51A7_5EED, 384),
                ),
            },
        ),
    }
}

fn voronoi_asset_100() -> Result<AuthoredVoxelAsset, Box<dyn Error>> {
    let state = voronoi_authoring_state_100();
    Ok(author_voxel_asset_with_options(state.input, state.options)?)
}

fn measure_authoring(
    state: &mut AuthoringState,
    iterations: usize,
) -> Result<WorkloadMetrics, Box<dyn Error>> {
    let mut metrics = WorkloadMetrics::default();
    for _ in 0..iterations {
        let asset = author_voxel_asset_with_options(state.input.clone(), state.options.clone())?;
        metrics = asset_metrics(&asset);
        metrics.checksum = metrics
            .checksum
            .wrapping_add(asset.core().support_nodes().len() as u64)
            .wrapping_add(asset.core().internal_bonds().len() as u64);
        black_box(asset);
    }
    Ok(metrics)
}

fn build_stress_state(size: u32) -> Result<StressState, Box<dyn Error>> {
    let asset = stress_grid_asset(size)?;
    let family = FxFamily::instantiate(STRESS_FAMILY, asset.core().clone());
    let inputs = right_edge_stress_inputs(&family, size);
    Ok(StressState {
        solver: StressSolver2D::new(StressSettings {
            damage_per_overload: 2.0,
            max_fractures_per_frame: u16::MAX,
            ..StressSettings::default()
        }),
        family,
        inputs,
    })
}

fn build_voronoi_stress_state_100() -> Result<StressState, Box<dyn Error>> {
    let asset = voronoi_asset_100()?;
    let family = FxFamily::instantiate(STRESS_FAMILY, asset.core().clone());
    let inputs = right_edge_stress_inputs(&family, 100);
    Ok(StressState {
        solver: StressSolver2D::new(StressSettings {
            damage_per_overload: 2.0,
            max_fractures_per_frame: u16::MAX,
            ..StressSettings::default()
        }),
        family,
        inputs,
    })
}

fn measure_stress_solver(
    state: &mut StressState,
    iterations: usize,
) -> Result<WorkloadMetrics, Box<dyn Error>> {
    let mut metrics = WorkloadMetrics {
        occupied_voxels: state.family.asset().occupancy().occupied_voxels().count(),
        support_nodes: state.family.asset().support_nodes().len(),
        internal_bonds: state.family.asset().internal_bonds().len(),
        stress_inputs: state.inputs.len(),
        ..WorkloadMetrics::default()
    };

    for _ in 0..iterations {
        let report = state
            .solver
            .generate_with_profile(&state.family, &state.inputs);
        metrics.generated_commands += report.commands.len();
        metrics.stress_profiles.absorb(report.profile);
        metrics.checksum = metrics
            .checksum
            .wrapping_add(report.commands.len() as u64)
            .wrapping_add(report.profile.internal_bonds_tested as u64)
            .wrapping_add(report.profile.generated_commands_before_cap as u64);
        black_box(report);
    }

    Ok(metrics)
}

fn build_fracture_apply_grid_state() -> Result<FractureApplyState, Box<dyn Error>> {
    let asset = stress_grid_asset(14)?;
    let occupied_voxels = asset.metrics().occupied_voxels;
    let support_nodes = asset.metrics().support_nodes;
    let internal_bonds = asset.core().internal_bonds().len();
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(Vector::ZERO);
    world.set_stress_settings(StressSettings {
        damage_per_overload: 2.0,
        max_fractures_per_frame: u16::MAX,
        ..StressSettings::default()
    });
    world.add_destructible(STRESS_FAMILY, asset)?;

    let family = world
        .family(STRESS_FAMILY)
        .ok_or("fracture apply family should exist")?;
    let solver = StressSolver2D::new(StressSettings {
        damage_per_overload: 2.0,
        max_fractures_per_frame: u16::MAX,
        ..StressSettings::default()
    });
    let inputs = right_edge_stress_inputs(family, 14);
    let commands = solver.generate(family, &inputs);
    if commands.is_empty() {
        return Err("fracture apply workload generated no commands".into());
    }

    Ok(FractureApplyState {
        world,
        commands,
        occupied_voxels,
        support_nodes,
        internal_bonds,
    })
}

fn measure_fracture_apply(
    state: &mut FractureApplyState,
    iterations: usize,
) -> Result<WorkloadMetrics, Box<dyn Error>> {
    if iterations != 1 {
        return Err("fracture-apply mutates its world; run it with --iters 1".into());
    }

    let step = state
        .world
        .apply_fracture_commands_to_family(STRESS_FAMILY, &state.commands)?;
    let mut metrics = WorkloadMetrics {
        occupied_voxels: state.occupied_voxels,
        support_nodes: state.support_nodes,
        internal_bonds: state.internal_bonds,
        generated_commands: state.commands.len(),
        fracture_events: step.report.fracture_events.len(),
        split_events: step.report.split_events.len(),
        active_bodies: step
            .diagnostics
            .budget
            .map(|budget| budget.active_bodies)
            .unwrap_or_else(|| state.world.performance_budget_report().active_bodies),
        budget: step.diagnostics.budget,
        ..WorkloadMetrics::default()
    };
    absorb_sync_report(&mut metrics.physics_sync, &step.diagnostics.physics_sync);
    metrics.checksum = metrics
        .checksum
        .wrapping_add(metrics.generated_commands as u64)
        .wrapping_add(metrics.fracture_events as u64)
        .wrapping_add(metrics.split_events as u64)
        .wrapping_add(metrics.physics_sync.rebuilt_colliders as u64)
        .wrapping_add(metrics.active_bodies as u64);
    black_box(step);
    Ok(metrics)
}

fn build_world_idle_state() -> Result<FxRapierWorld2D, Box<dyn Error>> {
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(Vector::ZERO);
    world.set_stress_settings(StressSettings {
        max_fractures_per_frame: 8,
        ..StressSettings::default()
    });
    world.add_destructible(STRESS_FAMILY, stress_grid_asset(14)?)?;
    Ok(world)
}

fn measure_world_idle_step(
    world: &mut FxRapierWorld2D,
    iterations: usize,
) -> Result<WorkloadMetrics, Box<dyn Error>> {
    let mut metrics = WorkloadMetrics::default();
    for _ in 0..iterations {
        let step = world.step_with_diagnostics()?;
        metrics.absorb_step(world, &step);
        metrics.internal_bonds = world
            .family(STRESS_FAMILY)
            .map(|family| family.asset().internal_bonds().len())
            .unwrap_or(0);
        black_box(step);
    }
    Ok(metrics)
}

fn build_world_stress_voronoi_state_100() -> Result<FxRapierWorld2D, Box<dyn Error>> {
    let asset = voronoi_asset_100()?;
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(Vector::ZERO);
    world.set_stress_settings(StressSettings {
        damage_per_overload: 2.0,
        max_fractures_per_frame: 8,
        ..StressSettings::default()
    });

    for id in 1..=WORLD_STRESS_VORONOI_FAMILY_COUNT {
        let family_id = FxFamilyId(id);
        world.add_destructible(family_id, asset.clone())?;
        translate_family_actors(&mut world, family_id, Vector::new(id as f32 * 140.0, 0.0))?;
    }

    Ok(world)
}

fn measure_world_stress_voronoi_step(
    world: &mut FxRapierWorld2D,
    iterations: usize,
) -> Result<WorkloadMetrics, Box<dyn Error>> {
    let mut metrics = WorkloadMetrics::default();
    for _ in 0..iterations {
        let step = world.step_with_diagnostics()?;
        let profile_work = step
            .diagnostics
            .stress_profiles
            .iter()
            .fold(0u64, |total, profile| {
                total
                    .wrapping_add(profile.actor_count_visited as u64)
                    .wrapping_add(profile.internal_bonds_tested as u64)
                    .wrapping_add(profile.external_bonds_tested as u64)
            });
        metrics.absorb_step(world, &step);
        metrics.internal_bonds = (1..=WORLD_STRESS_VORONOI_FAMILY_COUNT)
            .filter_map(|id| world.family(FxFamilyId(id)))
            .map(|family| family.asset().internal_bonds().len())
            .sum();
        metrics.checksum = metrics.checksum.wrapping_add(profile_work);
        black_box(step);
    }
    Ok(metrics)
}

fn build_phase6_world() -> Result<FxRapierWorld2D, Box<dyn Error>> {
    let mut world = FxRapierWorld2D::new();
    world.set_gravity(Vector::ZERO);
    world.set_stress_settings(StressSettings {
        damage_per_overload: 2.0,
        max_fractures_per_frame: 1,
        ..StressSettings::default()
    });

    for id in 1..=PHASE6_FAMILY_COUNT {
        world.add_destructible(FxFamilyId(id), two_node_asset(7)?)?;
    }

    for id in 1..=PHASE6_FAMILY_COUNT {
        let handles = world
            .actor_handles(FxFamilyId(id), FxActorId(0))
            .ok_or("phase6 actor should exist")?;
        let body = world
            .rigid_bodies_mut()
            .get_mut(handles.body)
            .ok_or("phase6 body should exist")?;
        body.set_position(
            Pose::from_translation(Vector::new(id as f32 * 6.0, 12.0)),
            true,
        );
    }

    place_candidate(&mut world, FxFamilyId(1), 4.0)?;
    place_candidate(&mut world, FxFamilyId(2), 10.0)?;
    Ok(world)
}

fn measure_world_fracture_step(
    world: &mut FxRapierWorld2D,
    iterations: usize,
) -> Result<WorkloadMetrics, Box<dyn Error>> {
    let mut metrics = WorkloadMetrics::default();
    for _ in 0..iterations {
        let mut split_seen = false;
        for _ in 0..8 {
            let step = world.step_with_diagnostics()?;
            split_seen |= !step.report.split_events.is_empty();
            metrics.absorb_step(world, &step);
            metrics.internal_bonds = (1..=PHASE6_FAMILY_COUNT)
                .filter_map(|id| world.family(FxFamilyId(id)))
                .map(|family| family.asset().internal_bonds().len())
                .sum();
            black_box(step);
            if split_seen {
                break;
            }
        }
        if !split_seen {
            return Err("phase6 fracture workload did not split within 8 steps".into());
        }
    }
    Ok(metrics)
}

impl WorkloadMetrics {
    fn absorb_step(
        &mut self,
        world: &FxRapierWorld2D,
        step: &fracture_rapier::FxStepWithDiagnostics,
    ) {
        let budget = step
            .diagnostics
            .budget
            .unwrap_or_else(|| world.performance_budget_report());
        self.occupied_voxels = budget.occupied_voxels;
        self.support_nodes = budget.support_nodes;
        self.active_bodies = budget.active_bodies;
        self.budget = Some(budget);
        self.stress_inputs += step.report.stress_inputs.len();
        self.fracture_events += step.report.fracture_events.len();
        self.split_events += step.report.split_events.len();
        self.generated_commands += step
            .diagnostics
            .global_stress_cap
            .generated_commands_after_cap;
        absorb_sync_report(&mut self.physics_sync, &step.diagnostics.physics_sync);
        for profile in &step.diagnostics.stress_profiles {
            self.stress_profiles.absorb(*profile);
        }
        self.checksum = self
            .checksum
            .wrapping_add(step.report.stress_inputs.len() as u64)
            .wrapping_add(step.report.fracture_events.len() as u64)
            .wrapping_add(step.report.split_events.len() as u64)
            .wrapping_add(
                step.diagnostics
                    .global_stress_cap
                    .generated_commands_after_cap as u64,
            )
            .wrapping_add(budget.active_bodies as u64);
    }
}

fn absorb_sync_report(total: &mut FxPhysicsSyncReport, step: &FxPhysicsSyncReport) {
    total.created_actor_bodies += step.created_actor_bodies;
    total.removed_actor_bodies += step.removed_actor_bodies;
    total.rebuilt_colliders += step.rebuilt_colliders;
    total.untouched_actor_count += step.untouched_actor_count;
    total.primitive_lod_replacements += step.primitive_lod_replacements;
    total
        .impulse_joint_handle_replacements
        .extend(step.impulse_joint_handle_replacements.iter().copied());
}

fn asset_metrics(asset: &AuthoredVoxelAsset) -> WorkloadMetrics {
    let voxel_metrics = asset.metrics();
    WorkloadMetrics {
        occupied_voxels: voxel_metrics.occupied_voxels,
        support_nodes: voxel_metrics.support_nodes,
        internal_bonds: asset.core().internal_bonds().len(),
        checksum: voxel_metrics.occupied_voxels as u64
            ^ ((voxel_metrics.support_nodes as u64) << 16)
            ^ ((asset.core().internal_bonds().len() as u64) << 32),
        ..WorkloadMetrics::default()
    }
}

fn stress_grid_asset(size: u32) -> Result<AuthoredVoxelAsset, Box<dyn Error>> {
    let mut input = grid_authoring_input(size, size, 1.0, 1);
    input.default_bond_health = 1.0;
    input.default_tension_limit = 0.01;
    input.default_shear_limit = 0.01;
    let options =
        VoxelAuthoringOptions::default().with_material_rule(1, VoxelClusterPolicy::isotropic(1, 1));
    Ok(author_voxel_asset_with_options(input, options)?)
}

fn grid_authoring_input(
    width: u32,
    height: u32,
    voxel_size: f32,
    material: u16,
) -> VoxelAuthoringInput {
    let count = width as usize * height as usize;
    VoxelAuthoringInput::new(
        width,
        height,
        voxel_size,
        vec![true; count],
        vec![material; count],
        vec![7; count],
        (0..count as u32).collect(),
    )
}

fn right_edge_stress_inputs(family: &FxFamily, size: u32) -> Vec<StressInput> {
    family
        .asset()
        .support_nodes()
        .iter()
        .filter(|node| node.voxels.iter().any(|voxel| voxel.x + 1 == size))
        .enumerate()
        .filter_map(|(index, node)| {
            let actor = family.node_owner(node.id)?;
            Some(StressInput {
                order_key: DeterministicOrderKey::new(
                    0,
                    30,
                    family.id,
                    actor,
                    CommandId(index as u32),
                ),
                actor,
                node: node.id,
                force: Vec2::new(80.0, -20.0),
                source: DamageSource::Stress,
            })
        })
        .collect()
}

fn two_node_asset(contact_material: u16) -> Result<AuthoredVoxelAsset, Box<dyn Error>> {
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
    Ok(author_voxel_asset(input)?)
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

fn translate_family_actors(
    world: &mut FxRapierWorld2D,
    family_id: FxFamilyId,
    translation: Vector,
) -> Result<(), Box<dyn Error>> {
    let actor_ids = world
        .family(family_id)
        .ok_or("world stress family should exist")?
        .actors()
        .map(|(actor, _)| *actor)
        .collect::<Vec<_>>();

    for actor in actor_ids {
        let handles = world
            .actor_handles(family_id, actor)
            .ok_or("world stress actor should exist")?;
        let body = world
            .rigid_bodies_mut()
            .get_mut(handles.body)
            .ok_or("world stress body should exist")?;
        let mut position = *body.position();
        position.translation += translation;
        body.set_position(position, true);
    }

    Ok(())
}

fn place_candidate(
    world: &mut FxRapierWorld2D,
    family: FxFamilyId,
    x: f32,
) -> Result<(), Box<dyn Error>> {
    let handles = world
        .actor_handles(family, FxActorId(0))
        .ok_or("candidate actor should exist")?;
    let body = world
        .rigid_bodies_mut()
        .get_mut(handles.body)
        .ok_or("candidate body should exist")?;
    body.set_position(
        Pose::from_parts(Vector::new(x, 3.0), Rotation::new(0.25)),
        true,
    );
    body.set_linvel(Vector::new(1.25, -6.0), true);
    body.set_angvel(1.5, true);
    body.enable_ccd(true);
    add_fixed_box(world, Vector::new(x + 0.48, 2.55), Vector::new(0.45, 0.1));
    Ok(())
}

fn render_markdown(results: &[BenchResult]) -> String {
    let mut out = String::from(
        "# Fracture Benchmark Report\n\n\
All timings are milliseconds per measured iteration. Run this example with `--release` for useful numbers.\n\n\
| Scenario | Samples | Warmup | Iters/sample | Min ms | Mean ms | P50 ms | P95 ms | Max ms |\n\
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |\n",
    );

    for result in results {
        out.push_str(&format!(
            "| {} | {} | {} | {} | {:.4} | {:.4} | {:.4} | {:.4} | {:.4} |\n",
            result.scenario.name(),
            result.samples,
            result.warmup,
            result.iterations_per_sample,
            result.stats.min_ms,
            result.stats.mean_ms,
            result.stats.p50_ms,
            result.stats.p95_ms,
            result.stats.max_ms,
        ));
    }

    out.push_str("\n## Workload Metrics\n\n");
    out.push_str(
        "| Scenario | Voxels | Nodes | Bonds | Active bodies | Stress inputs | Commands | Fractures | Splits | Rebuilt colliders | Primitive LOD | Total sample s |\n\
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |\n",
    );
    for result in results {
        let metrics = &result.metrics;
        out.push_str(&format!(
            "| {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {:.3} |\n",
            result.scenario.name(),
            metrics.occupied_voxels,
            metrics.support_nodes,
            metrics.internal_bonds,
            metrics.active_bodies,
            metrics.stress_inputs,
            metrics.generated_commands,
            metrics.fracture_events,
            metrics.split_events,
            metrics.physics_sync.rebuilt_colliders,
            metrics.physics_sync.primitive_lod_replacements,
            result.total_sample_time.as_secs_f64(),
        ));
    }

    out.push_str("\n## Stress Profile Totals From Last Sample\n\n");
    out.push_str(
        "| Scenario | Profile inputs | Actors visited | Actors with input | Internal candidates | Internal tested | External candidates | External tested | Dynamic tested | Before cap | After cap |\n\
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |\n",
    );
    for result in results {
        let profile = result.metrics.stress_profiles;
        out.push_str(&format!(
            "| {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} |\n",
            result.scenario.name(),
            profile.input_count,
            profile.actor_count_visited,
            profile.actors_with_input,
            profile.internal_bond_candidates,
            profile.internal_bonds_tested,
            profile.external_bond_candidates,
            profile.external_bonds_tested,
            profile.dynamic_structural_bonds_tested,
            profile.generated_commands_before_cap,
            profile.generated_commands_after_cap,
        ));
    }

    out
}

fn render_csv(results: &[BenchResult]) -> String {
    let mut out = String::from(
        "scenario,samples,warmup,iters_per_sample,min_ms,mean_ms,p50_ms,p95_ms,max_ms,occupied_voxels,support_nodes,internal_bonds,active_bodies,stress_inputs,commands,fractures,splits,rebuilt_colliders,primitive_lod,total_sample_s\n",
    );
    for result in results {
        let metrics = &result.metrics;
        out.push_str(&format!(
            "{},{},{},{},{:.6},{:.6},{:.6},{:.6},{:.6},{},{},{},{},{},{},{},{},{},{},{:.6}\n",
            result.scenario.name(),
            result.samples,
            result.warmup,
            result.iterations_per_sample,
            result.stats.min_ms,
            result.stats.mean_ms,
            result.stats.p50_ms,
            result.stats.p95_ms,
            result.stats.max_ms,
            metrics.occupied_voxels,
            metrics.support_nodes,
            metrics.internal_bonds,
            metrics.active_bodies,
            metrics.stress_inputs,
            metrics.generated_commands,
            metrics.fracture_events,
            metrics.split_events,
            metrics.physics_sync.rebuilt_colliders,
            metrics.physics_sync.primitive_lod_replacements,
            result.total_sample_time.as_secs_f64(),
        ));
    }
    out
}
