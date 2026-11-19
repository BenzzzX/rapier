//! Physics pipeline structures.

use crate::alloc_prelude::*;

use crate::counters::Counters;
use crate::dynamics::{
    CCDSolver, ImpulseJointSet, IntegrationParameters, IslandManager, MultibodyJointSet,
    RigidBodySet, SoftBodySet,
};
use crate::geometry::{
    BroadPhaseBvh, BroadPhasePairEvent, ColliderHandle, ColliderSet, ContactManifoldIndex,
    NarrowPhase,
};
use crate::math::{DIM, Real, Vector};
use crate::pipeline::{EventHandler, PhysicsHooks};

mod quarantine;
pub use quarantine::Quarantine;
mod solve;
mod substep;
#[cfg(test)]
mod test;
#[cfg(test)]
mod test_staged;

/// One soft-surface/rigid contact constraint from the last solver pass.
/// This is a solver observation, not proof of a continuous contact band.
#[derive(Copy, Clone, Debug)]
pub struct SoftRigidContactObservation {
    /// The soft body whose particles receive the impulse.
    pub soft_body: crate::dynamics::SoftBodyHandle,
    /// Contact pair handles, in narrow-phase order.
    pub colliders: [ColliderHandle; 2],
    /// The rigid collider's subshape as identified at constraint assembly.
    pub other_subshape: u32,
    /// Material particle indices and their interpolation weights.
    pub particles: [u32; DIM + 1],
    /// Weights correspond to `particles`; zero-weight entries are unused.
    pub weights: [Real; DIM + 1],
    /// Constraint anchors at assembly, not post-step material positions.
    pub surface_point: Vector,
    /// Other side's constraint anchor at assembly.
    pub other_point: Vector,
    /// Normal points in the direction of the impulse on the soft body.
    pub normal: Vector,
    /// Separation at assembly, with the contact skins subtracted.
    pub separation: Real,
    /// Total impulse from this solver pass, including its substeps.
    pub impulse: Vector,
    /// Friction coefficient used when assembling this particular constraint.
    pub friction: Real,
    /// True for predictive vertex contacts, false for manifold constraints.
    pub predictive_vertex: bool,
    /// Conveyor group selected for this actual solver contact, when enabled.
    #[cfg(all(feature = "dim2", feature = "experimental-soft-conveyor"))]
    pub conveyor_group: Option<u64>,
    /// Last solver substep's signed active tangential impulse. This excludes
    /// passive friction and is not a whole-step or all-CCD-passes total.
    #[cfg(all(feature = "dim2", feature = "experimental-soft-conveyor"))]
    pub conveyor_motor_impulse: Real,
    /// World-space impulses on the soft surface for the final substep only.
    #[cfg(all(feature = "dim2", feature = "experimental-soft-conveyor"))]
    pub conveyor_motor_impulse_world: Vector,
    /// Total soft-rigid contact impulse on the soft surface in the final substep.
    #[cfg(all(feature = "dim2", feature = "experimental-soft-conveyor"))]
    pub last_substep_impulse_world: Vector,
    /// Normal component of that same final-substep impulse.
    #[cfg(all(feature = "dim2", feature = "experimental-soft-conveyor"))]
    pub last_substep_normal_impulse_world: Vector,
    /// Original narrow-phase material coordinates before endpoint expansion;
    /// unavailable for predictive vertex constraints.
    #[cfg(all(feature = "dim2", feature = "experimental-soft-conveyor"))]
    pub source_material_weights: [Real; 2],
    /// Original narrow-phase separation, or NaN when unavailable.
    #[cfg(all(feature = "dim2", feature = "experimental-soft-conveyor"))]
    pub source_separation: Real,
}

/// Soft-contact observations collected after one CCD solver pass.
///
/// The ranges index the buffers returned by [`PhysicsPipeline::soft_contact_history`]
/// and [`PhysicsPipeline::soft_contact_history_bodies`]. They remain valid until
/// the next pipeline step or until recording is toggled.
#[derive(Clone, Debug)]
pub struct SoftContactSolvePass {
    /// Duration integrated by this pass, including its internal solver substeps.
    pub dt: Real,
    /// Range of soft/rigid contact rows from this pass.
    pub contacts: core::ops::Range<usize>,
    /// Range of soft bodies present in the solver workspace for this pass.
    pub solved_bodies: core::ops::Range<usize>,
    /// Linear motor rows sampled after each internal solver substep.
    pub joint_motors: core::ops::Range<usize>,
    /// Whether scalar motor sampling was valid for the solve groups in this pass.
    /// Generic multibody motor rows remain excluded.
    pub joint_motors_valid: bool,
}

/// A scalar rigid-joint linear motor row after an internal solver substep.
/// The world-space impulse is on the joint's second body; it includes warmstart
/// and relaxation. Generic multibody and angular motor rows are excluded.
#[derive(Copy, Clone, Debug)]
pub struct JointLinearMotorObservation {
    /// The active scalar joint whose row was solved.
    pub joint: crate::dynamics::ImpulseJointHandle,
    /// Joint-local linear motor axis index.
    pub axis: usize,
    /// Duration of the internal solver substep.
    pub dt: Real,
    /// World-space impulse applied to the second solver body in this substep.
    pub impulse_on_body2: Vector,
}

/// The main physics simulation engine that runs your physics world forward in time.
///
/// Think of this as the "game loop" for your physics simulation. Each frame, you call
/// [`PhysicsPipeline::step`] to advance the simulation by one timestep. This structure
/// handles all the complex physics calculations: detecting collisions between objects,
/// resolving contacts so objects don't overlap, and updating positions and velocities.
///
/// ## Performance note
/// This structure only contains temporary working memory (workspace buffers). You can create
/// a new one anytime, but it's more efficient to reuse the same instance across frames
/// since Rapier can reuse allocated memory.
///
/// ## How it works (simplified)
/// Rapier uses a time-stepping approach where each step involves:
/// 1. **Collision detection**: Find which objects are touching or overlapping
/// 2. **Constraint solving**: Calculate forces to prevent overlaps and enforce joint constraints
/// 3. **Integration**: Update object positions and velocities based on forces and gravity
/// 4. **Position correction**: Fix any remaining overlaps that might have occurred
// NOTE: this contains only workspace data, so there is no point in making this serializable.
pub struct PhysicsPipeline {
    /// Counters used for benchmarking only.
    pub counters: Counters,
    joint_constraint_indices: Vec<ContactManifoldIndex>,
    /// Whether [`Self::joint_constraint_indices`] has been filled by this pipeline yet.
    /// The joint set memoizes its selection against the buffer the caller keeps, so a
    /// pipeline that just came into existence must invalidate that memo before its first
    /// selection — otherwise it reuses a buffer it never filled.
    joint_selection_primed: bool,
    broad_phase_events: Vec<BroadPhasePairEvent>,
    /// Colliders moved by the last `advance_to_final_positions` with their fresh broad-phase
    /// AABBs, fed to the broad-phase update without the user-modification tracking. AABBs are
    /// computed inside the advance loop while body/collider are in cache.
    end_step_collider_aabbs: Vec<(ColliderHandle, crate::geometry::Aabb)>,
    /// Colliders the end-of-step soft-body sync deformed or moved to their cluster proxy's fresh
    /// pose, whose broad-phase AABBs follow them.
    soft_synced_colliders: Vec<ColliderHandle>,
    /// Non-finite state detected and neutralized during the last step.
    quarantine: Quarantine,
    /// Workspace buffer holding the active body handles (parallel body update).
    #[cfg(feature = "parallel")]
    active_body_handles: Vec<crate::dynamics::RigidBodyHandle>,
    /// Workspace: per-active-body sleep observations `(persistent island id, eligible)`,
    /// run-length compressed by the fused traversal, consumed by `IslandManager::
    /// update_islands`'s whole-island sleep decision — which never re-touches the body arena.
    sleep_observations: Vec<(u32, bool)>,
    /// The single, unified solver: the awake island is solved by its colored,
    /// staged workers. On a non-parallel (or wasm) build it runs with one worker
    /// inline on the calling thread.
    staged_solver: crate::dynamics::StagedIslandSolver,
    /// The workspace contains contacts from this step's last solver pass.
    last_soft_contact_solve: bool,
    soft_contact_history_enabled: bool,
    soft_contact_history: Vec<SoftRigidContactObservation>,
    soft_contact_history_bodies: Vec<crate::dynamics::SoftBodyHandle>,
    soft_contact_history_motors: Vec<JointLinearMotorObservation>,
    soft_contact_passes: Vec<SoftContactSolvePass>,
    /// Handle on the BVH optimization pass running concurrently with the narrow
    /// phase and solver (the `Mutex` only exists to keep the pipeline `Sync`; it is
    /// never contended).
    #[cfg(feature = "parallel")]
    deferred_bvh:
        std::sync::Mutex<Option<std::sync::mpsc::Receiver<crate::geometry::DeferredBvhOptimize>>>,
    /// Deferred BVH optimization that had no spare worker to run on (single-threaded
    /// pool, or `parallel` off): run inline by `join_deferred_bvh_optimize`, i.e. at
    /// the same point of the step where the concurrent one is joined.
    deferred_bvh_inline: Option<crate::geometry::DeferredBvhOptimize>,
    /// Pool running the parallel parts of the step (see [`Self::configure_thread_pool`]).
    /// `None` uses whichever pool the calling thread is in.
    #[cfg(all(feature = "parallel", not(feature = "unsync-callbacks")))]
    thread_pool: Option<std::sync::Arc<rayon::ThreadPool>>,
}

impl Default for PhysicsPipeline {
    fn default() -> Self {
        PhysicsPipeline::new()
    }
}

#[allow(dead_code)]
fn check_pipeline_send_sync() {
    fn do_test<T: Sync>() {}
    do_test::<PhysicsPipeline>();
}

impl PhysicsPipeline {
    /// Creates a new physics pipeline.
    ///
    /// Call this once when setting up your physics world. The pipeline can be reused
    /// across multiple frames for better performance.
    pub fn new() -> PhysicsPipeline {
        PhysicsPipeline {
            counters: Counters::new(true),
            #[cfg(feature = "parallel")]
            active_body_handles: vec![],
            sleep_observations: Vec::new(),
            staged_solver: crate::dynamics::StagedIslandSolver::new(),
            last_soft_contact_solve: false,
            soft_contact_history_enabled: false,
            soft_contact_history: Vec::new(),
            soft_contact_history_bodies: Vec::new(),
            soft_contact_history_motors: Vec::new(),
            soft_contact_passes: Vec::new(),
            #[cfg(feature = "parallel")]
            deferred_bvh: std::sync::Mutex::new(None),
            deferred_bvh_inline: None,
            #[cfg(all(feature = "parallel", not(feature = "unsync-callbacks")))]
            thread_pool: None,
            joint_constraint_indices: vec![],
            joint_selection_primed: false,
            broad_phase_events: vec![],
            end_step_collider_aabbs: vec![],
            soft_synced_colliders: vec![],
            quarantine: Quarantine::default(),
        }
    }

    /// Whether this body was included in the last pass's soft solver workspace.
    /// The result is invalidated by the next physics step.
    pub fn has_last_soft_body_solve(&self, body: crate::dynamics::SoftBodyHandle) -> bool {
        self.last_soft_contact_solve && self.staged_solver.soft_constraints.awake.iter().any(|a| a.handle == body)
    }

    /// Enables optional recording for each CCD pass of the next pipeline step.
    /// Toggling recording invalidates any previously recorded history.
    pub fn set_soft_contact_history_enabled(&mut self, enabled: bool) {
        self.soft_contact_history_enabled = enabled;
        self.staged_solver.joint_motor_history_enabled = enabled;
        self.clear_soft_contact_history();
    }

    /// Solver passes from the most recent pipeline step while recording was enabled.
    pub fn soft_contact_passes(&self) -> &[SoftContactSolvePass] {
        &self.soft_contact_passes
    }

    /// Contact rows referenced by [`Self::soft_contact_passes`]. These have the
    /// same exclusions and assembly-frame semantics as `last_soft_rigid_contacts`.
    pub fn soft_contact_history(&self) -> &[SoftRigidContactObservation] {
        &self.soft_contact_history
    }

    /// Soft bodies that participated in each recorded pass. Absence is not proof
    /// of separation, for example a sleeping body may not have been solved.
    pub fn soft_contact_history_bodies(&self) -> &[crate::dynamics::SoftBodyHandle] {
        &self.soft_contact_history_bodies
    }

    /// Scalar linear motor observations referenced by the recorded pass ranges.
    /// Missing rows do not imply that a sleeping or generic joint carried no load.
    pub fn soft_contact_history_motors(&self) -> &[JointLinearMotorObservation] {
        &self.soft_contact_history_motors
    }

    fn clear_soft_contact_history(&mut self) {
        self.soft_contact_history.clear();
        self.soft_contact_history_bodies.clear();
        self.soft_contact_history_motors.clear();
        self.soft_contact_passes.clear();
    }

    fn record_soft_contact_pass(&mut self, dt: Real) {
        if !self.soft_contact_history_enabled {
            return;
        }
        // Temporarily take the reused buffer so the iterator can borrow the
        // solver workspace without an allocation on every pass.
        let mut contacts = core::mem::take(&mut self.soft_contact_history);
        let contact_start = contacts.len();
        contacts.extend(self.last_soft_rigid_contacts());
        let contact_end = contacts.len();
        self.soft_contact_history = contacts;
        let body_start = self.soft_contact_history_bodies.len();
        if self.last_soft_contact_solve {
            self.soft_contact_history_bodies.extend(
                self.staged_solver.soft_constraints.awake.iter().map(|body| body.handle),
            );
        }
        let motor_start = self.soft_contact_history_motors.len();
        if self.last_soft_contact_solve {
            self.soft_contact_history_motors.extend_from_slice(&self.staged_solver.joint_motor_history);
        }
        self.soft_contact_passes.push(SoftContactSolvePass {
            dt,
            contacts: contact_start..contact_end,
            solved_bodies: body_start..self.soft_contact_history_bodies.len(),
            joint_motors: motor_start..self.soft_contact_history_motors.len(),
            joint_motors_valid: self.last_soft_contact_solve && self.staged_solver.joint_motor_history_valid.load(core::sync::atomic::Ordering::Relaxed),
        });
    }

    /// Iterates the last solver pass's soft-surface/rigid constraints.
    ///
    /// The workspace is replaced on the next physics step. Contacts may be
    /// speculative; sleeping bodies, other CCD passes, volume constraints and
    /// soft/soft contacts are not represented. An empty iterator does not prove
    /// separation. Anchors and directions describe assembly, not current pose.
    pub fn last_soft_rigid_contacts(&self) -> impl Iterator<Item = SoftRigidContactObservation> + '_ {
        let set = &self.staged_solver.soft_constraints;
        set.contacts.iter().filter_map(move |contact| {
            if !self.last_soft_contact_solve || contact.soft_other || contact.element.is_some() {
                return None;
            }
            let body = set.awake.get(contact.support_body as usize)?;
            let impulse = contact.reported_impulse();
            Some(SoftRigidContactObservation {
                soft_body: body.handle,
                colliders: [contact.source.collider1, contact.source.collider2],
                other_subshape: contact.source.slot,
                particles: contact.support_particle,
                weights: contact.weights,
                surface_point: contact.surface_point0,
                other_point: contact.body_point0,
                normal: contact.assembly_normal(),
                separation: contact.dist0,
                impulse,
                friction: contact.friction,
                predictive_vertex: contact.source.manifold == u32::MAX - 2,
                #[cfg(all(feature = "dim2", feature = "experimental-soft-conveyor"))]
                conveyor_group: contact.conveyor.map(|request| request.group),
                #[cfg(all(feature = "dim2", feature = "experimental-soft-conveyor"))]
                conveyor_motor_impulse: if contact.conveyor.is_some() {
                    contact.impulse_tangent[0] - contact.conveyor_passive_impulse
                } else { 0.0 },
                #[cfg(all(feature = "dim2", feature = "experimental-soft-conveyor"))]
                conveyor_motor_impulse_world: contact.tangents[0] * if contact.conveyor.is_some() {
                    contact.impulse_tangent[0] - contact.conveyor_passive_impulse
                } else { 0.0 },
                #[cfg(all(feature = "dim2", feature = "experimental-soft-conveyor"))]
                last_substep_impulse_world: contact.dir * contact.impulse_normal + contact.tangents[0] * contact.impulse_tangent[0],
                #[cfg(all(feature = "dim2", feature = "experimental-soft-conveyor"))]
                last_substep_normal_impulse_world: contact.dir * contact.impulse_normal,
                #[cfg(all(feature = "dim2", feature = "experimental-soft-conveyor"))]
                source_material_weights: contact.source_material_weights,
                #[cfg(all(feature = "dim2", feature = "experimental-soft-conveyor"))]
                source_separation: contact.source_separation,
            })
        })
    }

    /// Completes the BVH optimization pass deferred by the last broad-phase update (if
    /// any) and puts the optimized tree back into the broad-phase. Must be called before
    /// anything uses the broad-phase tree again.
    ///
    /// Waits for the concurrent pass when one was spawned; otherwise runs it here. Both
    /// paths leave the same tree behind, so the build and the pool size don't change what
    /// the rest of the step sees.
    fn join_deferred_bvh_optimize(&mut self, broad_phase: &mut BroadPhaseBvh) {
        #[cfg(feature = "parallel")]
        if let Some(rx) = self.deferred_bvh.get_mut().unwrap().take() {
            let task = rx.recv().expect("the deferred BVH optimization task died");
            broad_phase.finish_deferred_optimize(task);
            return;
        }

        if let Some(mut task) = self.deferred_bvh_inline.take() {
            task.run();
            broad_phase.finish_deferred_optimize(task);
        }
    }

    /// Advances the physics simulation by one timestep.
    ///
    /// This is the main function you'll call every frame in your game loop. It performs all
    /// physics calculations: collision detection, constraint solving, and updating object positions.
    ///
    /// # Parameters
    ///
    /// * `gravity` - The gravity vector applied to all dynamic bodies (e.g., `vector![0.0, -9.81, 0.0]` for Earth gravity pointing down)
    /// * `integration_parameters` - Controls the simulation quality and timestep size (typically 60 Hz = 1/60 second per step)
    /// * `islands` - Internal system that groups connected objects together for efficient solving (automatically managed)
    /// * `broad_phase` - Fast collision detection phase that filters out distant object pairs (automatically managed)
    /// * `narrow_phase` - Precise collision detection that computes exact contact points (automatically managed)
    /// * `bodies` - Your collection of rigid bodies (the physical objects that move and collide)
    /// * `colliders` - The collision shapes attached to your bodies (boxes, spheres, meshes, etc.)
    /// * `impulse_joints` - Regular joints connecting bodies (hinges, sliders, etc.)
    /// * `multibody_joints` - Articulated joints for robot-like structures (optional, can be empty)
    /// * `soft_bodies` - Deformable particle bodies (optional, can be empty)
    /// * `ccd_solver` - Continuous collision detection to prevent fast objects from tunneling through thin walls
    /// * `hooks` - Optional callbacks to customize collision filtering and contact modification
    /// * `events` - Optional handler to receive collision events (when objects start/stop touching)
    ///
    /// A step with a zero [`IntegrationParameters::dt`] (e.g. the first frame of a variable
    /// timestep) doesn't simulate anything: it only applies the user changes and updates the
    /// collision detection.
    ///
    /// # Example
    ///
    /// ```
    /// # use rapier3d::prelude::*;
    /// # let mut bodies = RigidBodySet::new();
    /// # let mut colliders = ColliderSet::new();
    /// # let mut impulse_joints = ImpulseJointSet::new();
    /// # let mut multibody_joints = MultibodyJointSet::new();
    /// # let mut soft_bodies = SoftBodySet::new();
    /// # let mut islands = IslandManager::new();
    /// # let mut broad_phase = BroadPhaseBvh::new();
    /// # let mut narrow_phase = NarrowPhase::new();
    /// # let mut ccd_solver = CCDSolver::new();
    /// # let mut physics_pipeline = PhysicsPipeline::new();
    /// # let integration_parameters = IntegrationParameters::default();
    /// // In your game loop:
    /// physics_pipeline.step(
    ///     Vector::new(0.0, -9.81, 0.0),  // Gravity pointing down
    ///     &integration_parameters,
    ///     &mut islands,
    ///     &mut broad_phase,
    ///     &mut narrow_phase,
    ///     &mut bodies,
    ///     &mut colliders,
    ///     &mut impulse_joints,
    ///     &mut multibody_joints,
    ///     &mut soft_bodies,
    ///     &mut ccd_solver,
    ///     &(),  // No custom hooks
    ///     &(),  // No event handler
    /// );
    /// ```
    pub fn step(
        &mut self,
        gravity: Vector,
        integration_parameters: &IntegrationParameters,
        islands: &mut IslandManager,
        broad_phase: &mut BroadPhaseBvh,
        narrow_phase: &mut NarrowPhase,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        impulse_joints: &mut ImpulseJointSet,
        multibody_joints: &mut MultibodyJointSet,
        soft_bodies: &mut SoftBodySet,
        ccd_solver: &mut CCDSolver,
        hooks: &dyn PhysicsHooks,
        events: &dyn EventHandler,
    ) {
        // With a dedicated pool configured, run the whole step inside it.
        #[cfg(all(feature = "parallel", not(feature = "unsync-callbacks")))]
        if let Some(pool) = self.thread_pool.clone() {
            return pool.install(|| {
                self.step_inner(
                    gravity,
                    integration_parameters,
                    islands,
                    broad_phase,
                    narrow_phase,
                    bodies,
                    colliders,
                    impulse_joints,
                    multibody_joints,
                    soft_bodies,
                    ccd_solver,
                    hooks,
                    events,
                )
            });
        }

        self.step_inner(
            gravity,
            integration_parameters,
            islands,
            broad_phase,
            narrow_phase,
            bodies,
            colliders,
            impulse_joints,
            multibody_joints,
            soft_bodies,
            ccd_solver,
            hooks,
            events,
        )
    }
}

#[cfg(all(feature = "parallel", not(feature = "unsync-callbacks")))]
impl PhysicsPipeline {
    /// Configures a dedicated thread pool for this pipeline's parallel work (default:
    /// whichever pool the calling thread is in — usually the global one or the one
    /// setup with `ThreadPool::install`.
    pub fn configure_thread_pool(
        &mut self,
        num_threads: usize,
    ) -> Result<(), rayon::ThreadPoolBuildError> {
        let builder = rayon::ThreadPoolBuilder::new()
            .num_threads(num_threads)
            .thread_name(|i| alloc::format!("rapier-worker-{i}"));

        self.thread_pool = Some(std::sync::Arc::new(builder.build()?));
        Ok(())
    }

    /// The thread-pool used by this physics pipeline, if it was configured.
    pub fn thread_pool(&self) -> Option<std::sync::Arc<rayon::ThreadPool>> {
        self.thread_pool.clone()
    }

    /// Sets (or clears) the thread pool running this pipeline's parallel work.
    ///
    /// Unlike [`Self::configure_thread_pool`], this takes an existing pool.
    pub fn set_thread_pool(&mut self, pool: Option<std::sync::Arc<rayon::ThreadPool>>) {
        self.thread_pool = pool;
    }

    /// Removes the dedicated thread pool: the parallel parts of the step run on whichever
    /// pool the calling thread is in again.
    pub fn clear_thread_pool(&mut self) {
        self.thread_pool = None;
    }

    /// The number of workers this pipeline's parallel work runs on: the size of its
    /// dedicated thread pool, or of the pool the calling thread is in if it has none.
    pub fn num_threads(&self) -> Option<usize> {
        self.thread_pool
            .as_ref()
            .map(|pool| pool.current_num_threads())
    }
}
