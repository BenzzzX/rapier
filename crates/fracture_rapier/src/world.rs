use std::collections::{BTreeMap, BTreeSet};

use fracture_core::{
    ConnectionError, ConnectionId, DynamicConnectionPolicy, ExternalBondId, FractureCommand,
    FxActorId, FxFamily, FxFamilyId, GridCoord, MergeActorsResult, SplitEvent, StressSettings,
    StressSolver2D, SupportNodeId, Vec2, apply_fracture_commands,
    snapshot::{SnapshotMode, encode_family_snapshot, restore_family_snapshot},
    sort_fracture_commands, split_dirty_actors,
};
use fracture_voxel::AuthoredVoxelAsset;
use rapier2d::prelude::*;
use thiserror::Error;

use crate::collider_sync::{
    ActorColliderBuildKind, ActorPhysicsHandles, ColliderLodSettings, DestructibleActorRef,
    FxPhysicsSyncReport, ImpulseJointHandleReplacement, VoxelContact, actor_body_builder_at,
    actor_collider_build, actor_default_contact_material,
};
use crate::connect_api::{
    DynamicStructuralConnectionDesc, StaticAnchorBodyPolicy, StaticAnchorConnectionDesc,
};
use crate::contact_map::{collider_key, rigid_body_key};
use crate::hooks::{ContactMaterialProperties, FxContactHooks, HookObservation};
use crate::impulse_readback::collect_contact_impulse_inputs;
use crate::joint_feedback::collect_joint_feedback_stress;
use crate::pipeline::{
    ACTIVE_BODY_BUDGET, FxPerformanceBudgetReport, FxStepDiagnostics, FxStepReport,
    FxStepWithDiagnostics, OCCUPIED_VOXEL_BUDGET, SUPPORT_NODE_BUDGET,
};
use crate::replay::{FxRapierReplayCommand, FxRapierReplayTickReport, sort_replay_commands};
use crate::snapshot::{
    ActorPhysicsSnapshot, AppliedStaticAnchorPolicySnapshot, BodyActorSnapshot,
    BodyBaselineSnapshot, BodyTypeSnapshot, ColliderActorSnapshot, ColliderVoxelSnapshot,
    FxRapierFamilySnapshot, FxRapierSnapshotError, FxRapierWorldSnapshot,
    StaticAnchorPolicySnapshot, StressSettingsSnapshot, VoxelContactSnapshot,
    decode_rapier_owned_state, decode_world_snapshot, encode_rapier_owned_state,
    encode_world_snapshot,
};

#[derive(Debug, Error, PartialEq)]
pub enum FxRapierError {
    #[error("family {0:?} already exists")]
    DuplicateFamily(FxFamilyId),
    #[error("unknown family {0:?}")]
    UnknownFamily(FxFamilyId),
    #[error("unknown actor {actor:?} in family {family:?}")]
    UnknownActor {
        family: FxFamilyId,
        actor: FxActorId,
    },
    #[error(
        "missing split parent snapshot for child {child:?} from parent {parent:?} in family {family:?}"
    )]
    MissingSplitParentSnapshot {
        family: FxFamilyId,
        parent: FxActorId,
        child: FxActorId,
    },
    #[error(transparent)]
    Connection(#[from] ConnectionError),
    #[error("connection policy {0:?} is reserved for a future Rapier adapter implementation")]
    UnsupportedConnectionPolicy(DynamicConnectionPolicy),
    #[error("deterministic replay API requires deterministic snapshot mode")]
    ReplayRequiresDeterministicMode,
    #[error("replay command references unknown family {0:?}")]
    UnknownReplayFamily(FxFamilyId),
    #[error("duplicate ambiguous replay key at tick {tick} stable_order {stable_order}")]
    DuplicateReplayKey { tick: u64, stable_order: u64 },
    #[error(transparent)]
    Snapshot(#[from] FxRapierSnapshotError),
}

#[derive(Clone, Debug)]
pub(crate) struct DestructibleFamily {
    pub(crate) asset: AuthoredVoxelAsset,
    pub(crate) family: FxFamily,
    pub(crate) physics: BTreeMap<FxActorId, ActorPhysicsState>,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct ActorPhysicsState {
    pub(crate) handles: ActorPhysicsHandles,
    pub(crate) body_local_origin_in_asset: Vec2,
}

#[derive(Clone, Copy, Debug)]
enum VoxelMetadataValidation {
    Exact,
    Captured,
}

impl VoxelMetadataValidation {
    fn accepts(self, actual: &[VoxelContact], expected: &[VoxelContact]) -> bool {
        match self {
            Self::Exact => actual == expected,
            Self::Captured => {
                let mut expected_contacts = BTreeSet::new();
                for contact in expected {
                    expected_contacts.insert(voxel_contact_key(contact));
                }
                let mut actual_contacts = BTreeSet::new();
                !actual.is_empty()
                    && actual.iter().all(|contact| {
                        let key = voxel_contact_key(contact);
                        expected_contacts.contains(&key) && actual_contacts.insert(key)
                    })
            }
        }
    }
}

fn voxel_contact_key(contact: &VoxelContact) -> (u32, u32, u32, u16, u32) {
    (
        contact.coord.x,
        contact.coord.y,
        contact.node.0,
        contact.contact_material,
        contact.subshape,
    )
}

pub struct FxRapierWorld2D {
    pub(crate) gravity: Vector,
    pub(crate) integration_parameters: IntegrationParameters,
    pipeline: PhysicsPipeline,
    islands: IslandManager,
    broad_phase: BroadPhaseBvh,
    narrow_phase: NarrowPhase,
    pub(crate) bodies: RigidBodySet,
    pub(crate) colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd_solver: CCDSolver,
    hooks: FxContactHooks,
    stress_solver: StressSolver2D,
    lod_settings: ColliderLodSettings,
    pub(crate) families: BTreeMap<FxFamilyId, DestructibleFamily>,
    body_actors: BTreeMap<(u32, u32), DestructibleActorRef>,
    static_anchor_policies: BTreeMap<(FxFamilyId, ExternalBondId), StaticAnchorBodyPolicy>,
    applied_static_anchor_policies: BTreeMap<(FxFamilyId, FxActorId), StaticAnchorBodyPolicy>,
    static_anchor_body_baselines: BTreeMap<(FxFamilyId, FxActorId), RigidBodyType>,
    pub(crate) snapshot_mode: SnapshotMode,
    pub(crate) tick: u64,
}

#[derive(Clone, Copy, Debug)]
struct BodySnapshot {
    position: Pose,
    world_center_of_mass: Vector,
    linvel: Vector,
    angvel: f32,
    ccd_enabled: bool,
    soft_ccd_prediction: f32,
    was_sleeping: bool,
    body_local_origin_in_asset: Vec2,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum JointEndpoint {
    Body1,
    Body2,
}

#[derive(Clone, Debug)]
struct SplitJointEndpointRemap {
    joint: ImpulseJointHandle,
    endpoint: JointEndpoint,
    old_body: RigidBodyHandle,
    new_actor: FxActorId,
    asset_anchor: Vec2,
}

impl Default for FxRapierWorld2D {
    fn default() -> Self {
        Self::new()
    }
}

impl FxRapierWorld2D {
    pub fn new() -> Self {
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
            hooks: FxContactHooks::new(),
            stress_solver: StressSolver2D::new(StressSettings::default()),
            lod_settings: ColliderLodSettings::default(),
            families: BTreeMap::new(),
            body_actors: BTreeMap::new(),
            static_anchor_policies: BTreeMap::new(),
            applied_static_anchor_policies: BTreeMap::new(),
            static_anchor_body_baselines: BTreeMap::new(),
            snapshot_mode: SnapshotMode::Normal,
            tick: 0,
        }
    }

    pub fn set_stress_settings(&mut self, settings: StressSettings) {
        self.stress_solver = StressSolver2D::new(settings);
    }

    pub fn stress_settings(&self) -> StressSettings {
        self.stress_solver.settings
    }

    pub fn lod_settings(&self) -> ColliderLodSettings {
        self.lod_settings
    }

    #[cfg(test)]
    pub(crate) fn set_lod_settings(&mut self, settings: ColliderLodSettings) {
        self.lod_settings = settings;
    }

    pub fn performance_budget_report(&self) -> FxPerformanceBudgetReport {
        let (occupied_voxels, support_nodes) =
            self.families
                .values()
                .fold((0usize, 0usize), |(occupied, support), entry| {
                    let metrics = entry.asset.metrics();
                    (
                        occupied + metrics.occupied_voxels,
                        support + metrics.support_nodes,
                    )
                });
        FxPerformanceBudgetReport {
            occupied_voxels,
            occupied_voxel_budget: OCCUPIED_VOXEL_BUDGET,
            support_nodes,
            support_node_budget: SUPPORT_NODE_BUDGET,
            active_bodies: self
                .families
                .values()
                .map(|entry| entry.physics.len())
                .sum(),
            active_body_budget: ACTIVE_BODY_BUDGET,
        }
    }

    pub fn validate_performance_budget(
        &self,
    ) -> Result<FxPerformanceBudgetReport, FxPerformanceBudgetReport> {
        let report = self.performance_budget_report();
        if report.within_budget() {
            Ok(report)
        } else {
            Err(report)
        }
    }

    pub fn gravity(&self) -> Vector {
        self.gravity
    }

    pub fn set_gravity(&mut self, gravity: Vector) {
        self.gravity = gravity;
    }

    pub fn integration_parameters(&self) -> &IntegrationParameters {
        &self.integration_parameters
    }

    pub fn integration_parameters_mut(&mut self) -> &mut IntegrationParameters {
        &mut self.integration_parameters
    }

    pub fn snapshot_mode(&self) -> SnapshotMode {
        self.snapshot_mode
    }

    pub fn set_snapshot_mode(&mut self, mode: SnapshotMode) {
        self.snapshot_mode = mode;
        if mode == SnapshotMode::Deterministic {
            self.lod_settings = ColliderLodSettings::disabled();
        }
    }

    pub fn tick(&self) -> u64 {
        self.tick
    }

    pub fn snapshot(&self) -> Result<Vec<u8>, FxRapierError> {
        ensure_snapshot_mode_available(self.snapshot_mode)?;
        ensure_deterministic_lod_disabled(self.snapshot_mode, self.lod_settings)?;
        Ok(encode_world_snapshot(&self.capture_snapshot()?))
    }

    pub fn restore_snapshot(bytes: &[u8]) -> Result<Self, FxRapierError> {
        let (_, snapshot) = decode_world_snapshot(bytes)?;
        ensure_snapshot_mode_available(snapshot.mode)?;
        Self::from_snapshot(snapshot)
    }

    pub fn rigid_bodies(&self) -> &RigidBodySet {
        &self.bodies
    }

    pub fn rigid_bodies_mut(&mut self) -> &mut RigidBodySet {
        &mut self.bodies
    }

    pub fn colliders(&self) -> &ColliderSet {
        &self.colliders
    }

    pub fn colliders_mut(&mut self) -> &mut ColliderSet {
        &mut self.colliders
    }

    pub fn insert_rigid_body(&mut self, body: impl Into<RigidBody>) -> RigidBodyHandle {
        self.bodies.insert(body)
    }

    pub fn insert_collider_with_parent(
        &mut self,
        collider: impl Into<Collider>,
        parent: RigidBodyHandle,
    ) -> ColliderHandle {
        self.colliders
            .insert_with_parent(collider.into(), parent, &mut self.bodies)
    }

    pub fn insert_impulse_joint(
        &mut self,
        body1: RigidBodyHandle,
        body2: RigidBodyHandle,
        data: impl Into<GenericJoint>,
        wake_up: bool,
    ) -> ImpulseJointHandle {
        self.impulse_joints.insert(body1, body2, data, wake_up)
    }

    pub fn impulse_joints(&self) -> &ImpulseJointSet {
        &self.impulse_joints
    }

    pub fn impulse_joints_mut(&mut self) -> &mut ImpulseJointSet {
        &mut self.impulse_joints
    }

    pub fn narrow_phase(&self) -> &NarrowPhase {
        &self.narrow_phase
    }

    pub fn drain_contact_hook_observations(&self) -> Vec<HookObservation> {
        self.hooks.drain_observations()
    }

    #[cfg(test)]
    pub(crate) fn contact_registry_snapshot(&self) -> crate::hooks::ContactMaterialRegistry {
        self.hooks
            .registry()
            .read()
            .expect("contact material registry poisoned")
            .clone()
    }

    pub fn set_contact_material_properties(
        &self,
        material: u16,
        properties: ContactMaterialProperties,
    ) {
        self.hooks.set_material_properties(material, properties);
    }

    pub fn add_destructible(
        &mut self,
        family_id: FxFamilyId,
        asset: AuthoredVoxelAsset,
    ) -> Result<(), FxRapierError> {
        if self.families.contains_key(&family_id) {
            return Err(FxRapierError::DuplicateFamily(family_id));
        }
        let family = FxFamily::instantiate(family_id, asset.core().clone());
        self.families.insert(
            family_id,
            DestructibleFamily {
                asset,
                family,
                physics: BTreeMap::new(),
            },
        );
        self.sync_family_actors(family_id)
    }

    pub fn family(&self, family_id: FxFamilyId) -> Option<&FxFamily> {
        self.families.get(&family_id).map(|entry| &entry.family)
    }

    pub fn actor_handles(
        &self,
        family: FxFamilyId,
        actor: FxActorId,
    ) -> Option<ActorPhysicsHandles> {
        self.families
            .get(&family)
            .and_then(|entry| entry.physics.get(&actor).map(|state| state.handles))
    }

    pub fn actor_handles_result(
        &self,
        family: FxFamilyId,
        actor: FxActorId,
    ) -> Result<ActorPhysicsHandles, FxRapierError> {
        self.actor_handles(family, actor)
            .ok_or(FxRapierError::UnknownActor { family, actor })
    }

    pub fn connect_static_anchor(
        &mut self,
        family_id: FxFamilyId,
        desc: StaticAnchorConnectionDesc,
    ) -> Result<ExternalBondId, FxRapierError> {
        self.sync_family_actors(family_id)?;
        let actor = self
            .families
            .get(&family_id)
            .ok_or(FxRapierError::UnknownFamily(family_id))?
            .family
            .node_owner(desc.core.node)
            .ok_or(ConnectionError::UnknownNode(desc.core.node))?;
        if desc.body_policy != StaticAnchorBodyPolicy::Preserve {
            self.actor_handles_result(family_id, actor)?;
        }
        let id = self
            .families
            .get_mut(&family_id)
            .ok_or(FxRapierError::UnknownFamily(family_id))?
            .family
            .connect_static_anchor(desc.core)?;
        if desc.body_policy != StaticAnchorBodyPolicy::Preserve {
            self.static_anchor_policies
                .insert((family_id, id), desc.body_policy);
        }
        self.reconcile_static_anchor_body_policies(family_id)?;
        Ok(id)
    }

    pub fn connect_dynamic_structural_bond(
        &mut self,
        family_id: FxFamilyId,
        desc: DynamicStructuralConnectionDesc,
    ) -> Result<ConnectionId, FxRapierError> {
        if desc.policy == DynamicConnectionPolicy::CustomHardConstraint {
            return Err(FxRapierError::UnsupportedConnectionPolicy(desc.policy));
        }
        self.sync_family_actors(family_id)?;
        let before_joints = self.impulse_joints.len();
        let id = self
            .families
            .get_mut(&family_id)
            .ok_or(FxRapierError::UnknownFamily(family_id))?
            .family
            .connect_dynamic_structural_bond_graph_only(desc.core)?;
        debug_assert_eq!(self.impulse_joints.len(), before_joints);
        Ok(id)
    }

    pub fn merge_actors(
        &mut self,
        family_id: FxFamilyId,
        actor_a: FxActorId,
        actor_b: FxActorId,
    ) -> Result<MergeActorsResult, FxRapierError> {
        self.sync_family_actors(family_id)?;
        let snapshots = self.snapshot_family_bodies(family_id);
        let snapshot_a = snapshots
            .get(&actor_a)
            .copied()
            .ok_or(FxRapierError::UnknownActor {
                family: family_id,
                actor: actor_a,
            })?;
        let snapshot_b = snapshots
            .get(&actor_b)
            .copied()
            .ok_or(FxRapierError::UnknownActor {
                family: family_id,
                actor: actor_b,
            })?;
        let handles_a = self.actor_handles_result(family_id, actor_a)?;
        let handles_b = self.actor_handles_result(family_id, actor_b)?;
        let mass_a = self
            .bodies
            .get(handles_a.body)
            .ok_or(FxRapierError::UnknownActor {
                family: family_id,
                actor: actor_a,
            })?
            .mass();
        let mass_b = self
            .bodies
            .get(handles_b.body)
            .ok_or(FxRapierError::UnknownActor {
                family: family_id,
                actor: actor_b,
            })?
            .mass();
        let result = self
            .families
            .get_mut(&family_id)
            .ok_or(FxRapierError::UnknownFamily(family_id))?
            .family
            .merge_actors(actor_a, actor_b)?;
        let merged_local_origin = self
            .families
            .get(&family_id)
            .and_then(|entry| entry.family.actor(result.kept_actor))
            .ok_or(FxRapierError::UnknownActor {
                family: family_id,
                actor: result.kept_actor,
            })?
            .local_com;
        let kept_snapshot = if result.kept_actor == actor_a {
            snapshot_a
        } else {
            snapshot_b
        };
        let merged_snapshot = merged_body_snapshot(
            kept_snapshot,
            snapshot_a,
            mass_a,
            snapshot_b,
            mass_b,
            merged_local_origin,
        );
        self.rebuild_actor_collider_preserving_body(
            family_id,
            result.kept_actor,
            Some(merged_snapshot),
        )?;
        self.set_actor_total_mass(family_id, result.kept_actor, mass_a + mass_b)?;
        self.set_actor_world_center_of_mass(
            family_id,
            result.kept_actor,
            merged_snapshot.world_center_of_mass,
        )?;
        self.remove_actor_handles(family_id, result.removed_actor);
        self.reconcile_static_anchor_body_policies(family_id)?;
        Ok(result)
    }

    pub fn step(&mut self) -> Result<FxStepReport, FxRapierError> {
        Ok(self.step_with_diagnostics()?.into_report())
    }

    pub fn step_with_diagnostics(&mut self) -> Result<FxStepWithDiagnostics, FxRapierError> {
        let family_ids = self.families.keys().copied().collect::<Vec<_>>();
        for family_id in family_ids {
            self.sync_family_actors(family_id)?;
        }

        self.hooks.clear_pre_solver_contact_cache();
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
            &self.hooks,
            &(),
        );

        let families = self
            .families
            .iter()
            .map(|(id, entry)| (*id, &entry.family))
            .collect::<Vec<_>>();
        let registry = self.hooks.registry();
        let registry = registry
            .read()
            .expect("contact material registry poisoned")
            .clone();
        let pre_solver_contact_cache = self.hooks.pre_solver_contact_cache_snapshot();
        let contact_readback = collect_contact_impulse_inputs(
            self.tick,
            self.integration_parameters.dt,
            &self.narrow_phase,
            &families,
            &registry,
            &pre_solver_contact_cache,
        );
        let joint_feedback = collect_joint_feedback_stress(
            self.tick,
            self.integration_parameters.dt,
            &self.impulse_joints,
            &self.body_actors,
            &families,
        );
        let contact_impulses = contact_readback.inputs;
        let contact_impulse_readback_miss_count = contact_readback.cache_misses.len();

        let mut report = FxStepReport {
            stress_inputs: contact_impulses
                .iter()
                .map(|input| input.stress.clone())
                .chain(
                    joint_feedback
                        .iter()
                        .map(|feedback| feedback.stress.clone()),
                )
                .collect(),
            contact_impulses,
            joint_feedback,
            ..FxStepReport::default()
        };
        let mut diagnostics = FxStepDiagnostics::default();
        diagnostics.contact_impulse_readback_miss_count = contact_impulse_readback_miss_count;

        self.apply_step_stress_inputs(&mut report, &mut diagnostics)?;

        diagnostics.budget = Some(self.performance_budget_report());
        self.tick += 1;
        Ok(FxStepWithDiagnostics {
            report,
            diagnostics,
        })
    }

    fn apply_step_stress_inputs(
        &mut self,
        report: &mut FxStepReport,
        diagnostics: &mut FxStepDiagnostics,
    ) -> Result<(), FxRapierError> {
        let family_ids = self.families.keys().copied().collect::<Vec<_>>();
        let global_cap = self.stress_solver.settings.max_fractures_per_frame;
        let mut uncapped_settings = self.stress_solver.settings;
        uncapped_settings.max_fractures_per_frame = u16::MAX;
        let uncapped_solver = StressSolver2D::new(uncapped_settings);
        let mut family_profiles = BTreeMap::new();
        let mut candidate_commands = Vec::new();
        let mut input_family_count = 0usize;

        for family_id in &family_ids {
            let stress_inputs = report
                .stress_inputs
                .iter()
                .filter(|input| input.order_key.family_id == *family_id)
                .cloned()
                .collect::<Vec<_>>();
            if !stress_inputs.is_empty() {
                input_family_count += 1;
            }
            let Some(entry) = self.families.get(family_id) else {
                return Err(FxRapierError::UnknownFamily(*family_id));
            };
            let stress_report =
                uncapped_solver.generate_with_profile(&entry.family, &stress_inputs);
            let mut profile = stress_report.profile;
            profile.frame_cap = global_cap;
            profile.generated_commands_after_cap = 0;
            family_profiles.insert(*family_id, profile);
            candidate_commands.extend(stress_report.commands);
        }

        sort_fracture_commands(&mut candidate_commands);
        let generated_commands_before_cap = candidate_commands.len();
        candidate_commands.truncate(global_cap as usize);
        let generated_commands_after_cap = candidate_commands.len();

        let mut selected_by_family: BTreeMap<FxFamilyId, Vec<FractureCommand>> = BTreeMap::new();
        for command in candidate_commands {
            selected_by_family
                .entry(command.order_key.family_id)
                .or_default()
                .push(command);
        }
        for (family_id, commands) in &selected_by_family {
            if let Some(profile) = family_profiles.get_mut(family_id) {
                profile.generated_commands_after_cap = commands.len();
            }
        }

        diagnostics.global_stress_cap.input_count = report.stress_inputs.len();
        diagnostics.global_stress_cap.family_count = input_family_count;
        diagnostics.global_stress_cap.generated_commands_before_cap = generated_commands_before_cap;
        diagnostics.global_stress_cap.generated_commands_after_cap = generated_commands_after_cap;
        diagnostics.global_stress_cap.frame_cap = global_cap;
        diagnostics.stress_profiles.extend(
            family_ids
                .iter()
                .filter_map(|id| family_profiles.remove(id)),
        );

        for family_id in family_ids {
            let parent_snapshots = self.snapshot_family_bodies(family_id);
            let split_events = {
                let Some(entry) = self.families.get_mut(&family_id) else {
                    return Err(FxRapierError::UnknownFamily(family_id));
                };
                if let Some(commands) = selected_by_family.get(&family_id) {
                    report
                        .fracture_events
                        .extend(apply_fracture_commands(&mut entry.family, commands));
                }
                split_dirty_actors(&mut entry.family)
            };
            if !split_events.is_empty() {
                let sync_report =
                    self.sync_split_family_actors(family_id, &parent_snapshots, &split_events)?;
                report.impulse_joint_handle_replacements.extend(
                    sync_report
                        .impulse_joint_handle_replacements
                        .iter()
                        .copied(),
                );
                diagnostics.physics_sync.absorb(sync_report);
            } else {
                self.reconcile_static_anchor_body_policies(family_id)?;
            }
            report.split_events.extend(split_events);
        }

        Ok(())
    }

    #[cfg(test)]
    pub(crate) fn step_with_stress_inputs_for_test(
        &mut self,
        stress_inputs: Vec<fracture_core::StressInput>,
    ) -> Result<FxStepWithDiagnostics, FxRapierError> {
        let mut report = FxStepReport {
            stress_inputs,
            ..FxStepReport::default()
        };
        let mut diagnostics = FxStepDiagnostics::default();
        self.apply_step_stress_inputs(&mut report, &mut diagnostics)?;
        diagnostics.budget = Some(self.performance_budget_report());
        self.tick += 1;
        Ok(FxStepWithDiagnostics {
            report,
            diagnostics,
        })
    }

    pub fn apply_fracture_commands_to_family(
        &mut self,
        family_id: FxFamilyId,
        commands: &[fracture_core::FractureCommand],
    ) -> Result<FxStepWithDiagnostics, FxRapierError> {
        self.sync_family_actors(family_id)?;
        let parent_snapshots = self.snapshot_family_bodies(family_id);
        let split_events = {
            let Some(entry) = self.families.get_mut(&family_id) else {
                return Err(FxRapierError::UnknownFamily(family_id));
            };
            let fracture_events = apply_fracture_commands(&mut entry.family, commands);
            let split_events = split_dirty_actors(&mut entry.family);
            (fracture_events, split_events)
        };
        let (fracture_events, split_events) = split_events;
        let mut report = FxStepReport {
            fracture_events,
            split_events: split_events.clone(),
            ..FxStepReport::default()
        };
        let mut diagnostics = FxStepDiagnostics::default();
        if !split_events.is_empty() {
            let sync_report =
                self.sync_split_family_actors(family_id, &parent_snapshots, &split_events)?;
            report.impulse_joint_handle_replacements.extend(
                sync_report
                    .impulse_joint_handle_replacements
                    .iter()
                    .copied(),
            );
            diagnostics.physics_sync.absorb(sync_report);
        } else {
            self.reconcile_static_anchor_body_policies(family_id)?;
        }
        diagnostics.budget = Some(self.performance_budget_report());
        Ok(FxStepWithDiagnostics {
            report,
            diagnostics,
        })
    }

    pub fn apply_replay_tick(
        &mut self,
        tick: u64,
        commands: &[FxRapierReplayCommand],
    ) -> Result<FxRapierReplayTickReport, FxRapierError> {
        if self.snapshot_mode != SnapshotMode::Deterministic {
            return Err(FxRapierError::ReplayRequiresDeterministicMode);
        }
        ensure_snapshot_mode_available(self.snapshot_mode)?;
        ensure_deterministic_lod_disabled(self.snapshot_mode, self.lod_settings)?;
        crate::replay::validate_replay_commands(commands)?;
        for family_id in self.family_ids() {
            self.sync_family_actors(family_id)?;
        }

        let mut selected = commands
            .iter()
            .filter(|entry| entry.tick == tick)
            .cloned()
            .collect::<Vec<_>>();
        sort_replay_commands(&mut selected);
        let mut commands_by_family =
            BTreeMap::<FxFamilyId, Vec<fracture_core::FractureCommand>>::new();
        for entry in selected {
            if !self.families.contains_key(&entry.family) {
                return Err(FxRapierError::UnknownReplayFamily(entry.family));
            }
            commands_by_family
                .entry(entry.family)
                .or_default()
                .push(entry.command);
        }

        let mut report = FxRapierReplayTickReport::default();
        for family_id in self.family_ids() {
            let Some(commands) = commands_by_family.get(&family_id) else {
                continue;
            };
            let parent_snapshots = self.snapshot_family_bodies(family_id);
            let split_events = {
                let Some(entry) = self.families.get_mut(&family_id) else {
                    return Err(FxRapierError::UnknownFamily(family_id));
                };
                report
                    .fracture_events
                    .extend(apply_fracture_commands(&mut entry.family, commands));
                split_dirty_actors(&mut entry.family)
            };
            if !split_events.is_empty() {
                let sync_report =
                    self.sync_split_family_actors(family_id, &parent_snapshots, &split_events)?;
                report
                    .impulse_joint_handle_replacements
                    .extend(sync_report.impulse_joint_handle_replacements);
            } else {
                self.reconcile_static_anchor_body_policies(family_id)?;
            }
            report.split_events.extend(split_events);
        }
        Ok(report)
    }

    #[cfg(test)]
    pub(crate) fn fracture_and_sync_for_test(
        &mut self,
        family_id: FxFamilyId,
        commands: &[fracture_core::FractureCommand],
    ) -> Result<Vec<SplitEvent>, FxRapierError> {
        self.sync_family_actors(family_id)?;
        let parent_snapshots = self.snapshot_family_bodies(family_id);
        let split_events = {
            let Some(entry) = self.families.get_mut(&family_id) else {
                return Err(FxRapierError::UnknownFamily(family_id));
            };
            apply_fracture_commands(&mut entry.family, commands);
            split_dirty_actors(&mut entry.family)
        };
        if !split_events.is_empty() {
            let _sync_report =
                self.sync_split_family_actors(family_id, &parent_snapshots, &split_events)?;
        } else {
            self.reconcile_static_anchor_body_policies(family_id)?;
        }
        Ok(split_events)
    }

    #[cfg(test)]
    pub(crate) fn fracture_and_sync_report_for_test(
        &mut self,
        family_id: FxFamilyId,
        commands: &[fracture_core::FractureCommand],
    ) -> Result<(Vec<SplitEvent>, FxPhysicsSyncReport), FxRapierError> {
        self.sync_family_actors(family_id)?;
        let parent_snapshots = self.snapshot_family_bodies(family_id);
        let split_events = {
            let Some(entry) = self.families.get_mut(&family_id) else {
                return Err(FxRapierError::UnknownFamily(family_id));
            };
            apply_fracture_commands(&mut entry.family, commands);
            split_dirty_actors(&mut entry.family)
        };
        let sync_report = if !split_events.is_empty() {
            self.sync_split_family_actors(family_id, &parent_snapshots, &split_events)?
        } else {
            self.reconcile_static_anchor_body_policies(family_id)?;
            FxPhysicsSyncReport::default()
        };
        Ok((split_events, sync_report))
    }

    #[cfg(test)]
    pub(crate) fn fracture_and_sync_without_parent_snapshot_for_test(
        &mut self,
        family_id: FxFamilyId,
        commands: &[fracture_core::FractureCommand],
        missing_parent: FxActorId,
    ) -> Result<Vec<SplitEvent>, FxRapierError> {
        self.sync_family_actors(family_id)?;
        let mut parent_snapshots = self.snapshot_family_bodies(family_id);
        parent_snapshots.remove(&missing_parent);
        let split_events = {
            let Some(entry) = self.families.get_mut(&family_id) else {
                return Err(FxRapierError::UnknownFamily(family_id));
            };
            apply_fracture_commands(&mut entry.family, commands);
            split_dirty_actors(&mut entry.family)
        };
        if !split_events.is_empty() {
            let _sync_report =
                self.sync_split_family_actors(family_id, &parent_snapshots, &split_events)?;
        } else {
            self.reconcile_static_anchor_body_policies(family_id)?;
        }
        Ok(split_events)
    }

    #[cfg(test)]
    pub(crate) fn family_ids_for_test(&self) -> Vec<FxFamilyId> {
        self.families.keys().copied().collect()
    }

    #[cfg(test)]
    pub(crate) fn fracture_all_and_sync_for_test(
        &mut self,
        commands_by_family: &[(FxFamilyId, Vec<fracture_core::FractureCommand>)],
    ) -> Result<Vec<SplitEvent>, FxRapierError> {
        for family_id in self.family_ids_for_test() {
            self.sync_family_actors(family_id)?;
        }

        let commands_by_family = commands_by_family
            .iter()
            .map(|(family, commands)| (*family, commands.as_slice()))
            .collect::<BTreeMap<_, _>>();
        let mut out = Vec::new();
        for family_id in self.family_ids_for_test() {
            let Some(commands) = commands_by_family.get(&family_id) else {
                continue;
            };
            let parent_snapshots = self.snapshot_family_bodies(family_id);
            let split_events = {
                let Some(entry) = self.families.get_mut(&family_id) else {
                    return Err(FxRapierError::UnknownFamily(family_id));
                };
                apply_fracture_commands(&mut entry.family, commands);
                split_dirty_actors(&mut entry.family)
            };
            if !split_events.is_empty() {
                let _sync_report =
                    self.sync_split_family_actors(family_id, &parent_snapshots, &split_events)?;
            } else {
                self.reconcile_static_anchor_body_policies(family_id)?;
            }
            out.extend(split_events);
        }
        Ok(out)
    }

    fn family_ids(&self) -> Vec<FxFamilyId> {
        self.families.keys().copied().collect()
    }

    fn capture_snapshot(&self) -> Result<FxRapierWorldSnapshot, FxRapierError> {
        let registry = self
            .hooks
            .registry()
            .read()
            .expect("contact material registry poisoned")
            .clone();
        self.validate_checkpoint_state(&registry, VoxelMetadataValidation::Exact)?;
        let mut families = Vec::new();
        let mut actor_physics = Vec::new();
        for (family_id, entry) in &self.families {
            for (actor_id, state) in &entry.physics {
                if entry.family.actor(*actor_id).is_none() {
                    return Err(FxRapierSnapshotError::StateMismatch(
                        "actor physics unknown actor",
                    )
                    .into());
                }
                if self.bodies.get(state.handles.body).is_none() {
                    return Err(FxRapierSnapshotError::StateMismatch(
                        "actor references missing body",
                    )
                    .into());
                }
                if self.colliders.get(state.handles.collider).is_none() {
                    return Err(FxRapierSnapshotError::StateMismatch(
                        "actor references missing collider",
                    )
                    .into());
                }
                let body_key = rigid_body_key(state.handles.body);
                let collider_key = collider_key(state.handles.collider);
                let actor_ref = DestructibleActorRef {
                    family: *family_id,
                    actor: *actor_id,
                };
                if self.body_actors.get(&body_key) != Some(&actor_ref)
                    || registry.collider_actors.get(&collider_key) != Some(&actor_ref)
                    || !registry.collider_voxels.contains_key(&collider_key)
                {
                    return Err(
                        FxRapierSnapshotError::StateMismatch("actor registry mismatch").into(),
                    );
                }
                actor_physics.push(ActorPhysicsSnapshot {
                    family: *family_id,
                    actor: *actor_id,
                    body_handle: state.handles.body.into_raw_parts(),
                    collider_handle: state.handles.collider.into_raw_parts(),
                    body_local_origin_in_asset: [
                        state.body_local_origin_in_asset.x,
                        state.body_local_origin_in_asset.y,
                    ],
                });
            }
            families.push(FxRapierFamilySnapshot {
                family: *family_id,
                asset: fracture_voxel::AuthoredVoxelAssetSnapshot {
                    bytes: entry
                        .asset
                        .to_snapshot_bytes()
                        .map_err(FxRapierSnapshotError::Voxel)?,
                },
                core_family: encode_family_snapshot(&entry.family, self.snapshot_mode)
                    .map_err(FxRapierSnapshotError::Core)?,
            });
        }
        families.sort_by_key(|entry| entry.family);
        actor_physics.sort_by_key(|item| (item.family, item.actor));
        let rapier_owned_state = encode_rapier_owned_state(
            &self.bodies,
            &self.colliders,
            &self.impulse_joints,
            &self.multibody_joints,
            &self.islands,
            &self.broad_phase,
            &self.narrow_phase,
            &self.ccd_solver,
        )
        .map_err(FxRapierSnapshotError::from)?;

        Ok(FxRapierWorldSnapshot {
            mode: self.snapshot_mode,
            tick: self.tick,
            gravity: [self.gravity.x, self.gravity.y],
            integration: integration_to_snapshot(&self.integration_parameters),
            stress: stress_to_snapshot(self.stress_solver.settings),
            rapier_owned_state,
            contact_materials: registry
                .material_properties
                .iter()
                .map(|(k, v)| (*k, *v))
                .collect(),
            families,
            actor_physics,
            body_actors: self
                .body_actors
                .iter()
                .map(|(body_handle, actor)| BodyActorSnapshot {
                    body_handle: *body_handle,
                    actor: *actor,
                })
                .collect(),
            collider_actors: registry
                .collider_actors
                .iter()
                .map(|(collider_handle, actor)| ColliderActorSnapshot {
                    collider_handle: *collider_handle,
                    actor: *actor,
                })
                .collect(),
            collider_voxels: registry
                .collider_voxels
                .iter()
                .map(|(collider_handle, voxels)| ColliderVoxelSnapshot {
                    collider_handle: *collider_handle,
                    voxels: voxels
                        .iter()
                        .copied()
                        .map(VoxelContactSnapshot::from)
                        .collect(),
                })
                .collect(),
            static_anchor_policies: self
                .static_anchor_policies
                .iter()
                .map(|((family, bond), policy)| StaticAnchorPolicySnapshot {
                    family: *family,
                    bond: *bond,
                    policy: *policy,
                })
                .collect(),
            applied_static_anchor_policies: self
                .applied_static_anchor_policies
                .iter()
                .map(
                    |((family, actor), policy)| AppliedStaticAnchorPolicySnapshot {
                        family: *family,
                        actor: *actor,
                        policy: *policy,
                    },
                )
                .collect(),
            static_anchor_body_baselines: self
                .static_anchor_body_baselines
                .iter()
                .map(|((family, actor), body_type)| BodyBaselineSnapshot {
                    family: *family,
                    actor: *actor,
                    body_type: BodyTypeSnapshot::from_body_type(*body_type),
                })
                .collect(),
        })
    }

    fn from_snapshot(snapshot: FxRapierWorldSnapshot) -> Result<Self, FxRapierError> {
        validate_snapshot_metadata(&snapshot)?;
        let mut world = Self::new();
        world.snapshot_mode = snapshot.mode;
        world.tick = snapshot.tick;
        world.gravity = Vector::new(snapshot.gravity[0], snapshot.gravity[1]);
        apply_integration_snapshot(&mut world.integration_parameters, snapshot.integration);
        world.stress_solver = StressSolver2D::new(stress_from_snapshot(snapshot.stress));
        world.lod_settings = if snapshot.mode == SnapshotMode::Deterministic {
            ColliderLodSettings::disabled()
        } else {
            ColliderLodSettings::default()
        };
        let rapier = decode_rapier_owned_state(&snapshot.rapier_owned_state)?;
        world.bodies = rapier.bodies;
        world.colliders = rapier.colliders;
        world.impulse_joints = rapier.impulse_joints;
        world.multibody_joints = rapier.multibody_joints;
        world.islands = rapier.islands;
        world.broad_phase = rapier.broad_phase;
        world.narrow_phase = rapier.narrow_phase;
        world.ccd_solver = rapier.ccd_solver;
        for (material, properties) in snapshot.contact_materials {
            world.set_contact_material_properties(material, properties);
        }

        let mut families = snapshot.families;
        families.sort_by_key(|entry| entry.family);
        for family_snapshot in families {
            let asset = AuthoredVoxelAsset::from_snapshot_bytes(&family_snapshot.asset.bytes)
                .map_err(FxRapierSnapshotError::Voxel)?;
            let family = restore_family_snapshot(&family_snapshot.core_family)
                .map_err(FxRapierSnapshotError::Core)?;
            if family_snapshot.family != family.id {
                return Err(
                    FxRapierSnapshotError::StateMismatch("family snapshot id mismatch").into(),
                );
            }
            if family.asset() != asset.core() {
                return Err(
                    FxRapierSnapshotError::StateMismatch("family asset/core mismatch").into(),
                );
            }
            world.families.insert(
                family_snapshot.family,
                DestructibleFamily {
                    asset,
                    family,
                    physics: BTreeMap::new(),
                },
            );
        }

        let expected_actors = world
            .families
            .iter()
            .flat_map(|(family_id, entry)| {
                entry
                    .family
                    .actors()
                    .map(move |(actor_id, _)| (*family_id, *actor_id))
            })
            .collect::<BTreeSet<_>>();
        let mut actor_physics = snapshot.actor_physics;
        actor_physics.sort_by_key(|item| (item.family, item.actor));
        let actual_actors = actor_physics
            .iter()
            .map(|item| {
                let Some(entry) = world.families.get(&item.family) else {
                    return Err(FxRapierSnapshotError::StateMismatch(
                        "actor physics unknown family",
                    ));
                };
                if entry.family.actor(item.actor).is_none() {
                    return Err(FxRapierSnapshotError::StateMismatch(
                        "actor physics unknown actor",
                    ));
                }
                if world
                    .bodies
                    .get(RigidBodyHandle::from_raw_parts(
                        item.body_handle.0,
                        item.body_handle.1,
                    ))
                    .is_none()
                {
                    return Err(FxRapierSnapshotError::StateMismatch(
                        "actor references missing body",
                    ));
                }
                if world
                    .colliders
                    .get(ColliderHandle::from_raw_parts(
                        item.collider_handle.0,
                        item.collider_handle.1,
                    ))
                    .is_none()
                {
                    return Err(FxRapierSnapshotError::StateMismatch(
                        "actor references missing collider",
                    ));
                }
                Ok((item.family, item.actor))
            })
            .collect::<Result<BTreeSet<_>, _>>()?;
        if actual_actors != expected_actors {
            return Err(FxRapierSnapshotError::StateMismatch(
                "actor physics does not match restored family actors",
            )
            .into());
        }
        for item in actor_physics {
            world
                .families
                .get_mut(&item.family)
                .ok_or(FxRapierError::UnknownFamily(item.family))?
                .physics
                .insert(
                    item.actor,
                    ActorPhysicsState {
                        handles: ActorPhysicsHandles {
                            body: RigidBodyHandle::from_raw_parts(
                                item.body_handle.0,
                                item.body_handle.1,
                            ),
                            collider: ColliderHandle::from_raw_parts(
                                item.collider_handle.0,
                                item.collider_handle.1,
                            ),
                        },
                        body_local_origin_in_asset: Vec2::new(
                            item.body_local_origin_in_asset[0],
                            item.body_local_origin_in_asset[1],
                        ),
                    },
                );
        }
        world.body_actors = snapshot
            .body_actors
            .into_iter()
            .map(|entry| (entry.body_handle, entry.actor))
            .collect();
        {
            let registry = world.hooks.registry();
            let mut registry = registry
                .write()
                .expect("contact material registry poisoned");
            registry.collider_actors = snapshot
                .collider_actors
                .into_iter()
                .map(|entry| (entry.collider_handle, entry.actor))
                .collect();
            registry.collider_voxels = snapshot
                .collider_voxels
                .into_iter()
                .map(|entry| {
                    (
                        entry.collider_handle,
                        entry.voxels.into_iter().map(Into::into).collect(),
                    )
                })
                .collect();
        }

        world.static_anchor_policies = snapshot
            .static_anchor_policies
            .into_iter()
            .map(|entry| ((entry.family, entry.bond), entry.policy))
            .collect();
        world.applied_static_anchor_policies = snapshot
            .applied_static_anchor_policies
            .into_iter()
            .map(|entry| ((entry.family, entry.actor), entry.policy))
            .collect();
        world.static_anchor_body_baselines = snapshot
            .static_anchor_body_baselines
            .into_iter()
            .map(|entry| ((entry.family, entry.actor), entry.body_type.to_body_type()))
            .collect();

        {
            let registry = world.hooks.registry();
            let registry = registry.read().expect("contact material registry poisoned");
            world.validate_checkpoint_state(&registry, VoxelMetadataValidation::Captured)?;
        }
        Ok(world)
    }

    fn validate_checkpoint_state(
        &self,
        registry: &crate::hooks::ContactMaterialRegistry,
        voxel_metadata: VoxelMetadataValidation,
    ) -> Result<(), FxRapierError> {
        let mut managed_body_keys = BTreeSet::new();
        let mut managed_collider_keys = BTreeSet::new();

        for (family_id, entry) in &self.families {
            let actor_ids = entry
                .family
                .actors()
                .map(|(actor, _)| *actor)
                .collect::<BTreeSet<_>>();
            if entry.physics.keys().copied().collect::<BTreeSet<_>>() != actor_ids {
                return Err(FxRapierSnapshotError::StateMismatch(
                    "actor physics map does not match live family actors",
                )
                .into());
            }
            for (actor_id, state) in &entry.physics {
                if self.bodies.get(state.handles.body).is_none() {
                    return Err(FxRapierSnapshotError::StateMismatch(
                        "actor references missing body",
                    )
                    .into());
                }
                if self.colliders.get(state.handles.collider).is_none() {
                    return Err(FxRapierSnapshotError::StateMismatch(
                        "actor references missing collider",
                    )
                    .into());
                }
                let collider = self.colliders.get(state.handles.collider).ok_or(
                    FxRapierSnapshotError::StateMismatch("actor references missing collider"),
                )?;
                if collider.parent() != Some(state.handles.body) {
                    return Err(FxRapierSnapshotError::StateMismatch(
                        "actor collider parent mismatch",
                    )
                    .into());
                }
                if !managed_body_keys.insert(rigid_body_key(state.handles.body)) {
                    return Err(FxRapierSnapshotError::StateMismatch(
                        "duplicate managed body handle",
                    )
                    .into());
                }
                if !managed_collider_keys.insert(collider_key(state.handles.collider)) {
                    return Err(FxRapierSnapshotError::StateMismatch(
                        "duplicate managed collider handle",
                    )
                    .into());
                }
                let actor_ref = DestructibleActorRef {
                    family: *family_id,
                    actor: *actor_id,
                };
                if self.body_actors.get(&rigid_body_key(state.handles.body)) != Some(&actor_ref) {
                    return Err(FxRapierSnapshotError::StateMismatch(
                        "body actor registry mismatch",
                    )
                    .into());
                }
                if registry
                    .collider_actors
                    .get(&collider_key(state.handles.collider))
                    != Some(&actor_ref)
                {
                    return Err(FxRapierSnapshotError::StateMismatch(
                        "collider actor registry mismatch",
                    )
                    .into());
                }
                let actor =
                    entry
                        .family
                        .actor(*actor_id)
                        .ok_or(FxRapierSnapshotError::StateMismatch(
                            "actor physics unknown actor",
                        ))?;
                let material_id =
                    actor_default_contact_material(actor, &entry.asset).unwrap_or_default();
                let properties = registry
                    .material_properties
                    .get(&material_id)
                    .copied()
                    .unwrap_or_default();
                let expected_voxels = actor_collider_build(
                    actor,
                    &entry.asset,
                    state.body_local_origin_in_asset,
                    properties,
                    self.lod_settings,
                )
                .ok_or(FxRapierSnapshotError::StateMismatch(
                    "actor collider metadata missing",
                ))?
                .voxels;
                let Some(actual_voxels) = registry
                    .collider_voxels
                    .get(&collider_key(state.handles.collider))
                else {
                    return Err(FxRapierSnapshotError::StateMismatch(
                        "collider voxel metadata mismatch",
                    )
                    .into());
                };
                if !voxel_metadata.accepts(actual_voxels, &expected_voxels) {
                    return Err(FxRapierSnapshotError::StateMismatch(
                        "collider voxel metadata mismatch",
                    )
                    .into());
                }
            }
        }
        if self.body_actors.keys().copied().collect::<BTreeSet<_>>() != managed_body_keys {
            return Err(FxRapierSnapshotError::StateMismatch(
                "body actor registry is not bijective",
            )
            .into());
        }
        if registry
            .collider_actors
            .keys()
            .copied()
            .collect::<BTreeSet<_>>()
            != managed_collider_keys
        {
            return Err(FxRapierSnapshotError::StateMismatch(
                "collider actor registry is not bijective",
            )
            .into());
        }
        if registry
            .collider_voxels
            .keys()
            .copied()
            .collect::<BTreeSet<_>>()
            != managed_collider_keys
        {
            return Err(FxRapierSnapshotError::StateMismatch(
                "collider voxel registry is not bijective",
            )
            .into());
        }
        for (body, actor) in &self.body_actors {
            let handle = RigidBodyHandle::from_raw_parts(body.0, body.1);
            if self.bodies.get(handle).is_none() {
                return Err(FxRapierSnapshotError::StateMismatch("stale body actor handle").into());
            }
            self.require_actor_ref(*actor)?;
        }
        for (collider, actor) in &registry.collider_actors {
            let handle = ColliderHandle::from_raw_parts(collider.0, collider.1);
            if self.colliders.get(handle).is_none() {
                return Err(
                    FxRapierSnapshotError::StateMismatch("stale collider actor handle").into(),
                );
            }
            self.require_actor_ref(*actor)?;
        }
        for collider in registry.collider_voxels.keys() {
            let handle = ColliderHandle::from_raw_parts(collider.0, collider.1);
            if self.colliders.get(handle).is_none() {
                return Err(
                    FxRapierSnapshotError::StateMismatch("stale collider voxel handle").into(),
                );
            }
        }
        for ((family, bond), _) in &self.static_anchor_policies {
            let Some(entry) = self.families.get(family) else {
                return Err(
                    FxRapierSnapshotError::StateMismatch("static policy unknown family").into(),
                );
            };
            if entry.family.external_bond(*bond).is_none() {
                return Err(
                    FxRapierSnapshotError::StateMismatch("static policy unknown bond").into(),
                );
            }
        }
        for ((family, actor), _) in &self.applied_static_anchor_policies {
            self.require_actor_ref(DestructibleActorRef {
                family: *family,
                actor: *actor,
            })?;
        }
        for ((family, actor), _) in &self.static_anchor_body_baselines {
            self.require_actor_ref(DestructibleActorRef {
                family: *family,
                actor: *actor,
            })?;
        }
        let applied_policy_keys = self
            .applied_static_anchor_policies
            .keys()
            .copied()
            .collect::<BTreeSet<_>>();
        let baseline_keys = self
            .static_anchor_body_baselines
            .keys()
            .copied()
            .collect::<BTreeSet<_>>();
        if applied_policy_keys != baseline_keys {
            return Err(FxRapierSnapshotError::StateMismatch(
                "static anchor applied policy/baseline key mismatch",
            )
            .into());
        }
        for ((family, actor), policy) in &self.applied_static_anchor_policies {
            let handles = self.actor_handles_result(*family, *actor)?;
            let Some(body) = self.bodies.get(handles.body) else {
                return Err(FxRapierSnapshotError::StateMismatch(
                    "static anchor applied policy missing body",
                )
                .into());
            };
            if body.body_type() != rigid_body_type_for_anchor_policy(*policy) {
                return Err(FxRapierSnapshotError::StateMismatch(
                    "static anchor applied body type mismatch",
                )
                .into());
            }
        }
        Ok(())
    }

    fn require_actor_ref(&self, actor: DestructibleActorRef) -> Result<(), FxRapierError> {
        if self
            .families
            .get(&actor.family)
            .is_some_and(|entry| entry.family.actor(actor.actor).is_some())
        {
            Ok(())
        } else {
            Err(FxRapierSnapshotError::StateMismatch("actor reference is stale").into())
        }
    }

    fn sync_family_actors(&mut self, family_id: FxFamilyId) -> Result<(), FxRapierError> {
        let actor_ids = self
            .families
            .get(&family_id)
            .ok_or(FxRapierError::UnknownFamily(family_id))?
            .family
            .actors()
            .map(|(id, _)| *id)
            .collect::<Vec<_>>();
        for actor_id in actor_ids {
            if self.actor_handles(family_id, actor_id).is_none() {
                self.rebuild_actor_handles(family_id, actor_id, None)?;
            }
        }

        let live = self
            .families
            .get(&family_id)
            .ok_or(FxRapierError::UnknownFamily(family_id))?
            .family
            .actors()
            .map(|(id, _)| *id)
            .collect::<BTreeSet<_>>();
        let stale = self
            .families
            .get(&family_id)
            .ok_or(FxRapierError::UnknownFamily(family_id))?
            .physics
            .keys()
            .filter(|actor| !live.contains(actor))
            .copied()
            .collect::<Vec<_>>();
        for actor in stale {
            self.remove_actor_handles(family_id, actor);
        }
        self.reconcile_static_anchor_body_policies(family_id)?;
        Ok(())
    }

    fn snapshot_family_bodies(&self, family_id: FxFamilyId) -> BTreeMap<FxActorId, BodySnapshot> {
        let mut out = BTreeMap::new();
        let Some(entry) = self.families.get(&family_id) else {
            return out;
        };
        for (actor_id, state) in &entry.physics {
            let Some(body) = self.bodies.get(state.handles.body) else {
                continue;
            };
            let Some(_actor) = entry.family.actor(*actor_id) else {
                continue;
            };
            out.insert(
                *actor_id,
                BodySnapshot {
                    position: *body.position(),
                    world_center_of_mass: body.center_of_mass(),
                    linvel: body.linvel(),
                    angvel: body.angvel(),
                    ccd_enabled: body.is_ccd_enabled(),
                    soft_ccd_prediction: body.soft_ccd_prediction(),
                    was_sleeping: body.is_sleeping(),
                    body_local_origin_in_asset: state.body_local_origin_in_asset,
                },
            );
        }
        out
    }

    fn sync_split_family_actors(
        &mut self,
        family_id: FxFamilyId,
        parent_snapshots: &BTreeMap<FxActorId, BodySnapshot>,
        split_events: &[SplitEvent],
    ) -> Result<FxPhysicsSyncReport, FxRapierError> {
        // Rapier handle allocation order is not part of the deterministic contract; adapter
        // family/actor traversal and report append order are kept deterministic with BTree maps.
        let mut report = FxPhysicsSyncReport::default();
        let mut child_snapshots = BTreeMap::new();
        let mut touched_existing = BTreeSet::new();
        let mut touched_created = BTreeSet::new();
        let joint_remaps = self.collect_split_joint_endpoint_remaps(family_id, split_events);
        for event in split_events {
            let Some(parent_snapshot) = parent_snapshots.get(&event.parent_actor).copied() else {
                if let Some(child) = event.created_children.first().copied() {
                    return Err(FxRapierError::MissingSplitParentSnapshot {
                        family: family_id,
                        parent: event.parent_actor,
                        child,
                    });
                }
                continue;
            };
            touched_existing.insert(event.parent_actor);
            touched_existing.insert(event.kept_actor);
            for child in &event.created_children {
                child_snapshots.insert(*child, parent_snapshot);
                touched_created.insert(*child);
            }
        }
        let existing = self
            .families
            .get(&family_id)
            .ok_or(FxRapierError::UnknownFamily(family_id))?
            .physics
            .keys()
            .copied()
            .collect::<Vec<_>>();
        let live = self
            .families
            .get(&family_id)
            .ok_or(FxRapierError::UnknownFamily(family_id))?
            .family
            .actors()
            .map(|(id, _)| *id)
            .collect::<BTreeSet<_>>();
        for actor in existing {
            if !live.contains(&actor) {
                if self.remove_actor_handles(family_id, actor) {
                    report.removed_actor_bodies += 1;
                }
            } else if touched_existing.contains(&actor) {
                if let Some(kind) = self.rebuild_actor_collider_preserving_body(
                    family_id,
                    actor,
                    parent_snapshots.get(&actor).copied(),
                )? {
                    report.rebuilt_colliders += 1;
                    report.record_kind(kind);
                }
            }
        }
        report.untouched_actor_count = live
            .iter()
            .filter(|actor| !touched_existing.contains(actor) && !touched_created.contains(actor))
            .count();
        let actor_ids = live.into_iter().collect::<Vec<_>>();
        for actor_id in actor_ids {
            if self.actor_handles(family_id, actor_id).is_none() {
                let parent_snapshot = child_snapshots.get(&actor_id).copied();
                let inherited = child_snapshots
                    .contains_key(&actor_id)
                    .then_some(parent_snapshot)
                    .flatten();
                if let Some(kind) = self.rebuild_actor_handles(family_id, actor_id, inherited)? {
                    report.created_actor_bodies += 1;
                    report.record_kind(kind);
                }
            }
        }
        self.reconcile_static_anchor_body_policies(family_id)?;
        report.impulse_joint_handle_replacements =
            self.apply_split_joint_endpoint_remaps(family_id, &joint_remaps)?;
        Ok(report)
    }

    fn collect_split_joint_endpoint_remaps(
        &self,
        family_id: FxFamilyId,
        split_events: &[SplitEvent],
    ) -> Vec<SplitJointEndpointRemap> {
        let Some(entry) = self.families.get(&family_id) else {
            return Vec::new();
        };
        let mut out = Vec::new();
        for event in split_events {
            let Some(parent_state) = entry.physics.get(&event.parent_actor) else {
                continue;
            };
            let candidate_nodes = event
                .fragments
                .iter()
                .flat_map(|fragment| fragment.iter().copied())
                .collect::<Vec<_>>();
            let parent_body = parent_state.handles.body;
            for (_, _, handle, joint) in self.impulse_joints.attached_joints(parent_body) {
                let Some((endpoint, local_anchor)) = joint_endpoint_for_body(joint, parent_body)
                else {
                    continue;
                };
                let asset_anchor = parent_state.body_local_origin_in_asset
                    + Vec2::new(local_anchor.x, local_anchor.y);
                let Some(node) =
                    support_node_for_asset_point(&entry.asset, &candidate_nodes, asset_anchor)
                else {
                    continue;
                };
                let Some(new_actor) = entry.family.node_owner(node) else {
                    continue;
                };
                if new_actor == event.parent_actor {
                    continue;
                }
                out.push(SplitJointEndpointRemap {
                    joint: handle,
                    endpoint,
                    old_body: parent_body,
                    new_actor,
                    asset_anchor,
                });
            }
        }
        out
    }

    fn apply_split_joint_endpoint_remaps(
        &mut self,
        family_id: FxFamilyId,
        remaps: &[SplitJointEndpointRemap],
    ) -> Result<Vec<ImpulseJointHandleReplacement>, FxRapierError> {
        let mut replacements = Vec::new();
        for remap in remaps {
            if !self.impulse_joints.contains(remap.joint) {
                continue;
            }
            let Some(new_handles) = self.actor_handles(family_id, remap.new_actor) else {
                continue;
            };
            let Some(new_state) = self
                .families
                .get(&family_id)
                .and_then(|entry| entry.physics.get(&remap.new_actor))
                .copied()
            else {
                continue;
            };
            let Some(mut joint) = self.impulse_joints.remove(remap.joint, true) else {
                continue;
            };
            let new_local_anchor = Vector::new(
                remap.asset_anchor.x - new_state.body_local_origin_in_asset.x,
                remap.asset_anchor.y - new_state.body_local_origin_in_asset.y,
            );
            match remap.endpoint {
                JointEndpoint::Body1 => {
                    if joint.body1 != remap.old_body {
                        let new =
                            self.impulse_joints
                                .insert(joint.body1, joint.body2, joint.data, true);
                        replacements.push(ImpulseJointHandleReplacement {
                            old: remap.joint,
                            new,
                        });
                        continue;
                    }
                    joint.body1 = new_handles.body;
                    joint.data.set_local_anchor1(new_local_anchor);
                }
                JointEndpoint::Body2 => {
                    if joint.body2 != remap.old_body {
                        let new =
                            self.impulse_joints
                                .insert(joint.body1, joint.body2, joint.data, true);
                        replacements.push(ImpulseJointHandleReplacement {
                            old: remap.joint,
                            new,
                        });
                        continue;
                    }
                    joint.body2 = new_handles.body;
                    joint.data.set_local_anchor2(new_local_anchor);
                }
            }
            let new = self
                .impulse_joints
                .insert(joint.body1, joint.body2, joint.data, true);
            replacements.push(ImpulseJointHandleReplacement {
                old: remap.joint,
                new,
            });
        }
        Ok(replacements)
    }

    fn rebuild_actor_handles(
        &mut self,
        family_id: FxFamilyId,
        actor_id: FxActorId,
        inherited: Option<BodySnapshot>,
    ) -> Result<Option<ActorColliderBuildKind>, FxRapierError> {
        self.remove_actor_handles(family_id, actor_id);
        let entry = self
            .families
            .get(&family_id)
            .ok_or(FxRapierError::UnknownFamily(family_id))?;
        let actor = entry
            .family
            .actor(actor_id)
            .ok_or(FxRapierError::UnknownActor {
                family: family_id,
                actor: actor_id,
            })?;
        let material_id = actor_default_contact_material(actor, &entry.asset).unwrap_or_default();
        let properties = self
            .hooks
            .registry()
            .read()
            .expect("contact material registry poisoned")
            .material_properties
            .get(&material_id)
            .copied()
            .unwrap_or_default();
        let body_translation = inherited
            .map(|snapshot| child_world_translation(snapshot, actor.local_com))
            .unwrap_or_else(|| Vector::new(actor.local_com.x, actor.local_com.y));
        let local_origin = actor.local_com;
        let Some(collider_build) = actor_collider_build(
            actor,
            &entry.asset,
            local_origin,
            properties,
            self.lod_settings,
        ) else {
            return Ok(None);
        };
        let kind = collider_build.kind;
        let body = self
            .bodies
            .insert(actor_body_builder_at(actor, body_translation));
        if let Some(snapshot) = inherited {
            apply_body_dynamic_state(self.bodies.get_mut(body).expect("inserted body"), snapshot);
        }
        let collider = self.colliders.insert_with_parent(
            collider_build.builder.build(),
            body,
            &mut self.bodies,
        );
        let handles = ActorPhysicsHandles { body, collider };
        let actor_ref = DestructibleActorRef {
            family: family_id,
            actor: actor_id,
        };
        {
            let registry = self.hooks.registry();
            let mut registry = registry
                .write()
                .expect("contact material registry poisoned");
            registry
                .collider_actors
                .insert(collider_key(collider), actor_ref);
            registry
                .collider_voxels
                .insert(collider_key(collider), collider_build.voxels);
        }
        self.body_actors.insert(rigid_body_key(body), actor_ref);
        self.families
            .get_mut(&family_id)
            .ok_or(FxRapierError::UnknownFamily(family_id))?
            .physics
            .insert(
                actor_id,
                ActorPhysicsState {
                    handles,
                    body_local_origin_in_asset: local_origin,
                },
            );
        Ok(Some(kind))
    }

    fn rebuild_actor_collider_preserving_body(
        &mut self,
        family_id: FxFamilyId,
        actor_id: FxActorId,
        snapshot: Option<BodySnapshot>,
    ) -> Result<Option<ActorColliderBuildKind>, FxRapierError> {
        let old_state = self
            .families
            .get(&family_id)
            .and_then(|entry| entry.physics.get(&actor_id).copied())
            .ok_or(FxRapierError::UnknownActor {
                family: family_id,
                actor: actor_id,
            })?;
        let old_handles = old_state.handles;
        {
            let registry = self.hooks.registry();
            let mut registry = registry
                .write()
                .expect("contact material registry poisoned");
            registry
                .collider_actors
                .remove(&collider_key(old_handles.collider));
            registry
                .collider_voxels
                .remove(&collider_key(old_handles.collider));
        }
        self.colliders.remove(
            old_handles.collider,
            &mut self.islands,
            &mut self.bodies,
            true,
        );

        let entry = self
            .families
            .get(&family_id)
            .ok_or(FxRapierError::UnknownFamily(family_id))?;
        let actor = entry
            .family
            .actor(actor_id)
            .ok_or(FxRapierError::UnknownActor {
                family: family_id,
                actor: actor_id,
            })?;
        let material_id = actor_default_contact_material(actor, &entry.asset).unwrap_or_default();
        let properties = self
            .hooks
            .registry()
            .read()
            .expect("contact material registry poisoned")
            .material_properties
            .get(&material_id)
            .copied()
            .unwrap_or_default();
        let local_origin = snapshot
            .map(|snapshot| snapshot.body_local_origin_in_asset)
            .unwrap_or(old_state.body_local_origin_in_asset);
        let Some(collider_build) = actor_collider_build(
            actor,
            &entry.asset,
            local_origin,
            properties,
            self.lod_settings,
        ) else {
            return Ok(None);
        };
        let kind = collider_build.kind;
        if let Some(snapshot) = snapshot {
            if let Some(body) = self.bodies.get_mut(old_handles.body) {
                apply_body_snapshot(body, snapshot);
            }
        }
        let collider = self.colliders.insert_with_parent(
            collider_build.builder.build(),
            old_handles.body,
            &mut self.bodies,
        );
        let handles = ActorPhysicsHandles {
            body: old_handles.body,
            collider,
        };
        let actor_ref = DestructibleActorRef {
            family: family_id,
            actor: actor_id,
        };
        {
            let registry = self.hooks.registry();
            let mut registry = registry
                .write()
                .expect("contact material registry poisoned");
            registry
                .collider_actors
                .insert(collider_key(collider), actor_ref);
            registry
                .collider_voxels
                .insert(collider_key(collider), collider_build.voxels);
        }
        self.families
            .get_mut(&family_id)
            .ok_or(FxRapierError::UnknownFamily(family_id))?
            .physics
            .insert(
                actor_id,
                ActorPhysicsState {
                    handles,
                    body_local_origin_in_asset: local_origin,
                },
            );
        Ok(Some(kind))
    }

    fn remove_actor_handles(&mut self, family_id: FxFamilyId, actor_id: FxActorId) -> bool {
        let handles = self
            .families
            .get_mut(&family_id)
            .and_then(|entry| entry.physics.remove(&actor_id))
            .map(|state| state.handles);
        let Some(handles) = handles else {
            return false;
        };
        {
            let registry = self.hooks.registry();
            let mut registry = registry
                .write()
                .expect("contact material registry poisoned");
            registry
                .collider_actors
                .remove(&collider_key(handles.collider));
            registry
                .collider_voxels
                .remove(&collider_key(handles.collider));
        }
        self.body_actors.remove(&rigid_body_key(handles.body));
        self.bodies.remove(
            handles.body,
            &mut self.islands,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            true,
        );
        true
    }

    fn set_actor_total_mass(
        &mut self,
        family_id: FxFamilyId,
        actor_id: FxActorId,
        target_mass: f32,
    ) -> Result<(), FxRapierError> {
        let handles = self.actor_handles_result(family_id, actor_id)?;
        let Some(body) = self.bodies.get_mut(handles.body) else {
            return Err(FxRapierError::UnknownActor {
                family: family_id,
                actor: actor_id,
            });
        };
        body.set_additional_mass(0.0, true);
        body.recompute_mass_properties_from_colliders(&self.colliders);
        let collider_mass = body.mass();
        let additional_mass = (target_mass - collider_mass).max(0.0);
        body.set_additional_mass(additional_mass, true);
        body.recompute_mass_properties_from_colliders(&self.colliders);
        Ok(())
    }

    fn set_actor_world_center_of_mass(
        &mut self,
        family_id: FxFamilyId,
        actor_id: FxActorId,
        target_com: Vector,
    ) -> Result<(), FxRapierError> {
        let handles = self.actor_handles_result(family_id, actor_id)?;
        let Some(body) = self.bodies.get_mut(handles.body) else {
            return Err(FxRapierError::UnknownActor {
                family: family_id,
                actor: actor_id,
            });
        };
        let mut position = *body.position();
        position.translation += target_com - body.center_of_mass();
        body.set_position(position, true);
        Ok(())
    }

    fn reconcile_static_anchor_body_policies(
        &mut self,
        family_id: FxFamilyId,
    ) -> Result<(), FxRapierError> {
        let (desired, live_actors) = {
            let entry = self
                .families
                .get(&family_id)
                .ok_or(FxRapierError::UnknownFamily(family_id))?;
            let mut desired = BTreeMap::new();
            for ((policy_family, bond_id), policy) in &self.static_anchor_policies {
                if *policy_family != family_id || *policy == StaticAnchorBodyPolicy::Preserve {
                    continue;
                }
                let Some(bond) = entry.family.external_bond(*bond_id) else {
                    continue;
                };
                if bond.runtime.is_broken() {
                    continue;
                }
                let Some(actor) = entry.family.node_owner(bond.node) else {
                    continue;
                };
                desired
                    .entry(actor)
                    .and_modify(|current| {
                        *current = combine_static_anchor_body_policy(*current, *policy)
                    })
                    .or_insert(*policy);
            }
            let live_actors = entry
                .family
                .actors()
                .map(|(actor, _)| *actor)
                .collect::<BTreeSet<_>>();
            (desired, live_actors)
        };

        let mut touched = desired.keys().copied().collect::<BTreeSet<_>>();
        touched.extend(
            self.applied_static_anchor_policies
                .keys()
                .filter_map(|(family, actor)| (*family == family_id).then_some(*actor)),
        );

        for actor in touched {
            let key = (family_id, actor);
            if !live_actors.contains(&actor) {
                self.applied_static_anchor_policies.remove(&key);
                self.static_anchor_body_baselines.remove(&key);
                continue;
            }
            match desired.get(&actor).copied() {
                Some(policy) => {
                    let handles = self.actor_handles_result(family_id, actor)?;
                    let Some(body) = self.bodies.get_mut(handles.body) else {
                        return Err(FxRapierError::UnknownActor {
                            family: family_id,
                            actor,
                        });
                    };
                    self.static_anchor_body_baselines
                        .entry(key)
                        .or_insert_with(|| body.body_type());
                    body.set_body_type(rigid_body_type_for_anchor_policy(policy), true);
                    self.applied_static_anchor_policies.insert(key, policy);
                }
                None => {
                    if self.applied_static_anchor_policies.remove(&key).is_some() {
                        let baseline = self
                            .static_anchor_body_baselines
                            .remove(&key)
                            .unwrap_or(RigidBodyType::Dynamic);
                        let handles = self.actor_handles_result(family_id, actor)?;
                        let Some(body) = self.bodies.get_mut(handles.body) else {
                            return Err(FxRapierError::UnknownActor {
                                family: family_id,
                                actor,
                            });
                        };
                        body.set_body_type(baseline, true);
                    }
                }
            }
        }
        Ok(())
    }
}

fn apply_body_snapshot(body: &mut RigidBody, snapshot: BodySnapshot) {
    body.set_position(snapshot.position, true);
    apply_body_dynamic_state(body, snapshot);
}

fn apply_body_dynamic_state(body: &mut RigidBody, snapshot: BodySnapshot) {
    body.set_linvel(snapshot.linvel, true);
    body.set_angvel(snapshot.angvel, true);
    body.enable_ccd(snapshot.ccd_enabled);
    body.set_soft_ccd_prediction(snapshot.soft_ccd_prediction);
    if snapshot.was_sleeping {
        body.sleep();
    }
}

fn integration_to_snapshot(params: &IntegrationParameters) -> crate::snapshot::IntegrationSnapshot {
    crate::snapshot::IntegrationSnapshot {
        dt: params.dt,
        min_ccd_dt: params.min_ccd_dt,
        contact_softness_natural_frequency: params.contact_softness.natural_frequency,
        contact_softness_damping_ratio: params.contact_softness.damping_ratio,
        warmstart_coefficient: params.warmstart_coefficient,
        length_unit: params.length_unit,
        normalized_allowed_linear_error: params.normalized_allowed_linear_error,
        normalized_max_corrective_velocity: params.normalized_max_corrective_velocity,
        normalized_prediction_distance: params.normalized_prediction_distance,
        num_solver_iterations: params.num_solver_iterations,
        num_internal_pgs_iterations: params.num_internal_pgs_iterations,
        num_internal_stabilization_iterations: params.num_internal_stabilization_iterations,
        min_island_size: params.min_island_size,
        max_ccd_substeps: params.max_ccd_substeps,
    }
}

fn apply_integration_snapshot(
    params: &mut IntegrationParameters,
    snapshot: crate::snapshot::IntegrationSnapshot,
) {
    params.dt = snapshot.dt;
    params.min_ccd_dt = snapshot.min_ccd_dt;
    params.contact_softness.natural_frequency = snapshot.contact_softness_natural_frequency;
    params.contact_softness.damping_ratio = snapshot.contact_softness_damping_ratio;
    params.warmstart_coefficient = snapshot.warmstart_coefficient;
    params.length_unit = snapshot.length_unit;
    params.normalized_allowed_linear_error = snapshot.normalized_allowed_linear_error;
    params.normalized_max_corrective_velocity = snapshot.normalized_max_corrective_velocity;
    params.normalized_prediction_distance = snapshot.normalized_prediction_distance;
    params.num_solver_iterations = snapshot.num_solver_iterations;
    params.num_internal_pgs_iterations = snapshot.num_internal_pgs_iterations;
    params.num_internal_stabilization_iterations = snapshot.num_internal_stabilization_iterations;
    params.min_island_size = snapshot.min_island_size;
    params.max_ccd_substeps = snapshot.max_ccd_substeps;
}

fn stress_to_snapshot(settings: StressSettings) -> StressSettingsSnapshot {
    settings.into()
}

fn stress_from_snapshot(snapshot: StressSettingsSnapshot) -> StressSettings {
    snapshot.into()
}

pub(crate) fn ensure_snapshot_mode_available(mode: SnapshotMode) -> Result<(), FxRapierError> {
    if mode == SnapshotMode::Deterministic && !cfg!(feature = "deterministic-replay") {
        return Err(FxRapierSnapshotError::DeterministicReplayFeatureRequired.into());
    }
    Ok(())
}

fn ensure_deterministic_lod_disabled(
    mode: SnapshotMode,
    lod_settings: ColliderLodSettings,
) -> Result<(), FxRapierError> {
    if mode == SnapshotMode::Deterministic && lod_settings.enabled {
        return Err(FxRapierSnapshotError::InvalidValue("deterministic lod").into());
    }
    Ok(())
}

fn validate_snapshot_metadata(snapshot: &FxRapierWorldSnapshot) -> Result<(), FxRapierError> {
    if !snapshot.gravity[0].is_finite() || !snapshot.gravity[1].is_finite() {
        return Err(FxRapierSnapshotError::InvalidValue("gravity").into());
    }
    validate_integration_snapshot(snapshot.integration)?;
    validate_stress_snapshot(snapshot.stress)?;

    let mut families = BTreeSet::new();
    for family in &snapshot.families {
        if !families.insert(family.family) {
            return Err(FxRapierSnapshotError::StateMismatch("duplicate family").into());
        }
    }

    let mut actor_physics = BTreeSet::new();
    let mut actor_bodies = BTreeSet::new();
    let mut actor_colliders = BTreeSet::new();
    for item in &snapshot.actor_physics {
        if !actor_physics.insert((item.family, item.actor)) {
            return Err(FxRapierSnapshotError::StateMismatch("duplicate actor physics").into());
        }
        if !actor_bodies.insert(item.body_handle) {
            return Err(FxRapierSnapshotError::StateMismatch("duplicate actor body handle").into());
        }
        if !actor_colliders.insert(item.collider_handle) {
            return Err(
                FxRapierSnapshotError::StateMismatch("duplicate actor collider handle").into(),
            );
        }
        if !item.body_local_origin_in_asset[0].is_finite()
            || !item.body_local_origin_in_asset[1].is_finite()
        {
            return Err(FxRapierSnapshotError::InvalidValue("actor local origin").into());
        }
    }
    let mut body_actors = BTreeSet::new();
    for item in &snapshot.body_actors {
        if !body_actors.insert(item.body_handle) {
            return Err(FxRapierSnapshotError::StateMismatch("duplicate body actor").into());
        }
    }
    let mut collider_actors = BTreeSet::new();
    for item in &snapshot.collider_actors {
        if !collider_actors.insert(item.collider_handle) {
            return Err(FxRapierSnapshotError::StateMismatch("duplicate collider actor").into());
        }
    }
    let mut collider_voxels = BTreeSet::new();
    for item in &snapshot.collider_voxels {
        if !collider_voxels.insert(item.collider_handle) {
            return Err(FxRapierSnapshotError::StateMismatch("duplicate collider voxels").into());
        }
    }

    let mut static_policies = BTreeSet::new();
    for item in &snapshot.static_anchor_policies {
        if !static_policies.insert((item.family, item.bond)) {
            return Err(FxRapierSnapshotError::StateMismatch("duplicate static policy").into());
        }
    }
    let mut applied_policies = BTreeSet::new();
    for item in &snapshot.applied_static_anchor_policies {
        if !applied_policies.insert((item.family, item.actor)) {
            return Err(FxRapierSnapshotError::StateMismatch("duplicate applied policy").into());
        }
    }
    let mut baselines = BTreeSet::new();
    for item in &snapshot.static_anchor_body_baselines {
        if !baselines.insert((item.family, item.actor)) {
            return Err(FxRapierSnapshotError::StateMismatch("duplicate body baseline").into());
        }
    }
    let mut materials = BTreeSet::new();
    for (material, properties) in &snapshot.contact_materials {
        if !materials.insert(*material) {
            return Err(FxRapierSnapshotError::StateMismatch("duplicate contact material").into());
        }
        if !properties.friction.is_finite() || !properties.restitution.is_finite() {
            return Err(FxRapierSnapshotError::InvalidValue("contact material").into());
        }
    }
    Ok(())
}

fn validate_integration_snapshot(
    snapshot: crate::snapshot::IntegrationSnapshot,
) -> Result<(), FxRapierError> {
    let scalars = [
        snapshot.dt,
        snapshot.min_ccd_dt,
        snapshot.contact_softness_natural_frequency,
        snapshot.contact_softness_damping_ratio,
        snapshot.warmstart_coefficient,
        snapshot.length_unit,
        snapshot.normalized_allowed_linear_error,
        snapshot.normalized_max_corrective_velocity,
        snapshot.normalized_prediction_distance,
    ];
    if scalars.into_iter().any(|value| !value.is_finite()) {
        return Err(FxRapierSnapshotError::InvalidValue("integration").into());
    }
    Ok(())
}

fn validate_stress_snapshot(snapshot: StressSettingsSnapshot) -> Result<(), FxRapierError> {
    if !snapshot.tension_limit_scale.is_finite()
        || !snapshot.shear_limit_scale.is_finite()
        || !snapshot.damage_per_overload.is_finite()
    {
        return Err(FxRapierSnapshotError::InvalidValue("stress").into());
    }
    Ok(())
}

fn child_world_translation(snapshot: BodySnapshot, child_local_com: Vec2) -> Vector {
    let delta = Vector::new(
        child_local_com.x - snapshot.body_local_origin_in_asset.x,
        child_local_com.y - snapshot.body_local_origin_in_asset.y,
    );
    snapshot.position.translation + snapshot.position.rotation * delta
}

fn joint_endpoint_for_body(
    joint: &ImpulseJoint,
    body: RigidBodyHandle,
) -> Option<(JointEndpoint, Vector)> {
    if joint.body1 == body {
        Some((JointEndpoint::Body1, joint.data.local_anchor1()))
    } else if joint.body2 == body {
        Some((JointEndpoint::Body2, joint.data.local_anchor2()))
    } else {
        None
    }
}

fn support_node_for_asset_point(
    asset: &AuthoredVoxelAsset,
    candidates: &[SupportNodeId],
    point: Vec2,
) -> Option<SupportNodeId> {
    let candidate_set = candidates.iter().copied().collect::<BTreeSet<_>>();
    let voxel_size = asset.core().voxel_size();
    if point.x >= 0.0 && point.y >= 0.0 {
        let coord = GridCoord::new(
            (point.x / voxel_size).floor() as u32,
            (point.y / voxel_size).floor() as u32,
        );
        if let Some(node) = asset.core().node_at(coord)
            && candidate_set.contains(&node)
        {
            return Some(node);
        }
    }

    let mut best = None;
    for node_id in candidates {
        let Some(node) = asset.core().node(*node_id) else {
            continue;
        };
        for voxel in &node.voxels {
            let center = voxel.center(voxel_size);
            let delta = center - point;
            let distance2 = delta.dot(delta);
            match best {
                Some((_, old_distance2)) if old_distance2 <= distance2 => {}
                _ => best = Some((*node_id, distance2)),
            }
        }
    }
    best.map(|(node, _)| node)
}

fn merged_body_snapshot(
    kept_snapshot: BodySnapshot,
    snapshot_a: BodySnapshot,
    mass_a: f32,
    snapshot_b: BodySnapshot,
    mass_b: f32,
    body_local_origin_in_asset: Vec2,
) -> BodySnapshot {
    let total_mass = mass_a + mass_b;
    let (weight_a, weight_b) = if total_mass > 0.0 && total_mass.is_finite() {
        (mass_a / total_mass, mass_b / total_mass)
    } else {
        (0.5, 0.5)
    };
    // Phase 4 preserves linear momentum exactly for the merged body. Angular
    // momentum is approximated deterministically with a mass-weighted angular
    // velocity because full inertia transfer across two arbitrary body poses is
    // outside the MVP connection contract.
    BodySnapshot {
        position: Pose::from_parts(
            snapshot_a.world_center_of_mass * weight_a + snapshot_b.world_center_of_mass * weight_b,
            kept_snapshot.position.rotation,
        ),
        world_center_of_mass: snapshot_a.world_center_of_mass * weight_a
            + snapshot_b.world_center_of_mass * weight_b,
        linvel: snapshot_a.linvel * weight_a + snapshot_b.linvel * weight_b,
        angvel: snapshot_a.angvel * weight_a + snapshot_b.angvel * weight_b,
        ccd_enabled: snapshot_a.ccd_enabled || snapshot_b.ccd_enabled,
        soft_ccd_prediction: snapshot_a
            .soft_ccd_prediction
            .max(snapshot_b.soft_ccd_prediction),
        was_sleeping: snapshot_a.was_sleeping && snapshot_b.was_sleeping,
        body_local_origin_in_asset,
    }
}

fn rigid_body_type_for_anchor_policy(policy: StaticAnchorBodyPolicy) -> RigidBodyType {
    match policy {
        StaticAnchorBodyPolicy::Preserve => RigidBodyType::Dynamic,
        StaticAnchorBodyPolicy::Fixed => RigidBodyType::Fixed,
        StaticAnchorBodyPolicy::KinematicVelocityBased => RigidBodyType::KinematicVelocityBased,
    }
}

fn combine_static_anchor_body_policy(
    current: StaticAnchorBodyPolicy,
    next: StaticAnchorBodyPolicy,
) -> StaticAnchorBodyPolicy {
    match (current, next) {
        (StaticAnchorBodyPolicy::Fixed, _) | (_, StaticAnchorBodyPolicy::Fixed) => {
            StaticAnchorBodyPolicy::Fixed
        }
        (StaticAnchorBodyPolicy::KinematicVelocityBased, _)
        | (_, StaticAnchorBodyPolicy::KinematicVelocityBased) => {
            StaticAnchorBodyPolicy::KinematicVelocityBased
        }
        _ => StaticAnchorBodyPolicy::Preserve,
    }
}
