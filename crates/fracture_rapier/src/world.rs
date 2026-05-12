use std::collections::{BTreeMap, BTreeSet};

use fracture_core::{
    FxActorId, FxFamily, FxFamilyId, SplitEvent, StressSettings, StressSolver2D, Vec2,
    apply_fracture_commands, split_dirty_actors,
};
use fracture_voxel::AuthoredVoxelAsset;
use rapier2d::prelude::*;
use thiserror::Error;

use crate::collider_sync::{
    ActorPhysicsHandles, DestructibleActorRef, actor_body_builder_at, actor_collider_build,
    actor_default_contact_material,
};
use crate::contact_map::{collider_key, rigid_body_key};
use crate::hooks::{ContactMaterialProperties, FxContactHooks, HookObservation};
use crate::impulse_readback::collect_contact_impulse_inputs;
use crate::joint_feedback::collect_joint_feedback_stress;
use crate::pipeline::FxStepReport;

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
}

#[derive(Clone, Debug)]
struct DestructibleFamily {
    asset: AuthoredVoxelAsset,
    family: FxFamily,
    physics: BTreeMap<FxActorId, ActorPhysicsState>,
}

#[derive(Clone, Copy, Debug)]
struct ActorPhysicsState {
    handles: ActorPhysicsHandles,
    body_local_origin_in_asset: Vec2,
}

pub struct FxRapierWorld2D {
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
    hooks: FxContactHooks,
    stress_solver: StressSolver2D,
    families: BTreeMap<FxFamilyId, DestructibleFamily>,
    body_actors: BTreeMap<(u32, u32), DestructibleActorRef>,
    tick: u64,
}

#[derive(Clone, Copy, Debug)]
struct BodySnapshot {
    position: Pose,
    linvel: Vector,
    angvel: f32,
    ccd_enabled: bool,
    soft_ccd_prediction: f32,
    was_sleeping: bool,
    body_local_origin_in_asset: Vec2,
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
            families: BTreeMap::new(),
            body_actors: BTreeMap::new(),
            tick: 0,
        }
    }

    pub fn set_stress_settings(&mut self, settings: StressSettings) {
        self.stress_solver = StressSolver2D::new(settings);
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

    pub fn step(&mut self) -> Result<FxStepReport, FxRapierError> {
        let family_ids = self.families.keys().copied().collect::<Vec<_>>();
        for family_id in family_ids {
            self.sync_family_actors(family_id)?;
        }

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
        let contact_impulses = collect_contact_impulse_inputs(
            self.tick,
            self.integration_parameters.dt,
            &self.narrow_phase,
            &families,
            &registry,
        );
        let joint_feedback = collect_joint_feedback_stress(
            self.tick,
            self.integration_parameters.dt,
            &self.impulse_joints,
            &self.body_actors,
            &families,
        );

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

        let family_ids = self.families.keys().copied().collect::<Vec<_>>();
        for family_id in family_ids {
            let stress_inputs = report
                .stress_inputs
                .iter()
                .filter(|input| input.order_key.family_id == family_id)
                .cloned()
                .collect::<Vec<_>>();
            let parent_snapshots = self.snapshot_family_bodies(family_id);
            let Some(entry) = self.families.get_mut(&family_id) else {
                return Err(FxRapierError::UnknownFamily(family_id));
            };
            let commands = self.stress_solver.generate(&entry.family, &stress_inputs);
            report
                .fracture_events
                .extend(apply_fracture_commands(&mut entry.family, &commands));
            let split_events = split_dirty_actors(&mut entry.family);
            let split_happened = !split_events.is_empty();
            if split_happened {
                self.sync_split_family_actors(family_id, &parent_snapshots, &split_events)?;
            }
            report.split_events.extend(split_events);
        }

        self.tick += 1;
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
            self.sync_split_family_actors(family_id, &parent_snapshots, &split_events)?;
        }
        Ok(split_events)
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
            self.sync_split_family_actors(family_id, &parent_snapshots, &split_events)?;
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
                self.sync_split_family_actors(family_id, &parent_snapshots, &split_events)?;
            }
            out.extend(split_events);
        }
        Ok(out)
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
    ) -> Result<(), FxRapierError> {
        // Rapier handle allocation order is not part of the deterministic contract; adapter
        // family/actor traversal and report append order are kept deterministic with BTree maps.
        let mut child_snapshots = BTreeMap::new();
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
            for child in &event.created_children {
                child_snapshots.insert(*child, parent_snapshot);
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
            if live.contains(&actor) {
                self.rebuild_actor_collider_preserving_body(
                    family_id,
                    actor,
                    parent_snapshots.get(&actor).copied(),
                )?;
            } else {
                self.remove_actor_handles(family_id, actor);
            }
        }
        let actor_ids = live.into_iter().collect::<Vec<_>>();
        for actor_id in actor_ids {
            if self.actor_handles(family_id, actor_id).is_none() {
                let parent_snapshot = child_snapshots.get(&actor_id).copied();
                if child_snapshots.contains_key(&actor_id) {
                    self.rebuild_actor_handles(family_id, actor_id, parent_snapshot)?;
                } else {
                    self.rebuild_actor_handles(family_id, actor_id, None)?;
                }
            }
        }
        Ok(())
    }

    fn rebuild_actor_handles(
        &mut self,
        family_id: FxFamilyId,
        actor_id: FxActorId,
        inherited: Option<BodySnapshot>,
    ) -> Result<(), FxRapierError> {
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
        let Some(collider_build) =
            actor_collider_build(actor, &entry.asset, local_origin, properties)
        else {
            return Ok(());
        };
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
        Ok(())
    }

    fn rebuild_actor_collider_preserving_body(
        &mut self,
        family_id: FxFamilyId,
        actor_id: FxActorId,
        snapshot: Option<BodySnapshot>,
    ) -> Result<(), FxRapierError> {
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
        let Some(collider_build) =
            actor_collider_build(actor, &entry.asset, local_origin, properties)
        else {
            return Ok(());
        };
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
        Ok(())
    }

    fn remove_actor_handles(&mut self, family_id: FxFamilyId, actor_id: FxActorId) {
        let handles = self
            .families
            .get_mut(&family_id)
            .and_then(|entry| entry.physics.remove(&actor_id))
            .map(|state| state.handles);
        let Some(handles) = handles else {
            return;
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

fn child_world_translation(snapshot: BodySnapshot, child_local_com: Vec2) -> Vector {
    let delta = Vector::new(
        child_local_com.x - snapshot.body_local_origin_in_asset.x,
        child_local_com.y - snapshot.body_local_origin_in_asset.y,
    );
    snapshot.position.translation + snapshot.position.rotation * delta
}
