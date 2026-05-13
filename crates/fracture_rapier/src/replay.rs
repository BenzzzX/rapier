use std::collections::BTreeSet;

use fracture_core::{FractureEvent, FxFamilyId, SplitEvent};
use rapier2d::prelude::*;

use crate::{FxRapierError, FxRapierWorld2D, ImpulseJointHandleReplacement};

#[derive(Clone, Debug, PartialEq)]
pub struct FxRapierReplayCommand {
    pub tick: u64,
    pub stable_order: u64,
    pub family: FxFamilyId,
    pub command: fracture_core::FractureCommand,
}

impl FxRapierReplayCommand {
    pub fn new(
        tick: u64,
        stable_order: u64,
        family: FxFamilyId,
        command: fracture_core::FractureCommand,
    ) -> Self {
        Self {
            tick,
            stable_order,
            family,
            command,
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct FxRapierReplayTickReport {
    pub fracture_events: Vec<FractureEvent>,
    pub split_events: Vec<SplitEvent>,
    pub impulse_joint_handle_replacements: Vec<ImpulseJointHandleReplacement>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ReplayTraceActorBody {
    pub family: FxFamilyId,
    pub actor: fracture_core::FxActorId,
    pub translation: [u32; 2],
    pub rotation_angle: u32,
    pub linvel: [u32; 2],
    pub angvel: u32,
    pub body_type: u8,
    pub sleeping: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ReplayTraceTick {
    pub tick: u64,
    pub family_digests: Vec<(FxFamilyId, u64)>,
    pub fracture_events: Vec<FractureEvent>,
    pub split_events: Vec<SplitEvent>,
    pub actor_bodies: Vec<ReplayTraceActorBody>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct ReplayTrace {
    pub ticks: Vec<ReplayTraceTick>,
}

pub fn sort_replay_commands(commands: &mut [FxRapierReplayCommand]) {
    commands.sort_by(|a, b| {
        a.tick
            .cmp(&b.tick)
            .then_with(|| a.stable_order.cmp(&b.stable_order))
            .then_with(|| a.family.cmp(&b.family))
            .then_with(|| a.command.order_key.cmp(&b.command.order_key))
            .then_with(|| a.command.target.cmp(&b.command.target))
            .then_with(|| a.command.actor.cmp(&b.command.actor))
            .then_with(|| {
                a.command
                    .health_loss
                    .to_bits()
                    .cmp(&b.command.health_loss.to_bits())
            })
            .then_with(|| {
                a.command
                    .effective_length_loss
                    .to_bits()
                    .cmp(&b.command.effective_length_loss.to_bits())
            })
    });
}

pub fn validate_replay_commands(commands: &[FxRapierReplayCommand]) -> Result<(), FxRapierError> {
    let mut seen = BTreeSet::new();
    for command in commands {
        let key = (
            command.tick,
            command.stable_order,
            command.family,
            command.command.order_key,
            command.command.target,
        );
        if !seen.insert(key) {
            return Err(FxRapierError::DuplicateReplayKey {
                tick: command.tick,
                stable_order: command.stable_order,
            });
        }
    }
    Ok(())
}

impl FxRapierWorld2D {
    pub fn run_replay_trace(
        &mut self,
        end_tick: u64,
        commands: &[FxRapierReplayCommand],
    ) -> Result<ReplayTrace, FxRapierError> {
        if self.snapshot_mode() != fracture_core::snapshot::SnapshotMode::Deterministic {
            return Err(FxRapierError::ReplayRequiresDeterministicMode);
        }
        crate::world::ensure_snapshot_mode_available(self.snapshot_mode())?;
        validate_replay_commands(commands)?;
        let mut trace = ReplayTrace::default();
        while self.tick() < end_tick {
            let tick = self.tick();
            let manual = self.apply_replay_tick(tick, commands)?;
            let step = self.step()?;
            let mut fracture_events = manual.fracture_events;
            fracture_events.extend(step.fracture_events);
            let mut split_events = manual.split_events;
            split_events.extend(step.split_events);
            trace.ticks.push(ReplayTraceTick {
                tick,
                family_digests: self.family_digests(),
                fracture_events,
                split_events,
                actor_bodies: self.sorted_actor_body_trace(),
            });
        }
        Ok(trace)
    }

    pub fn family_digests(&self) -> Vec<(FxFamilyId, u64)> {
        self.families
            .iter()
            .map(|(id, entry)| (*id, entry.family.deterministic_state_digest()))
            .collect()
    }

    pub fn sorted_actor_body_trace(&self) -> Vec<ReplayTraceActorBody> {
        let mut out = Vec::new();
        for (family_id, entry) in &self.families {
            for (actor_id, state) in &entry.physics {
                let Some(body) = self.bodies.get(state.handles.body) else {
                    continue;
                };
                out.push(ReplayTraceActorBody {
                    family: *family_id,
                    actor: *actor_id,
                    translation: [
                        body.translation().x.to_bits(),
                        body.translation().y.to_bits(),
                    ],
                    rotation_angle: body.rotation().angle().to_bits(),
                    linvel: [body.linvel().x.to_bits(), body.linvel().y.to_bits()],
                    angvel: body.angvel().to_bits(),
                    body_type: match body.body_type() {
                        RigidBodyType::Dynamic => 0,
                        RigidBodyType::Fixed => 1,
                        RigidBodyType::KinematicPositionBased => 2,
                        RigidBodyType::KinematicVelocityBased => 3,
                    },
                    sleeping: body.is_sleeping(),
                });
            }
        }
        out.sort_by_key(|entry| (entry.family, entry.actor));
        out
    }
}
