use std::collections::BTreeMap;

use fracture_core::{
    CommandId, DamageSource, DeterministicOrderKey, FxFamily, FxFamilyId, StressInput, Vec2,
};
use rapier2d::prelude::*;

use crate::collider_sync::DestructibleActorRef;
use crate::contact_map::rigid_body_key;

#[derive(Clone, Debug, PartialEq)]
pub struct JointFeedbackStress {
    pub joint: ImpulseJointHandle,
    pub destructible: DestructibleActorRef,
    pub impulse_magnitude: f32,
    pub stress: StressInput,
}

pub(crate) fn collect_joint_feedback_stress(
    tick: u64,
    dt: f32,
    impulse_joints: &ImpulseJointSet,
    body_actors: &BTreeMap<(u32, u32), DestructibleActorRef>,
    families: &[(FxFamilyId, &FxFamily)],
) -> Vec<JointFeedbackStress> {
    let mut joints = impulse_joints.iter().collect::<Vec<_>>();
    joints.sort_by_key(|(handle, joint)| {
        (
            handle.into_raw_parts(),
            rigid_body_key(joint.body1),
            rigid_body_key(joint.body2),
        )
    });

    let mut out = Vec::new();
    let mut command_id = 0u32;
    for (handle, joint) in joints {
        let destructible = body_actors
            .get(&rigid_body_key(joint.body1))
            .or_else(|| body_actors.get(&rigid_body_key(joint.body2)))
            .copied();
        let Some(destructible) = destructible else {
            continue;
        };
        let impulse_magnitude = joint.impulses.length();
        if impulse_magnitude <= 0.0 {
            continue;
        }
        let Some((_, family)) = families.iter().find(|(id, _)| *id == destructible.family) else {
            continue;
        };
        let Some(node) = family
            .actor(destructible.actor)
            .and_then(|actor| actor.owned_nodes.first().copied())
        else {
            continue;
        };
        let stress = StressInput {
            order_key: DeterministicOrderKey::new(
                tick,
                20,
                destructible.family,
                destructible.actor,
                CommandId(command_id),
            ),
            actor: destructible.actor,
            node,
            force: Vec2::new(impulse_magnitude / dt.max(f32::EPSILON), 0.0),
            source: DamageSource::JointFeedback,
        };
        command_id += 1;
        out.push(JointFeedbackStress {
            joint: handle,
            destructible,
            impulse_magnitude,
            stress,
        });
    }
    out.sort_by_key(|feedback| feedback.stress.order_key);
    out
}
