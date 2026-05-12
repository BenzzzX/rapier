use fracture_core::{FractureEvent, SplitEvent, StressInput};

use crate::{ContactImpulseInput, JointFeedbackStress};

#[derive(Clone, Debug, Default, PartialEq)]
pub struct FxStepReport {
    pub contact_impulses: Vec<ContactImpulseInput>,
    pub joint_feedback: Vec<JointFeedbackStress>,
    pub stress_inputs: Vec<StressInput>,
    pub fracture_events: Vec<FractureEvent>,
    pub split_events: Vec<SplitEvent>,
}
