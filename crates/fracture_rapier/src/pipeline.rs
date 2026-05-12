use fracture_core::{FractureEvent, SplitEvent, StressInput, StressProfile};

use crate::{ContactImpulseInput, FxPhysicsSyncReport, JointFeedbackStress};

pub const OCCUPIED_VOXEL_BUDGET: usize = 10_000;
pub const SUPPORT_NODE_BUDGET: usize = 200;
pub const ACTIVE_BODY_BUDGET: usize = 100;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FxPerformanceBudgetReport {
    pub occupied_voxels: usize,
    pub occupied_voxel_budget: usize,
    pub support_nodes: usize,
    pub support_node_budget: usize,
    pub active_bodies: usize,
    pub active_body_budget: usize,
}

impl FxPerformanceBudgetReport {
    pub fn within_budget(self) -> bool {
        self.occupied_voxels <= self.occupied_voxel_budget
            && self.support_nodes <= self.support_node_budget
            && self.active_bodies <= self.active_body_budget
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct FxStepReport {
    pub contact_impulses: Vec<ContactImpulseInput>,
    pub joint_feedback: Vec<JointFeedbackStress>,
    pub stress_inputs: Vec<StressInput>,
    pub fracture_events: Vec<FractureEvent>,
    pub split_events: Vec<SplitEvent>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct FxGlobalStressCapReport {
    pub input_count: usize,
    pub family_count: usize,
    pub generated_commands_before_cap: usize,
    pub generated_commands_after_cap: usize,
    pub frame_cap: u16,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct FxStepDiagnostics {
    pub stress_profiles: Vec<StressProfile>,
    pub global_stress_cap: FxGlobalStressCapReport,
    pub physics_sync: FxPhysicsSyncReport,
    pub budget: Option<FxPerformanceBudgetReport>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct FxStepWithDiagnostics {
    pub report: FxStepReport,
    pub diagnostics: FxStepDiagnostics,
}

impl FxStepWithDiagnostics {
    pub fn into_report(self) -> FxStepReport {
        self.report
    }
}
