use fracture_core::{DynamicConnectionPolicy, DynamicStructuralBondDesc, StaticAnchorDesc};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum StaticAnchorBodyPolicy {
    #[default]
    Preserve,
    Fixed,
    KinematicVelocityBased,
}

#[derive(Clone, Debug, PartialEq)]
pub struct StaticAnchorConnectionDesc {
    pub core: StaticAnchorDesc,
    pub body_policy: StaticAnchorBodyPolicy,
}

impl StaticAnchorConnectionDesc {
    pub fn new(core: StaticAnchorDesc) -> Self {
        Self {
            core,
            body_policy: StaticAnchorBodyPolicy::Preserve,
        }
    }

    pub fn with_body_policy(mut self, body_policy: StaticAnchorBodyPolicy) -> Self {
        self.body_policy = body_policy;
        self
    }
}

impl From<StaticAnchorDesc> for StaticAnchorConnectionDesc {
    fn from(core: StaticAnchorDesc) -> Self {
        Self::new(core)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct DynamicStructuralConnectionDesc {
    pub core: DynamicStructuralBondDesc,
    pub policy: DynamicConnectionPolicy,
}

impl DynamicStructuralConnectionDesc {
    pub fn graph_only(core: DynamicStructuralBondDesc) -> Self {
        Self {
            core,
            policy: DynamicConnectionPolicy::GraphOnly,
        }
    }

    pub fn custom_hard_constraint(core: DynamicStructuralBondDesc) -> Self {
        Self {
            core,
            policy: DynamicConnectionPolicy::CustomHardConstraint,
        }
    }
}
