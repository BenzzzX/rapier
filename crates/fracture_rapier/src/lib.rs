//! Rapier 2D adapter for the engine-neutral fracture crates.

#[cfg(all(feature = "deterministic-replay", feature = "parallel"))]
compile_error!("fracture_rapier feature `deterministic-replay` is incompatible with `parallel`");
#[cfg(all(feature = "deterministic-replay", feature = "simd-stable"))]
compile_error!("fracture_rapier feature `deterministic-replay` is incompatible with `simd-stable`");
#[cfg(all(feature = "deterministic-replay", feature = "simd-nightly"))]
compile_error!(
    "fracture_rapier feature `deterministic-replay` is incompatible with `simd-nightly`"
);

mod collider_sync;
mod connect_api;
mod contact_map;
mod hooks;
mod impulse_readback;
mod joint_feedback;
mod pipeline;
pub mod replay;
pub mod snapshot;
pub mod world;

pub use collider_sync::{
    ActorColliderBuildKind, ActorPhysicsHandles, ColliderLodSettings, DestructibleActorRef,
    FxPhysicsSyncReport, VoxelContact,
};
pub use connect_api::{
    DynamicStructuralConnectionDesc, StaticAnchorBodyPolicy, StaticAnchorConnectionDesc,
};
pub use contact_map::{ContactPairMapping, ContactPairSide};
pub use hooks::{ContactMaterialProperties, HookObservation};
pub use impulse_readback::{ContactImpulseInput, TrackedContactImpulse};
pub use joint_feedback::JointFeedbackStress;
pub use pipeline::{
    ACTIVE_BODY_BUDGET, FxGlobalStressCapReport, FxPerformanceBudgetReport, FxStepDiagnostics,
    FxStepReport, FxStepWithDiagnostics, OCCUPIED_VOXEL_BUDGET, SUPPORT_NODE_BUDGET,
};
pub use replay::{
    FxRapierReplayCommand, FxRapierReplayTickReport, ReplayTrace, ReplayTraceActorBody,
};
pub use snapshot::{FxRapierSnapshotError, SnapshotReplayMode};
pub use world::{FxRapierError, FxRapierWorld2D};

#[cfg(test)]
mod tests;
