//! Rapier 2D adapter for the engine-neutral fracture crates.

mod collider_sync;
mod contact_map;
mod hooks;
mod impulse_readback;
mod joint_feedback;
mod pipeline;
pub mod world;

pub use collider_sync::{ActorPhysicsHandles, DestructibleActorRef, VoxelContact};
pub use contact_map::{ContactPairMapping, ContactPairSide};
pub use hooks::{ContactMaterialProperties, HookObservation};
pub use impulse_readback::{ContactImpulseInput, TrackedContactImpulse};
pub use joint_feedback::JointFeedbackStress;
pub use pipeline::FxStepReport;
pub use world::{FxRapierError, FxRapierWorld2D};

#[cfg(test)]
mod tests;
