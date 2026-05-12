# Phase 0 Reference Notes

This document locks the reference facts and vocabulary needed before Phase 1 `fracture_core` implementation. It is intentionally a facts file, not an implementation plan.

Phase 0 non-goals:

- Do not implement `fracture_core`.
- Do not change the Rapier public API or fork internals.
- Do not start Phase 2 runtime edit authoring, Phase 3 Rapier integration, or Phase 7 demos.

Primary project source: [fracture_engine_development_plan.md](../../fracture_engine_development_plan.md), especially sections 1, 3, 4, 8, 13, 14, 17, 19, and 20.

## Source Set

Use these links as the stable reference set for Phase 1 design decisions:

- Blast introduction: [online](https://docs.omniverse.nvidia.com/kit/docs/blast-sdk/latest/docs/api/introduction.html), [local](../../references/blast/docs/_source/introduction.txt).
- Blast low-level users guide: [online](https://docs.omniverse.nvidia.com/kit/docs/blast-sdk/latest/docs/api/api_ll_users_guide.html), [local](../../references/blast/docs/_source/api_ll_users_guide.txt).
- Blast stress extension: [online](https://nvidia-omniverse.github.io/PhysX/blast/docs/api/extensions/ext_stress.html), [local](../../references/blast/docs/_source/ext_stress.txt).
- Blast asset utilities: [online](https://docs.omniverse.nvidia.com/kit/docs/blast-sdk/latest/docs/api/extensions/ext_assetutils.html), [local](../../references/blast/docs/_source/ext_assetutils.txt).
- Rapier advanced collision detection: [online](https://rapier.rs/docs/user_guides/rust/advanced_collision_detection/), local code in [narrow_phase.rs](../src/geometry/narrow_phase.rs), [contact_pair.rs](../src/geometry/contact_pair.rs), and [physics_hooks.rs](../src/pipeline/physics_hooks.rs).
- Rapier colliders and voxels: [online](https://rapier.rs/docs/user_guides/rust/colliders/), local code and examples in [CHANGELOG.md](../CHANGELOG.md) and [examples3d/voxels3.rs](../examples3d/voxels3.rs).
- Rapier joints: [online constraints guide](https://rapier.rs/docs/user_guides/javascript/joint_constraints/), [online Rust joints guide](https://rapier.rs/docs/user_guides/rust/joints/), local code in [impulse_joint_set.rs](../src/dynamics/joint/impulse_joint/impulse_joint_set.rs).
- Rapier determinism: [online](https://rapier.rs/docs/user_guides/rust/determinism/), local feature definitions in [crates/rapier2d/Cargo.toml](../crates/rapier2d/Cargo.toml) and compile guard in [src/lib.rs](../src/lib.rs).

## Blast Facts

### Asset, Family, Actor

- A Blast asset contains the static destructible data. Assets are instanced into actors; damaged/fractured actors are broken into chunks, and connected groups of chunks become actors according to the asset support graph. Sources: [Blast introduction](https://docs.omniverse.nvidia.com/kit/docs/blast-sdk/latest/docs/api/introduction.html), [local introduction](../../references/blast/docs/_source/introduction.txt), [project plan](../../fracture_engine_development_plan.md).
- Actors live inside a family created from asset data. The family owns the initial actor and all descendant actors produced by recursive fracture. Sources: [Blast low-level guide](https://docs.omniverse.nvidia.com/kit/docs/blast-sdk/latest/docs/api/api_ll_users_guide.html), [local low-level guide](../../references/blast/docs/_source/api_ll_users_guide.txt).
- Blast low-level is physics-agnostic: collision geometry and physical joints are user-managed through user data. Phase 1 must keep `FxAsset`/`FxFamily`/`FxActor` independent from Rapier types, with only stable handles or bridge IDs at the integration boundary. Sources: [Blast low-level guide](https://docs.omniverse.nvidia.com/kit/docs/blast-sdk/latest/docs/api/api_ll_users_guide.html), [project plan section 3](../../fracture_engine_development_plan.md).

### Chunk, Support Graph, Exact Cover

- Chunks are hierarchical. A user may tag chunks as support chunks; support chunks form the graph used for actor connectivity after fracture. Sources: [Blast introduction](https://docs.omniverse.nvidia.com/kit/docs/blast-sdk/latest/docs/api/introduction.html), [local introduction](../../references/blast/docs/_source/introduction.txt).
- Blast requires support chunks to form an exact cover of the unfractured object: no missing occupied area and no overlapping support coverage. Phase 1 maps this to `FxAsset.support_nodes` covering every occupied voxel exactly once. Sources: [Blast introduction](https://docs.omniverse.nvidia.com/kit/docs/blast-sdk/latest/docs/api/introduction.html), [local introduction](../../references/blast/docs/_source/introduction.txt), [project plan section 7](../../fracture_engine_development_plan.md).
- Support chunks joined by unbroken bonds belong to the same actor island. When bonds break, island detection over the support graph determines new actor membership. Sources: [Blast introduction](https://docs.omniverse.nvidia.com/kit/docs/blast-sdk/latest/docs/api/introduction.html), [project plan section 13](../../fracture_engine_development_plan.md).

### Bond and External World Bond

- A Blast bond represents the surface joining neighboring support chunks. It carries geometric data such as centroid, average normal, and area; the project maps `area` to 2D interface `length`. Sources: [Blast introduction](https://docs.omniverse.nvidia.com/kit/docs/blast-sdk/latest/docs/api/introduction.html), [local introduction](../../references/blast/docs/_source/introduction.txt), [project plan section 6](../../fracture_engine_development_plan.md).
- Bonds may connect two support chunks or a support chunk to the world. There is no real world chunk; the external endpoint represents an environment connection, and world-bound actors may be kept static or kinematic by the user. Sources: [Blast introduction](https://docs.omniverse.nvidia.com/kit/docs/blast-sdk/latest/docs/api/introduction.html), [local introduction](../../references/blast/docs/_source/introduction.txt).
- A world bond descriptor can use the invalid chunk index. This creates a graph node that does not correspond to a chunk; actors containing it can be treated as world-bound. Sources: [Blast low-level guide](https://docs.omniverse.nvidia.com/kit/docs/blast-sdk/latest/docs/api/api_ll_users_guide.html), [local low-level guide](../../references/blast/docs/_source/api_ll_users_guide.txt), [Blast release notes local](../../references/blast/docs/release_notes.txt).
- Blast asset utilities can add world bonds and can merge assets with new bonds joining support chunks across assets. Phase 1 only needs the graph concept; full dynamic merge API is a later phase. Sources: [Blast asset utilities](https://docs.omniverse.nvidia.com/kit/docs/blast-sdk/latest/docs/api/extensions/ext_assetutils.html), [local asset utilities](../../references/blast/docs/_source/ext_assetutils.txt), [project plan section 15](../../fracture_engine_development_plan.md).

### Damage Staged Flow

- Blast damage is health loss on bonds and chunks, driven by user-defined material/shader functions that can interpret bond geometry and impact location. Sources: [Blast introduction](https://docs.omniverse.nvidia.com/kit/docs/blast-sdk/latest/docs/api/introduction.html), [local introduction](../../references/blast/docs/_source/introduction.txt).
- Low-level Blast fracture is staged: generate fracture commands from a damage program, apply fracture commands to mutate actor/family health state, then split the actor into children. Split is not performed by apply, and multiple applies may happen before split. Sources: [Blast low-level guide](https://docs.omniverse.nvidia.com/kit/docs/blast-sdk/latest/docs/api/api_ll_users_guide.html), [local low-level guide](../../references/blast/docs/_source/api_ll_users_guide.txt), [project plan sections 8 and 13](../../fracture_engine_development_plan.md).
- Blast split reports new actors and may report the old actor as deleted. This project deliberately changes that identity rule: the largest fragment keeps the parent `FxActor` and future Rapier handle mapping; smaller fragments become children. Sources: [Blast low-level guide](https://docs.omniverse.nvidia.com/kit/docs/blast-sdk/latest/docs/api/api_ll_users_guide.html), [project plan section 13.2](../../fracture_engine_development_plan.md).

### Stress

- Blast `NvBlastExtStress` works directly on the bond graph and is physics-library independent. It needs per-node physical data such as mass, volume, position, and static/world-bound state because Blast itself does not own physics. Sources: [Blast stress extension](https://nvidia-omniverse.github.io/PhysX/blast/docs/api/extensions/ext_stress.html), [local stress extension](../../references/blast/docs/_source/ext_stress.txt), [project plan section 11](../../fracture_engine_development_plan.md).
- Stress can turn impact into graph-level breakage at weak places instead of only breaking at the contact point. Phase 1 must allow damage/stress commands that target remote weak bonds after graph propagation. Sources: [Blast stress extension](https://nvidia-omniverse.github.io/PhysX/blast/docs/api/extensions/ext_stress.html), [local stress extension](../../references/blast/docs/_source/ext_stress.txt), [project plan section 20.2](../../fracture_engine_development_plan.md).
- Stress solve flow is: notify actor create/destroy after split, add forces/gravity/angular velocity, update stress, generate fracture commands for overstressed bonds, apply them through the normal fracture flow. Sources: [Blast stress extension](https://nvidia-omniverse.github.io/PhysX/blast/docs/api/extensions/ext_stress.html), [local stress extension](../../references/blast/docs/_source/ext_stress.txt), [project plan section 11](../../fracture_engine_development_plan.md).

## Rapier Facts

### Broad Phase and Narrow Phase

- Rapier collision detection is divided into broad phase and narrow phase; contact pairs/manifolds live in the narrow phase and are the source for detailed contact geometry. Sources: [Rapier advanced collision detection](https://rapier.rs/docs/user_guides/rust/advanced_collision_detection/), local [broad_phase_bvh.rs](../src/geometry/broad_phase_bvh.rs), local [narrow_phase.rs](../src/geometry/narrow_phase.rs).
- `NarrowPhase` stores an interaction graph of `ContactPair` values and exposes `contact_pairs_with`, `contact_pair`, and `contact_pairs`. Phase 3 will use these APIs; Phase 1 must not depend on them directly. Sources: local [narrow_phase.rs](../src/geometry/narrow_phase.rs), [Rapier advanced collision detection](https://rapier.rs/docs/user_guides/rust/advanced_collision_detection/).

### Contact Pair, Manifold, and Tracked Contact Impulse

- A `ContactPair` stores `collider1`, `collider2`, and a vector of `ContactManifold`. The pair order must be read from the returned pair, not assumed from query argument order. Sources: local [contact_pair.rs](../src/geometry/contact_pair.rs), [Rapier advanced collision detection](https://rapier.rs/docs/user_guides/rust/advanced_collision_detection/), [project plan section 9](../../fracture_engine_development_plan.md).
- Geometric contacts are persistent/tracked and contain `ContactData` fields including normal impulse and tangent impulse. Local source names this data in `ContactData`, and aggregate helpers sum `pt.data.impulse`. Sources: local [contact_pair.rs](../src/geometry/contact_pair.rs), [Rapier contact type](https://docs.rs/rapier2d/latest/rapier2d/geometry/type.Contact.html), [Rapier advanced collision detection](https://rapier.rs/docs/user_guides/rust/advanced_collision_detection/).
- Solver contacts are transient. Rapier writes solver results back into manifold contact data during contact constraint writeback. Phase 3 must build the mapping before solve and read `data.impulse`/`data.tangent_impulse` after solve; Phase 1 should consume an engine-neutral `ContactImpulseInput`. Sources: local [generic_contact_constraint.rs](../src/dynamics/solver/contact_constraint/generic_contact_constraint.rs), local [contact_with_coulomb_friction.rs](../src/dynamics/solver/contact_constraint/contact_with_coulomb_friction.rs), [Rapier advanced collision detection](https://rapier.rs/docs/user_guides/rust/advanced_collision_detection/).

### Contact Modification Hook

- Rapier `PhysicsHooks::modify_solver_contacts` receives a `ContactModificationContext` with solver contacts for an existing manifold. It can modify solver contact properties such as friction/restitution through the provided context, but it is not the mechanism for creating new contact points. Sources: local [physics_hooks.rs](../src/pipeline/physics_hooks.rs), local [narrow_phase.rs](../src/geometry/narrow_phase.rs), [Rapier advanced collision detection](https://rapier.rs/docs/user_guides/rust/advanced_collision_detection/).
- Phase 3 contact material work should live at this hook boundary. Phase 1 only locks material IDs and damage/stress inputs; it must not require Rapier hook types. Sources: [project plan sections 9 and 12](../../fracture_engine_development_plan.md), local [physics_hooks.rs](../src/pipeline/physics_hooks.rs).

### ImpulseJointSet Feedback

- The project MVP uses `ImpulseJointSet` feedback only. Rapier has an explicit `ImpulseJointSet` type and solver writeback paths for joint impulses. Sources: local [impulse_joint_set.rs](../src/dynamics/joint/impulse_joint/impulse_joint_set.rs), local [generic_joint_constraint.rs](../src/dynamics/solver/joint_constraint/generic_joint_constraint.rs), [Rapier joint constraints](https://rapier.rs/docs/user_guides/javascript/joint_constraints/).
- Multibody joint force feedback is not part of the MVP boundary. Phase 1 should model `JointFeedback` as an engine-neutral stress input, not as a direct joint dependency. Sources: [Rapier joint constraints](https://rapier.rs/docs/user_guides/javascript/joint_constraints/), [project plan section 10](../../fracture_engine_development_plan.md).

### Voxel Colliders

- Rapier/Parry has voxel collider support through builder APIs such as `ColliderBuilder::voxels`, `voxels_from_points`, and `voxelized_mesh`; the local 3D example builds a voxel collider from sampled points. Sources: local [CHANGELOG.md](../CHANGELOG.md), local [examples3d/voxels3.rs](../examples3d/voxels3.rs), [Rapier colliders guide](https://rapier.rs/docs/user_guides/rust/colliders/).
- Phase 1 must not build colliders. It must define enough local voxel/node/bond identity for Phase 3 to rebuild dirty voxel colliders and map Rapier features back to voxels/nodes/bonds. Sources: [project plan sections 9 and 16](../../fracture_engine_development_plan.md), [Rapier colliders guide](https://rapier.rs/docs/user_guides/rust/colliders/).

### Determinism Constraints

- Rapier deterministic behavior requires the same initial state and same construction/add/remove order; enhanced determinism is a feature flag. Sources: [Rapier determinism](https://rapier.rs/docs/user_guides/rust/determinism/), local [crates/rapier2d/Cargo.toml](../crates/rapier2d/Cargo.toml), [project plan section 17](../../fracture_engine_development_plan.md).
- Local Rapier source has a compile guard rejecting simultaneous SIMD and `enhanced-determinism`. Phase 1 deterministic mode must therefore avoid relying on parallel/SIMD iteration behavior for fracture ordering. Sources: local [src/lib.rs](../src/lib.rs), [Rapier determinism](https://rapier.rs/docs/user_guides/rust/determinism/), [project plan section 17.2](../../fracture_engine_development_plan.md).
- Deterministic fracture ordering is a project-level obligation beyond Rapier: commands, actor traversal, island traversal, fragment selection, health transfer, and reductions must have stable order. Sources: [project plan sections 17 and 20.5](../../fracture_engine_development_plan.md), [Rapier determinism](https://rapier.rs/docs/user_guides/rust/determinism/).

## Locked Vocabulary

| Project term | Locked meaning | Primary sources |
|---|---|---|
| `FxAsset` | Immutable authoring/runtime resource containing dense voxel identity, chunk hierarchy, support nodes, and internal bonds. | [Plan section 4.1](../../fracture_engine_development_plan.md), [Blast introduction](https://docs.omniverse.nvidia.com/kit/docs/blast-sdk/latest/docs/api/introduction.html) |
| `FxFamily` | Runtime instance state for an asset: actors, node ownership, bond/chunk health, external bonds, IDs, and snapshots. | [Plan section 4.1](../../fracture_engine_development_plan.md), [Blast low-level guide](https://docs.omniverse.nvidia.com/kit/docs/blast-sdk/latest/docs/api/api_ll_users_guide.html) |
| `FxActor` | Active connected island of support nodes with cached mass/COM/inertia and future physics handle mapping. | [Plan section 4.1](../../fracture_engine_development_plan.md), [Blast introduction](https://docs.omniverse.nvidia.com/kit/docs/blast-sdk/latest/docs/api/introduction.html) |
| `SupportNode2D` | 2D support graph node representing a connected set of occupied voxels, not a single voxel. | [Plan sections 4.1 and 7](../../fracture_engine_development_plan.md), [Blast introduction](https://docs.omniverse.nvidia.com/kit/docs/blast-sdk/latest/docs/api/introduction.html) |
| `Bond2D` | Internal graph edge between two support nodes, with centroid, normal, 2D length, material pair, lineage, and runtime health. | [Plan section 6](../../fracture_engine_development_plan.md), [Blast introduction](https://docs.omniverse.nvidia.com/kit/docs/blast-sdk/latest/docs/api/introduction.html) |
| `ExternalBond2D` | Runtime graph edge from a support node to world/static/kinematic/dynamic external target; it is not a default soft joint. | [Plan section 15](../../fracture_engine_development_plan.md), [Blast asset utilities](https://docs.omniverse.nvidia.com/kit/docs/blast-sdk/latest/docs/api/extensions/ext_assetutils.html) |
| `DamageCommand` | Input command/event for damage generation; generate stage must not mutate family state. | [Plan sections 4.1 and 12](../../fracture_engine_development_plan.md), [Blast low-level guide](https://docs.omniverse.nvidia.com/kit/docs/blast-sdk/latest/docs/api/api_ll_users_guide.html) |
| `FractureCommand` | Generated health/effective-length mutation request targeting bonds/chunks/nodes; applied in deterministic order. | [Plan sections 12 and 13](../../fracture_engine_development_plan.md), [Blast low-level guide](https://docs.omniverse.nvidia.com/kit/docs/blast-sdk/latest/docs/api/api_ll_users_guide.html) |
| `Split` | Island detection after fracture application; project rule keeps the largest fragment as parent. | [Plan section 13.2](../../fracture_engine_development_plan.md), [Blast low-level guide](https://docs.omniverse.nvidia.com/kit/docs/blast-sdk/latest/docs/api/api_ll_users_guide.html) |
| `Rapier integration boundary` | Engine-neutral inputs/outputs between Phase 1 core and future Rapier contact/joint/collider synchronization. | [Plan sections 8, 9, 10, and 16](../../fracture_engine_development_plan.md), [Rapier advanced collision detection](https://rapier.rs/docs/user_guides/rust/advanced_collision_detection/) |

## Phase 1 Design Consequences

- `fracture_core` must be fully usable in memory without Rapier. Rapier-specific contact, joint, and collider operations enter only as future adapter inputs.
- The minimum closed loop is `DamageCommand` or stress input -> generate `FractureCommand` -> apply health/effective-length mutation -> split islands -> emit deterministic events.
- Exact cover and stable node/bond IDs are not optional; they are prerequisites for support graph tests, split tests, runtime edit health transfer, and deterministic replay.
- The largest-fragment-keeps-parent policy overrides Blast's default old-actor-deleted split identity. This is a project invariant, not a reference fact from Blast.
- Runtime edit repair is only a Phase 2 implementation target, but Phase 1 data must reserve lineage, stable IDs, and health-transfer semantics so Phase 2 does not need to reinterpret existing health.
