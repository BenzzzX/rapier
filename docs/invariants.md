# Phase 0 Invariants

This document turns the Phase 0 reference lock into testable obligations for Phase 1 `fracture_core`. It should be treated as the acceptance contract for the first in-memory implementation.

Phase 0 non-goals:

- Do not implement `fracture_core` in this phase.
- Do not change Rapier APIs or fork internals in this phase.
- Do not implement Phase 2 runtime edit repair, Phase 3 Rapier integration, or Phase 7 demos in this phase.

Primary sources:

- [fracture_engine_development_plan.md](../../fracture_engine_development_plan.md)
- [reference_notes.md](reference_notes.md)
- Blast staged fracture and support model: [online](https://docs.omniverse.nvidia.com/kit/docs/blast-sdk/latest/docs/api/api_ll_users_guide.html), [local](../../references/blast/docs/_source/api_ll_users_guide.txt), [local intro](../../references/blast/docs/_source/introduction.txt)
- Rapier collision/joint/determinism boundaries: [advanced collision detection](https://rapier.rs/docs/user_guides/rust/advanced_collision_detection/), [determinism](https://rapier.rs/docs/user_guides/rust/determinism/), local [narrow_phase.rs](../src/geometry/narrow_phase.rs), local [contact_pair.rs](../src/geometry/contact_pair.rs)

## Invariant Style

Each invariant below is a test obligation. Phase 1 should name tests close to the suggestions here unless the implementation layout demands a clearer local name.

Use these categories:

- Required for Phase 1: must pass before core implementation is considered complete.
- Phase 2 forward invariant: not fully implemented in Phase 1, but Phase 1 data structures must not make it impossible or ambiguous.
- Rapier boundary invariant: Phase 1 must keep the core independent from Rapier while preserving enough IDs and data for later integration.

## `FxAsset`

Required for Phase 1:

- `FxAsset` is immutable after creation except through explicit asset rebuild tooling outside Phase 1 runtime mutation.
- `voxel_size > 0`.
- Every occupied voxel is covered by exactly one `SupportNode2D`.
- No empty voxel is covered by a `SupportNode2D`.
- `support_nodes` have stable IDs. Serialization/deserialization or deterministic rebuild of the same input must not renumber nodes arbitrarily.
- `internal_bonds` only reference valid support node IDs from the same asset.
- `chunk_hierarchy` must represent an exact cover over support nodes: no support node is missing from the hierarchy and no support node is owned twice.
- Material/contact/external/orientation maps, when present, must have the same dimensions as `occupancy_grid`.

Suggested tests:

- `exact_cover_basic` from plan 20.1: occupied grid plus cluster map covers each occupied voxel exactly once.
- `asset_rejects_overlapping_support_nodes`: overlapping support spans fail validation.
- `asset_rejects_missing_support_coverage`: occupied voxel without support node fails validation.
- `asset_rejects_bond_endpoint_out_of_range`: bond endpoint not in `support_nodes` fails validation.
- `asset_stable_node_ids_on_rebuild`: same deterministic input produces same node IDs.

## `FxFamily`

Required for Phase 1:

- `FxFamily.asset_ref` points to one immutable `FxAsset`.
- Every active support node has exactly one active owner in `node_owner`.
- No active `FxActor` owns a node outside the family asset.
- Active unbroken internal bonds only connect nodes that still exist and are owned by active actors.
- Runtime `bond_health`, `effective_length`, and `chunk_health` arrays have deterministic indexing keyed by asset or stable runtime IDs.
- Broken bonds do not contribute to island connectivity.
- Family ID allocation is deterministic in deterministic mode and monotonic within a core-local test state.
- Phase 1 must expose enough core-local state for determinism tests: actor ownership, bond health, effective length, chunk health, stable ID allocator state, and pending command order keys must be readable through a `deterministic_state_digest`, debug dump, or serializable test state. Phase 4 extends that deterministic state to external bonds and dynamic structural connections.
- Full binary checkpoint format, full replay module, and long-running restore validation are Phase 5 responsibilities, not Phase 1 requirements.

Suggested tests:

- `family_node_owner_unique`: duplicate active ownership is rejected.
- `family_unowned_active_node_rejected`: active node without owner is rejected.
- `apply_fracture_mutates_health` from plan 20.2: command buffer mutates health and emits ordered deltas.
- `deterministic_state_digest_stable`: same initial core state and same ordered inputs produce the same digest/debug dump.
- Phase 5 test mapping: `snapshot_restore` belongs to the future binary checkpoint/replay module unless Phase 1 explicitly adds a narrow serializable test-state helper.

## `FxActor`

Required for Phase 1:

- `FxActor.owned_nodes` is non-empty for every active actor.
- `owned_nodes` is connected through currently unbroken internal bonds, except during an explicitly marked dirty/commit window.
- `owned_nodes` contains only nodes whose `FxFamily.node_owner` is this actor.
- Cached mass, center of mass, inertia, and voxel bounds are derived from owned nodes and must be rebuilt after split.
- A parent actor identity may survive split only if it owns the selected largest fragment.
- Newly created child actors receive stable IDs in deterministic order.

Suggested tests:

- `actor_owned_nodes_connected`: disconnected ownership fails validation before split.
- `largest_fragment_keeps_parent` from plan 20.1: largest island remains the parent actor.
- `split_child_ids_are_stable`: identical split input creates children in identical order and with identical IDs.
- `replay_split_order` from plan 20.5: split events and body-mapping placeholders are identical for same seed/input.

## `SupportNode2D`

Required for Phase 1:

- A support node contains at least one occupied voxel.
- Voxels inside one support node are 4-neighbor connected unless a future authoring flag explicitly records an allowed exception. Diagonal contact alone is not connectivity.
- A support node has one stable `node_id`, one `chunk_id` or equivalent hierarchy association, and a deterministic voxel span/bounds.
- `material_summary`, anisotropy frame, and stable seed are deterministic functions of source maps and authoring parameters.
- Runtime repair must not reuse a `node_id` to mean a completely unrelated region.

Suggested tests:

- `four_neighbor_connectivity` from plan 20.1: diagonal-touch voxels are not connected.
- `support_node_rejects_empty_span`: empty node fails validation.
- `support_node_material_summary_stable`: same source material map gives same summary and seed.
- Phase 2 forward test name: `remove_voxel_splits_node` from plan 20.3 must preserve lineage for surviving parts.

## `Bond2D`

Required for Phase 1:

- `Bond2D.node_a != Bond2D.node_b`.
- Each `Bond2D` endpoint exists in the same family/asset context.
- `Bond2D.length > 0` and `effective_length >= 0`.
- `Bond2D.health >= 0`.
- A bond is considered broken when `health <= 0` or `effective_length <= epsilon`.
- Broken bonds are ignored by island connectivity and stress propagation.
- Interface geometry is deterministic: centroid, normal, tangent, and length are computed from sorted interface edges or a stable equivalent.
- Disconnected interface islands between the same node pair are separate bonds, not one merged bond.

Suggested tests:

- `bond_generation_edge_scan` from plan 20.1: two nodes sharing edges produce length equal to shared edge count times `voxel_size`.
- `disconnected_interface_expands_bonds` from plan 20.1: separated interfaces generate independent bonds.
- `bond_rejects_self_loop`: `node_a == node_b` fails validation.
- `stress_tension_break` and `stress_shear_break` from plan 20.2: over-limit forces produce fracture commands.

## `ExternalBond2D`

Phase 4 implemented invariant:

- `ExternalBond2D` has exactly one internal support node endpoint and one external target endpoint represented by an engine-neutral target kind/token, not a Rapier handle.
- `FxFamily` exposes explicit APIs for static/world external anchors, same-family actor merge, and GraphOnly dynamic structural bonds.
- External bond descriptors must reference owned support nodes, unique external bond IDs, finite vectors, and finite nonnegative runtime scalars.
- External bond target, anchor, normal/tangent, limits, and runtime state are part of the deterministic core digest.
- Static/world external endpoints act as fixed/infinite endpoints for core stress/fracture while their external bond remains unbroken.
- Broken external bonds are ignored by stress, damage generation, and direct fracture apply.
- The Rapier adapter may apply an explicit static-anchor body policy (`Fixed` or kinematic) while the owning actor has a live unbroken anchor, and must reconcile that policy after split/sync and after external bond damage. The default policy preserves a dynamic body.
- Dynamic structural bonds are limited to node references in the same `FxFamily` and same asset. Descriptors must use unique IDs, owned endpoints, finite vectors, finite nonnegative runtime scalars, and no self-connection.
- GraphOnly dynamic structural bonds affect core graph/stress/fracture state but do not create default Rapier soft, fixed, spring, or impulse joints.
- Broken dynamic structural bonds are ignored by graph traversal, stress, damage generation, and direct fracture apply.
- Dynamic merge is same-family/same-asset only and requires an unbroken GraphOnly dynamic structural bond between the two actors. A merge without that connection must return an explicit error without core/Rapier side effects.
- A successful merge must preserve the active actor `owned_nodes` graph-connectivity invariant. It must not create a clean active actor with disconnected ownership.
- The Rapier adapter merges two actor bodies into one kept actor, removes stale body/collider handles, and preserves mass/COM/linear velocity. Angular velocity uses a deterministic mass-weighted fallback rather than full angular momentum transfer.
- `CustomHardConstraint` is explicitly unsupported in Phase 4 and must return an error without side effects.
- Phase 4 does not implement cross-family/cross-asset merge or Phase 5 snapshot/replay/checkpoint behavior.

Phase 4 tests:

- Static anchors are covered by `static_anchor_stress_fixed_endpoint`, `static_anchor_marks_actor_fixed_or_kinematic`, `static_anchor_policy_moves_to_split_child`, and `broken_static_anchor_clears_body_policy`.
- Dynamic merge is covered by `same_family_actor_merge_preserves_connection_state`, `merge_actors_requires_unbroken_graph_connection`, `merge_preserves_dirty_split_obligation`, `dynamic_merge_conserves_mass_com_velocity`, and `dynamic_merge_requires_graph_connection_no_side_effects`.
- GraphOnly dynamic bonds are covered by `dynamic_bond_graph_only`, `graph_only_connection_stress_generates_once_for_two_endpoint_inputs`, and `dynamic_bond_custom_hard_constraint_is_future_error`.
- Connection determinism, broken-connection behavior, and repair validation are covered by `phase4_digest_includes_connection_state`, `broken_external_and_dynamic_connections_ignore_direct_damage`, `connection_validation_rejects_invalid_inputs`, `repair_plan_rejects_removed_external_bond_endpoint`, and `repair_plan_rejects_removed_dynamic_connection_endpoint`.

## `DamageCommand` and `FractureCommand` Flow

Required for Phase 1:

- `DamageCommand` collection is deterministic before generation. Each command has a deterministic order key: `(tick, source_priority, family_id, actor_id, command_id)` or a stricter stable equivalent.
- Generate stage reads actor/family/material/stress context and writes `FractureCommand` values only. It must not mutate family state, actor ownership, health, or split state.
- `FractureCommand` values identify targets by stable IDs, not transient iteration positions.
- Apply stage is the only stage that mutates bond/chunk/node health and effective length.
- Apply stage clamps health/effective length at zero and emits old/new health events in deterministic order.
- Split stage runs after apply, not during generate or apply.
- Multiple apply passes may occur before split only if the command ordering and dirty actor set remain deterministic and tests cover the sequence.

Suggested tests:

- `damage_generate_no_mutation` from plan 20.2: generation does not change family state.
- `apply_fracture_mutates_health` from plan 20.2: applying commands changes health and emits expected events.
- `damage_commands_sorted_before_generate`: shuffled input produces identical generated commands.
- `fracture_commands_target_stable_ids`: applying commands by stable ID works after unrelated vector order changes.
- `deterministic_sort_commands` from plan 20.5: shuffled command input sorts identically.

## `EventStream`, `FractureEvent`, and `SplitEvent`

Required for Phase 1:

- `EventStream` event order is deterministic in deterministic mode and follows the same sorted command/apply/split order as state mutation.
- Every emitted event has a stable `event_id`, source command/input ID, family ID, actor ID where applicable, and deterministic tick/sequence metadata.
- `FractureEvent` records target kind and stable target ID, source, old health/effective length, new health/effective length, and enough position/material context for gameplay/debug consumers.
- `SplitEvent` records parent actor, kept actor, created child actors, deterministic node sets per fragment, and parent-retention reason/key.
- Event emission is a consequence of apply/split stages; generate stage may produce debug traces but must not emit committed health/split events.

Suggested tests:

- `apply_fracture_mutates_health` from plan 20.2: assert health mutation and matching `FractureEvent` old/new values.
- `replay_split_order` from plan 20.5: assert deterministic `SplitEvent` order and payload.
- `deterministic_event_ordering`: shuffled equivalent command input produces identical committed event sequence after sorting.

## Split and Parent Identity

Required for Phase 1:

- Split builds components over an actor's owned support nodes using only unbroken connectivity.
- If there is one component, no new actor is created and parent identity remains unchanged.
- If there are multiple components, select the kept parent fragment using:
  1. `voxel_count` descending.
  2. `total_mass` descending.
  3. `min_stable_node_id` ascending.
- The selected fragment keeps the original `FxActorId` and parent mapping placeholder.
- All other fragments become new child actors in deterministic component order.
- Children inherit velocity/momentum data only through an explicit Phase 1 physics-neutral policy object or placeholder; Phase 1 must not require Rapier `RigidBodyHandle`.
- Split emits a deterministic `SplitEvent` with `parent_actor`, `kept_actor`, `created_children`, and node sets.

Suggested tests:

- `largest_fragment_keeps_parent` from plan 20.1.
- `largest_fragment_tie_breaks_by_min_node_id`: equal voxel and mass tie chooses smallest stable node ID.
- `split_single_island_noop`: no new actor or event for one connected island.
- `replay_split_order` from plan 20.5.

## Runtime Edit Repair Health Preservation

Phase 2 forward invariant, but Phase 1 data must support it:

- Runtime edits are represented as queued commands/journal entries, not direct mutation from gameplay callbacks.
- Phase 1 node and bond state must include enough lineage or stable matching data to transfer health during Phase 2 repair.
- Removing voxels from an old node must allow surviving connected parts to inherit old health weighted by surviving overlap.
- Adding voxels must initialize new health without resetting surviving old node or bond damage.
- Rebuilt bonds must be matchable by endpoint lineage and interface edge overlap so surviving bond damage is preserved.
- Full recluster is not the default runtime edit repair path; explicit rebuild may discard lineage only when called as such.
- After any future edit commit, exact cover must be revalidated before split.

Suggested tests:

- `remove_voxel_splits_node` from plan 20.3: deletion splits node and transfers old health by overlap.
- `add_voxel_no_auto_merge` from plan 20.3: added voxel between actors does not auto-merge without explicit weld.
- `repair_preserves_bond_damage` from plan 20.3: surviving damaged interface does not reset.
- `runtime_edit_revalidates_exact_cover`: edited asset/family overlay preserves exact cover.
- `repair_preserves_node_lineage`: surviving regions retain lineage for deterministic future matching.

## Stress Invariants

Required for Phase 1:

- Stress operates on support graph nodes and bonds, not on raw voxels alone.
- Stress inputs may come from contact impulse, joint feedback, scripted forces, radial damage, or gravity, but Phase 1 consumes them as engine-neutral graph inputs.
- Stress solve order and reductions are deterministic in deterministic mode.
- Stress fracture output is a `FractureCommand` buffer; stress does not directly break bonds outside apply.
- Tension, compression, and shear limits are represented separately or through a material model that can test them separately.
- A remote weak bond may break because of propagated stress even when the local contact region is stronger.

Suggested tests:

- `stress_tension_break` from plan 20.2.
- `stress_shear_break` from plan 20.2.
- `impact_breaks_weak_place` from plan 20.2.
- `stress_generate_commands_no_direct_mutation`: stress output does not mutate family until apply.
- `deterministic_stress_reduce_order`: same inputs with shuffled source order produce same commands in deterministic mode.

## Determinism Ordering

Required for Phase 1:

- Deterministic mode must not iterate hash maps or unordered sets directly when output order or floating reduction order matters.
- All external commands must be assigned stable order keys before generation.
- Actor traversal sorts by stable family ID then actor ID.
- Node traversal sorts by stable node ID.
- Bond traversal sorts by stable bond ID or deterministic endpoint/interface key.
- Component discovery must produce stable component ordering independent of adjacency vector insertion order.
- Floating accumulation must use serial deterministic order or an explicitly deterministic reduction tree.
- Normal mode may use faster ordering, but tests must not assert bit-exact replay for normal mode.

Suggested tests:

- `deterministic_sort_commands` from plan 20.5.
- `replay_split_order` from plan 20.5.
- `deterministic_state_digest_stable`: same core-local state and ordered inputs produce identical `deterministic_state_digest` values.
- Phase 5 mapping: `snapshot_restore` validates the future checkpoint/replay module and must not force Phase 1 to define the final snapshot format.
- `normal_mode_not_assert_exact` from plan 20.5.
- `deterministic_component_order_independent_of_adjacency`: shuffled adjacency gives same split events.

## Rapier Integration Boundary

Rapier boundary invariant for Phase 1:

- `fracture_core` must not depend on `RigidBodyHandle`, `ColliderHandle`, `NarrowPhase`, `ContactPair`, `ContactManifold`, `PhysicsHooks`, or `ImpulseJointSet`.
- `fracture_core` may later live as `rapier/crates/fracture_core` and be a workspace member, but it must not depend on `rapier2d`, `rapier3d`, or shared `src/`, and Phase 1 must not modify existing Rapier public APIs.
- Phase 1 may define bridge-neutral structs such as `ContactImpulseInput`, `JointFeedbackInput`, `ColliderSyncRequest`, or `PhysicsHandleToken` only if they do not import Rapier.
- Phase 1 `ContactImpulseInput` synthetic fixtures only need to express point, normal, normal/tangent impulse, material, source command/key, and target hints such as voxel/node/bond.
- Full Rapier pair/manifold/contact identity preservation, including pair side, manifold/contact index, feature ID, and tracked-contact readback keys, is a Phase 3 adapter obligation.
- Joint feedback input must identify actor/node/bond target and force/torque in core units without assuming multibody feedback.
- Collider sync output must describe changed actor fragments and voxel/primitive collider needs without building Rapier colliders.
- Same-step late fracture rule is locked for future integration: no mid-solver topology mutation.

Suggested future-facing tests:

- Phase 1 boundary test `core_has_no_rapier_dependency`: `fracture_core` builds without importing Rapier crates.
- Phase 1 boundary test `contact_impulse_input_maps_to_node_and_bond`: synthetic contact impulse can target graph damage/stress.
- Phase 3 future tests from plan 20.4 remain outside Phase 1 implementation: `contact_mapping_pair_order`, `tracked_impulse_readback`, `contact_hook_material`, `joint_feedback_stress`, and `same_step_split_sync`.

## Phase 1 Gate Checklist

Phase 1 should not pass until these plan 20.1, 20.2, 20.3, and 20.5 obligations are either implemented as tests or explicitly deferred with a reason:

| Plan section | Test obligation | Phase 1 status expectation |
|---|---|---|
| 20.1 | `exact_cover_basic` | Required |
| 20.1 | `four_neighbor_connectivity` | Required |
| 20.1 | `bond_generation_edge_scan` | Required if Phase 1 authors bonds from fixtures; otherwise required in first authoring step |
| 20.1 | `disconnected_interface_expands_bonds` | Required if Phase 1 authors bonds from fixtures; otherwise required in first authoring step |
| 20.1 | `largest_fragment_keeps_parent` | Required |
| 20.2 | `damage_generate_no_mutation` | Required |
| 20.2 | `apply_fracture_mutates_health` | Required |
| 20.2 | `stress_tension_break` | Required |
| 20.2 | `stress_shear_break` | Required |
| 20.2 | `impact_breaks_weak_place` | Required |
| 20.3 | `remove_voxel_splits_node` | Phase 2 forward invariant; Phase 1 data lineage must support it |
| 20.3 | `add_voxel_no_auto_merge` | Phase 2 forward invariant; no implicit merge behavior in Phase 1 |
| 20.3 | `repair_preserves_bond_damage` | Phase 2 forward invariant; bond lineage/effective length must support it |
| 20.3 | `explicit_static_anchor` | Implemented in Phase 4 static-anchor connection tests |
| 20.3 | `dynamic_merge_conserves_momentum` | Implemented for Phase 4 mass/COM/linear velocity; angular velocity uses documented deterministic fallback |
| 20.5 | `deterministic_sort_commands` | Required |
| 20.5 | `replay_split_order` | Required |
| 20.5 | `snapshot_restore` | Phase 5 checkpoint/replay responsibility; Phase 1 only needs `deterministic_state_digest` or an equivalent core-local debug/test state |
| 20.5 | `normal_mode_not_assert_exact` | Required as a testing policy |

The core implementation should prefer small deterministic fixtures for these tests before any large demo scene.
