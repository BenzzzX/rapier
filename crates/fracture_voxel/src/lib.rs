//! Dense voxel authoring and local runtime edit repair for `fracture_core`.
//!
//! This crate is deliberately engine-neutral. It does not depend on Rapier or
//! Parry; it builds `FxAsset` values and commits repaired topology through the
//! narrow core repair API.

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use fracture_core::{
    BondId, BondRuntimeState, Chunk2D, ChunkId, FxActorId, FxAsset, FxAssetDesc, FxFamily,
    FxFamilyId, GridAabb, GridCoord, NodeRuntimeState, RepairError, RepairPlan, SplitEvent,
    SupportNodeId, ValidationError, Vec2,
};
#[cfg(feature = "parallel")]
use rayon::prelude::*;
use thiserror::Error;

pub mod snapshot;
pub use snapshot::{AuthoredVoxelAssetSnapshot, VoxelSnapshotError};

#[derive(Clone, Debug, PartialEq)]
pub struct VoxelAuthoringInput {
    pub width: u32,
    pub height: u32,
    pub voxel_size: f32,
    pub occupancy: Vec<bool>,
    pub fracture_material: Vec<u16>,
    pub contact_material: Vec<u16>,
    pub external_id: Vec<u32>,
    pub orientation: Option<Vec<u16>>,
    pub support_node_hint: Option<Vec<Option<u32>>>,
    pub default_bond_health: f32,
    pub default_tension_limit: f32,
    pub default_shear_limit: f32,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct VoxelAuthoringOptions {
    pub material_cluster_rules: BTreeMap<u16, VoxelClusterPolicy>,
    pub hierarchy_policy: VoxelHierarchyPolicy,
}

impl VoxelAuthoringOptions {
    pub fn with_material_rule(mut self, material: u16, rule: VoxelClusterPolicy) -> Self {
        self.material_cluster_rules.insert(material, rule);
        self
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VoxelClusterPolicy {
    pub mode: VoxelClusterMode,
}

impl VoxelClusterPolicy {
    pub fn material_components() -> Self {
        Self {
            mode: VoxelClusterMode::MaterialComponents,
        }
    }

    pub fn isotropic(max_extent: u32, max_voxels: usize) -> Self {
        Self {
            mode: VoxelClusterMode::Isotropic {
                max_extent,
                max_voxels,
            },
        }
    }

    pub fn brittle_isotropic(max_extent: u32, max_voxels: usize) -> Self {
        Self::isotropic(max_extent, max_voxels)
    }

    pub fn fiber(along_extent: u32, cross_extent: u32) -> Self {
        Self {
            mode: VoxelClusterMode::Fiber {
                along_extent,
                cross_extent,
            },
        }
    }

    pub fn structural_beam(axis: VoxelClusterAxis, along_extent: u32, cross_extent: u32) -> Self {
        Self {
            mode: VoxelClusterMode::StructuralBeam {
                axis,
                along_extent,
                cross_extent,
            },
        }
    }

    pub fn natural_voronoi(seeds: Vec<GridCoord>) -> Self {
        Self {
            mode: VoxelClusterMode::NaturalVoronoi(NaturalVoronoi::explicit(seeds)),
        }
    }

    pub fn natural_voronoi_generated(seed_count: usize, random_seed: u64) -> Self {
        Self {
            mode: VoxelClusterMode::NaturalVoronoi(NaturalVoronoi::generated(
                seed_count,
                random_seed,
            )),
        }
    }
}

impl Default for VoxelClusterPolicy {
    fn default() -> Self {
        Self::material_components()
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum VoxelHierarchyPolicy {
    #[default]
    Flat,
    ParentChunksByMaterial,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum VoxelClusterMode {
    MaterialComponents,
    Isotropic {
        max_extent: u32,
        max_voxels: usize,
    },
    Fiber {
        along_extent: u32,
        cross_extent: u32,
    },
    /// Phase 2 structural authoring uses material metadata to describe beam or
    /// column direction. Runtime stress still propagates through the bond graph.
    StructuralBeam {
        axis: VoxelClusterAxis,
        along_extent: u32,
        cross_extent: u32,
    },
    NaturalVoronoi(NaturalVoronoi),
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct NaturalVoronoi {
    pub seeds: NaturalVoronoiSeeds,
    pub noise: NaturalVoronoiNoise,
    pub fields: Vec<NaturalVoronoiClusterField>,
}

impl NaturalVoronoi {
    pub const DISTANCE_SCALE_ONE: u32 = 1024;

    pub fn explicit(seeds: Vec<GridCoord>) -> Self {
        Self {
            seeds: NaturalVoronoiSeeds::Explicit(seeds),
            noise: NaturalVoronoiNoise::default(),
            fields: Vec::new(),
        }
    }

    pub fn generated(seed_count: usize, random_seed: u64) -> Self {
        Self {
            seeds: NaturalVoronoiSeeds::Generated {
                seed_count,
                random_seed,
            },
            noise: NaturalVoronoiNoise::default(),
            fields: Vec::new(),
        }
    }

    pub fn with_noise(mut self, seed: u64, amplitude: i64) -> Self {
        self.noise = NaturalVoronoiNoise { seed, amplitude };
        self
    }

    pub fn with_field(mut self, field: NaturalVoronoiClusterField) -> Self {
        self.fields.push(field);
        self
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum NaturalVoronoiSeeds {
    Explicit(Vec<GridCoord>),
    Generated { seed_count: usize, random_seed: u64 },
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct NaturalVoronoiNoise {
    pub seed: u64,
    pub amplitude: i64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NaturalVoronoiClusterField {
    pub center: GridCoord,
    pub radius: u32,
    pub extra_seed_count: usize,
    pub random_seed: u64,
    pub distance_scale: u32,
    pub distance_bias: i64,
}

impl NaturalVoronoiClusterField {
    pub fn new(center: GridCoord, radius: u32) -> Self {
        Self {
            center,
            radius,
            extra_seed_count: 0,
            random_seed: 0,
            distance_scale: NaturalVoronoi::DISTANCE_SCALE_ONE,
            distance_bias: 0,
        }
    }

    pub fn with_extra_seeds(mut self, seed_count: usize, random_seed: u64) -> Self {
        self.extra_seed_count = seed_count;
        self.random_seed = random_seed;
        self
    }

    pub fn with_distance_bias(mut self, distance_scale: u32, distance_bias: i64) -> Self {
        self.distance_scale = distance_scale;
        self.distance_bias = distance_bias;
        self
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum VoxelClusterAxis {
    X,
    Y,
}

impl VoxelAuthoringInput {
    pub fn new(
        width: u32,
        height: u32,
        voxel_size: f32,
        occupancy: Vec<bool>,
        fracture_material: Vec<u16>,
        contact_material: Vec<u16>,
        external_id: Vec<u32>,
    ) -> Self {
        Self {
            width,
            height,
            voxel_size,
            occupancy,
            fracture_material,
            contact_material,
            external_id,
            orientation: None,
            support_node_hint: None,
            default_bond_health: 1.0,
            default_tension_limit: 10.0,
            default_shear_limit: 10.0,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct NodeAuthoringSummary {
    pub node_id: SupportNodeId,
    pub fracture_material: u16,
    pub contact_material_summary: u16,
    pub external_id_min: u32,
    pub external_id_max: u32,
    pub orientation_summary: Option<u16>,
    pub anisotropy_axis: Vec2,
    pub stable_seed: u64,
    pub voxel_count: usize,
}

#[derive(Clone, Debug, PartialEq)]
pub struct BondAuthoringSummary {
    pub bond_id: BondId,
    pub node_a: SupportNodeId,
    pub node_b: SupportNodeId,
    pub fracture_material_pair: (u16, u16),
    pub contact_material_pairs: Vec<(u16, u16)>,
    pub external_id_pairs: Vec<(u32, u32)>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct VoxelMetadata {
    pub coord: GridCoord,
    pub occupied: bool,
    pub node: Option<SupportNodeId>,
    pub fracture_material: u16,
    pub contact_material: u16,
    pub external_id: u32,
    pub orientation: Option<u16>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct VoxelAssetMetrics {
    pub occupied_voxels: usize,
    pub support_nodes: usize,
}

#[derive(Clone, Debug, PartialEq)]
pub struct AuthoredVoxelAsset {
    core: FxAsset,
    width: u32,
    height: u32,
    contact_material: Vec<u16>,
    fracture_material: Vec<u16>,
    external_id: Vec<u32>,
    orientation: Option<Vec<u16>>,
    summaries: Vec<NodeAuthoringSummary>,
    bond_summaries: Vec<BondAuthoringSummary>,
    default_bond_health: f32,
    default_tension_limit: f32,
    default_shear_limit: f32,
}

impl AuthoredVoxelAsset {
    pub fn core(&self) -> &FxAsset {
        &self.core
    }

    pub fn node_summaries(&self) -> &[NodeAuthoringSummary] {
        &self.summaries
    }

    pub fn bond_summaries(&self) -> &[BondAuthoringSummary] {
        &self.bond_summaries
    }

    pub fn fracture_material_map(&self) -> &[u16] {
        &self.fracture_material
    }

    pub fn contact_material_map(&self) -> &[u16] {
        &self.contact_material
    }

    pub fn external_id_map(&self) -> &[u32] {
        &self.external_id
    }

    pub fn orientation_map(&self) -> Option<&[u16]> {
        self.orientation.as_deref()
    }

    pub fn metrics(&self) -> VoxelAssetMetrics {
        VoxelAssetMetrics {
            occupied_voxels: self
                .core
                .occupancy()
                .cells()
                .iter()
                .filter(|cell| **cell)
                .count(),
            support_nodes: self.core.support_nodes().len(),
        }
    }

    pub fn voxel_metadata(&self, coord: GridCoord) -> Result<VoxelMetadata, VoxelError> {
        if coord.x >= self.width || coord.y >= self.height {
            return Err(VoxelError::CoordinateOutOfBounds(coord));
        }
        let idx = index(self.width, coord);
        Ok(VoxelMetadata {
            coord,
            occupied: self.core.occupancy().cells()[idx],
            node: self.core.node_at(coord),
            fracture_material: self.fracture_material[idx],
            contact_material: self.contact_material[idx],
            external_id: self.external_id[idx],
            orientation: self.orientation.as_ref().map(|map| map[idx]),
        })
    }

    pub fn validate_exact_cover(&self) -> Result<(), VoxelError> {
        validate_exact_cover(
            self.width,
            self.height,
            &self.occupancy(),
            self.core.voxel_to_node_map(),
        )
    }

    pub fn occupancy(&self) -> Vec<bool> {
        self.core.occupancy().cells().to_vec()
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct VoxelAdd {
    pub coord: GridCoord,
    pub fracture_material: u16,
    pub contact_material: u16,
    pub external_id: u32,
    pub orientation: Option<u16>,
}

#[derive(Clone, Debug, PartialEq)]
pub enum RuntimeEdit {
    RemoveVoxels {
        voxels: Vec<GridCoord>,
    },
    AddVoxels {
        actor: FxActorId,
        voxels: Vec<VoxelAdd>,
    },
    SetMaterial {
        voxels: Vec<GridCoord>,
        fracture_material: u16,
    },
}

#[derive(Clone, Debug, PartialEq)]
pub struct RepairReport {
    pub dirty_bbox: Option<GridAabb>,
    pub affected_old_nodes: Vec<SupportNodeId>,
    pub unchanged_nodes: Vec<SupportNodeId>,
    pub unchanged_bonds: Vec<UnchangedBondProof>,
    pub unchanged_actors: Vec<FxActorId>,
    pub reused_nodes: Vec<SupportNodeId>,
    pub new_nodes: Vec<SupportNodeId>,
    pub dirty_actors: Vec<FxActorId>,
    pub preserved_dirty_actors: Vec<FxActorId>,
    pub exact_cover_validated: bool,
    pub unaffected_region_preserved: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct UnchangedBondProof {
    pub old_bond: BondId,
    pub new_bond: BondId,
}

#[derive(Clone, Debug, PartialEq)]
pub struct VoxelRuntime {
    asset: AuthoredVoxelAsset,
    family: FxFamily,
    voxel_owner: Vec<Option<FxActorId>>,
    node_lineage: BTreeMap<SupportNodeId, SupportNodeId>,
    next_node_id: u32,
    last_repair: Option<RepairReport>,
}

impl VoxelRuntime {
    pub fn instantiate(family_id: FxFamilyId, asset: AuthoredVoxelAsset) -> Self {
        let family = FxFamily::instantiate(family_id, asset.core.clone());
        let mut voxel_owner = vec![None; cell_count(asset.width, asset.height)];
        for coord in all_coords(asset.width, asset.height) {
            let idx = index(asset.width, coord);
            if let Some(node) = asset.core.node_at(coord) {
                voxel_owner[idx] = family.node_owner(node);
            }
        }
        let node_lineage = asset
            .core
            .support_nodes()
            .iter()
            .map(|node| (node.id, node.id))
            .collect::<BTreeMap<_, _>>();
        let next_node_id = asset
            .core
            .support_nodes()
            .iter()
            .map(|node| node.id.0 + 1)
            .max()
            .unwrap_or(0);
        Self {
            asset,
            family,
            voxel_owner,
            node_lineage,
            next_node_id,
            last_repair: None,
        }
    }

    pub fn family(&self) -> &FxFamily {
        &self.family
    }

    pub fn asset(&self) -> &AuthoredVoxelAsset {
        &self.asset
    }

    pub fn last_repair(&self) -> Option<&RepairReport> {
        self.last_repair.as_ref()
    }

    pub fn apply_fracture_commands(
        &mut self,
        commands: &[fracture_core::FractureCommand],
    ) -> Vec<fracture_core::FractureEvent> {
        fracture_core::apply_fracture_commands(&mut self.family, commands)
    }

    pub fn split_dirty_actors(&mut self) -> Vec<SplitEvent> {
        fracture_core::split_dirty_actors(&mut self.family)
    }

    pub fn apply_edit(&mut self, edit: RuntimeEdit) -> Result<RepairReport, VoxelError> {
        let old_asset = self.asset.clone();
        let old_family = self.family.clone();
        let old_voxel_owner = self.voxel_owner.clone();
        let old_node_lineage = self.node_lineage.clone();
        let old_dirty_actors = old_family.dirty_actors().collect::<BTreeSet<_>>();

        let mut occupancy = old_asset.occupancy();
        let mut fracture_material = old_asset.fracture_material.clone();
        let mut contact_material = old_asset.contact_material.clone();
        let mut external_id = old_asset.external_id.clone();
        let mut orientation = old_asset.orientation.clone();
        let mut voxel_owner = self.voxel_owner.clone();
        let mut edited = Vec::new();
        let mut dirty_actors = BTreeSet::new();

        match edit {
            RuntimeEdit::RemoveVoxels { voxels } => {
                for coord in voxels {
                    self.require_in_bounds(coord)?;
                    let idx = index(self.asset.width, coord);
                    if occupancy[idx] {
                        if let Some(actor) = voxel_owner[idx] {
                            dirty_actors.insert(actor);
                        }
                        occupancy[idx] = false;
                        voxel_owner[idx] = None;
                        edited.push(coord);
                    }
                }
            }
            RuntimeEdit::AddVoxels { actor, voxels } => {
                if self.family.actor(actor).is_none() {
                    return Err(VoxelError::UnknownActor(actor));
                }
                for add in voxels {
                    self.require_in_bounds(add.coord)?;
                    let idx = index(self.asset.width, add.coord);
                    if occupancy[idx] {
                        return Err(VoxelError::OccupiedVoxel(add.coord));
                    }
                    occupancy[idx] = true;
                    fracture_material[idx] = add.fracture_material;
                    contact_material[idx] = add.contact_material;
                    external_id[idx] = add.external_id;
                    if add.orientation.is_some() && orientation.is_none() {
                        orientation = Some(vec![0; occupancy.len()]);
                    }
                    if let (Some(map), Some(angle)) = (&mut orientation, add.orientation) {
                        map[idx] = angle;
                    }
                    voxel_owner[idx] = Some(actor);
                    dirty_actors.insert(actor);
                    edited.push(add.coord);
                }
            }
            RuntimeEdit::SetMaterial {
                voxels,
                fracture_material: new_material,
            } => {
                for coord in voxels {
                    self.require_in_bounds(coord)?;
                    let idx = index(self.asset.width, coord);
                    if occupancy[idx] {
                        fracture_material[idx] = new_material;
                        if let Some(actor) = voxel_owner[idx] {
                            dirty_actors.insert(actor);
                        }
                        edited.push(coord);
                    }
                }
            }
        }

        let dirty_bbox = bbox_for_edits(self.asset.width, self.asset.height, &edited);
        let Some(dirty_bbox) = dirty_bbox else {
            return Ok(RepairReport {
                dirty_bbox: None,
                affected_old_nodes: Vec::new(),
                unchanged_nodes: self
                    .asset
                    .core()
                    .support_nodes()
                    .iter()
                    .map(|node| node.id)
                    .collect(),
                unchanged_bonds: self
                    .asset
                    .core()
                    .internal_bonds()
                    .iter()
                    .map(|bond| UnchangedBondProof {
                        old_bond: bond.id,
                        new_bond: bond.id,
                    })
                    .collect(),
                unchanged_actors: self.family.actors().map(|(actor, _)| *actor).collect(),
                reused_nodes: Vec::new(),
                new_nodes: Vec::new(),
                dirty_actors: old_dirty_actors.iter().copied().collect(),
                preserved_dirty_actors: old_dirty_actors.iter().copied().collect(),
                exact_cover_validated: true,
                unaffected_region_preserved: true,
            });
        };

        let affected_old_nodes = affected_nodes(&old_asset.core, dirty_bbox);
        for node in &affected_old_nodes {
            if let Some(actor) = old_family.node_owner(*node) {
                dirty_actors.insert(actor);
            }
        }

        let repair = build_repaired_topology(RepairBuildInput {
            width: self.asset.width,
            height: self.asset.height,
            voxel_size: self.asset.core.voxel_size(),
            occupancy: &occupancy,
            fracture_material: &fracture_material,
            contact_material: &contact_material,
            external_id: &external_id,
            orientation: orientation.as_ref(),
            voxel_owner: &voxel_owner,
            old_asset: &old_asset.core,
            old_family: &old_family,
            old_voxel_owner: &old_voxel_owner,
            old_node_lineage: &old_node_lineage,
            affected_old_nodes: &affected_old_nodes,
            next_node_id: &mut self.next_node_id,
            default_bond_health: old_asset.default_bond_health,
            default_tension_limit: old_asset.default_tension_limit,
            default_shear_limit: old_asset.default_shear_limit,
        })?;

        let post_actors = repair
            .node_owners
            .iter()
            .map(|(_, actor)| *actor)
            .collect::<BTreeSet<_>>();
        for actor in &old_dirty_actors {
            if post_actors.contains(actor) {
                dirty_actors.insert(*actor);
            }
        }
        let preserved_dirty_actors = old_dirty_actors
            .iter()
            .filter(|actor| dirty_actors.contains(actor))
            .copied()
            .collect::<Vec<_>>();

        let plan = RepairPlan {
            asset: repair.asset.core.clone(),
            node_owners: repair.node_owners,
            node_states: repair.node_states,
            bond_states: repair.bond_states,
            dirty_actors: dirty_actors.iter().copied().collect(),
        };
        let summary = self.family.apply_repair_plan(plan)?;

        self.asset = repair.asset;
        self.voxel_owner = voxel_owner;
        self.node_lineage = repair.node_lineage;
        let report = RepairReport {
            dirty_bbox: Some(dirty_bbox),
            affected_old_nodes,
            unchanged_nodes: repair.local_proof.unchanged_nodes,
            unchanged_bonds: repair.local_proof.unchanged_bonds,
            unchanged_actors: repair.local_proof.unchanged_actors,
            reused_nodes: repair.reused_nodes,
            new_nodes: repair.new_nodes,
            dirty_actors: summary.dirty_actors,
            preserved_dirty_actors,
            exact_cover_validated: true,
            unaffected_region_preserved: repair.local_proof.unaffected_region_preserved,
        };
        self.last_repair = Some(report.clone());
        Ok(report)
    }

    fn require_in_bounds(&self, coord: GridCoord) -> Result<(), VoxelError> {
        if coord.x < self.asset.width && coord.y < self.asset.height {
            Ok(())
        } else {
            Err(VoxelError::CoordinateOutOfBounds(coord))
        }
    }
}

pub fn author_voxel_asset(input: VoxelAuthoringInput) -> Result<AuthoredVoxelAsset, VoxelError> {
    author_voxel_asset_with_options(input, VoxelAuthoringOptions::default())
}

pub fn author_voxel_asset_with_options(
    input: VoxelAuthoringInput,
    options: VoxelAuthoringOptions,
) -> Result<AuthoredVoxelAsset, VoxelError> {
    validate_input_maps(&input)?;
    let support_node_map = build_authoring_support_node_map(&input, &options);
    let authored_chunks = build_authored_chunks(
        input.width,
        &input.fracture_material,
        &support_node_map,
        &options,
    );
    let core = make_core_asset(
        input.width,
        input.height,
        input.voxel_size,
        input.occupancy.clone(),
        input.fracture_material.clone(),
        input.orientation.clone(),
        support_node_map,
        authored_chunks,
        input.default_bond_health,
        input.default_tension_limit,
        input.default_shear_limit,
    )?;
    let asset = AuthoredVoxelAsset {
        summaries: node_summaries(
            &core,
            &input.contact_material,
            &input.external_id,
            input.width,
        ),
        bond_summaries: bond_summaries(
            &core,
            &input.contact_material,
            &input.external_id,
            input.width,
            input.height,
        ),
        core,
        width: input.width,
        height: input.height,
        contact_material: input.contact_material,
        fracture_material: input.fracture_material,
        external_id: input.external_id,
        orientation: input.orientation,
        default_bond_health: input.default_bond_health,
        default_tension_limit: input.default_tension_limit,
        default_shear_limit: input.default_shear_limit,
    };
    asset.validate_exact_cover()?;
    Ok(asset)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct AuthorClusterKey {
    material: u16,
    bin: ClusterBin,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum ClusterBin {
    MaterialComponent,
    Tile {
        axis: VoxelClusterAxis,
        direction: u8,
        x: u32,
        y: u32,
    },
    NaturalVoronoi {
        seed: u32,
    },
}

#[derive(Clone, Debug)]
struct KeyedComponent<K> {
    key: K,
    voxels: Vec<GridCoord>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct RuntimeKey {
    material: u16,
    owner: FxActorId,
    old_node: Option<SupportNodeId>,
}

#[derive(Debug)]
struct RepairBuildInput<'a> {
    width: u32,
    height: u32,
    voxel_size: f32,
    occupancy: &'a [bool],
    fracture_material: &'a [u16],
    contact_material: &'a [u16],
    external_id: &'a [u32],
    orientation: Option<&'a Vec<u16>>,
    voxel_owner: &'a [Option<FxActorId>],
    old_asset: &'a FxAsset,
    old_family: &'a FxFamily,
    old_voxel_owner: &'a [Option<FxActorId>],
    old_node_lineage: &'a BTreeMap<SupportNodeId, SupportNodeId>,
    affected_old_nodes: &'a [SupportNodeId],
    next_node_id: &'a mut u32,
    default_bond_health: f32,
    default_tension_limit: f32,
    default_shear_limit: f32,
}

#[derive(Clone, Debug)]
struct RepairBuildOutput {
    asset: AuthoredVoxelAsset,
    node_owners: Vec<(SupportNodeId, FxActorId)>,
    node_states: Vec<(SupportNodeId, NodeRuntimeState)>,
    bond_states: Vec<BondRuntimeState>,
    node_lineage: BTreeMap<SupportNodeId, SupportNodeId>,
    local_proof: LocalRepairProof,
    reused_nodes: Vec<SupportNodeId>,
    new_nodes: Vec<SupportNodeId>,
}

#[derive(Clone, Debug)]
struct LocalRepairProof {
    unaffected_region_preserved: bool,
    unchanged_nodes: Vec<SupportNodeId>,
    unchanged_bonds: Vec<UnchangedBondProof>,
    unchanged_actors: Vec<FxActorId>,
}

fn build_repaired_topology(input: RepairBuildInput<'_>) -> Result<RepairBuildOutput, VoxelError> {
    let affected: BTreeSet<_> = input.affected_old_nodes.iter().copied().collect();
    let mut support_node_map = vec![None; cell_count(input.width, input.height)];
    let mut node_owners = Vec::new();
    let mut node_states = Vec::new();
    let mut node_lineage = BTreeMap::new();
    let mut reused_nodes = Vec::new();
    let mut new_nodes = Vec::new();
    let mut used_old_ids = BTreeSet::new();
    for old_node in input.old_asset.support_nodes() {
        if affected.contains(&old_node.id) {
            continue;
        }
        let mut has_surviving_voxels = false;
        for &coord in &old_node.voxels {
            let idx = index(input.width, coord);
            if input.occupancy[idx] {
                support_node_map[idx] = Some(old_node.id);
                has_surviving_voxels = true;
            }
        }
        if has_surviving_voxels {
            let Some(actor) = input.old_family.node_owner(old_node.id) else {
                return Err(VoxelError::MissingOldNodeOwner(old_node.id));
            };
            node_owners.push((old_node.id, actor));
            node_states.push((
                old_node.id,
                input
                    .old_family
                    .node_state(old_node.id)
                    .cloned()
                    .unwrap_or_else(initial_node_state),
            ));
            node_lineage.insert(
                old_node.id,
                input
                    .old_node_lineage
                    .get(&old_node.id)
                    .copied()
                    .unwrap_or(old_node.id),
            );
            reused_nodes.push(old_node.id);
            used_old_ids.insert(old_node.id);
        }
    }

    let components = connected_components_by_key(input.width, input.height, |coord| {
        let idx = index(input.width, coord);
        if !input.occupancy[idx] {
            return None;
        }
        let old_node = input.old_asset.node_at(coord);
        if old_node.is_some_and(|node| !affected.contains(&node)) {
            return None;
        }
        let owner = input.voxel_owner[idx]?;
        Some((
            RuntimeKey {
                material: input.fracture_material[idx],
                owner,
                old_node,
            },
            old_node,
        ))
    });
    let state_sources = node_state_sources(&components, &input);
    let node_id_reusers = choose_node_id_reusers(&components, &input);
    let surviving_overlap = old_node_surviving_overlap(&components, &input);

    for (component_idx, component) in components.iter().enumerate() {
        let mut overlap: BTreeMap<SupportNodeId, usize> = BTreeMap::new();
        for &coord in &component.voxels {
            if let Some(old_node) = input.old_asset.node_at(coord) {
                *overlap.entry(old_node).or_default() += 1;
            }
        }
        let dominant_old = overlap
            .iter()
            .max_by(|(node_a, count_a), (node_b, count_b)| {
                count_a.cmp(count_b).then_with(|| node_b.cmp(node_a))
            })
            .map(|(node, _)| *node);
        let state_source_old = state_sources.get(&component_idx).copied();
        let reusable_old = node_id_reusers.get(&component_idx).copied();
        let node_id = reusable_old
            .filter(|node| !used_old_ids.contains(node))
            .unwrap_or_else(|| {
                let id = SupportNodeId(*input.next_node_id);
                *input.next_node_id += 1;
                id
            });
        if reusable_old == Some(node_id) {
            reused_nodes.push(node_id);
        } else {
            new_nodes.push(node_id);
        }
        used_old_ids.insert(node_id);
        for &coord in &component.voxels {
            support_node_map[index(input.width, coord)] = Some(node_id);
        }
        node_owners.push((node_id, component.key.owner));
        let lineage = dominant_old
            .and_then(|old| input.old_node_lineage.get(&old).copied())
            .or(dominant_old)
            .unwrap_or(node_id);
        node_lineage.insert(node_id, lineage);
        let state = state_source_old
            .and_then(|old| {
                let old_state = input.old_family.node_state(old)?;
                let overlap_count = overlap.get(&old).copied().unwrap_or(0);
                let total_surviving = surviving_overlap.get(&old).copied().unwrap_or(0);
                Some(transfer_node_state(
                    old_state,
                    overlap_count,
                    total_surviving,
                ))
            })
            .unwrap_or_else(initial_node_state);
        node_states.push((node_id, state));
    }

    node_owners.sort_by_key(|(node, _)| *node);
    node_states.sort_by_key(|(node, _)| *node);
    reused_nodes.sort_unstable();
    reused_nodes.dedup();
    new_nodes.sort_unstable();
    new_nodes.dedup();

    let core = make_core_asset(
        input.width,
        input.height,
        input.voxel_size,
        input.occupancy.to_vec(),
        input.fracture_material.to_vec(),
        input.orientation.cloned(),
        support_node_map,
        None,
        input.default_bond_health,
        input.default_tension_limit,
        input.default_shear_limit,
    )?;
    let bond_states = transfer_bond_states(
        input.old_asset,
        input.old_family,
        &core,
        input.old_node_lineage,
        &node_lineage,
    );
    let asset = AuthoredVoxelAsset {
        summaries: node_summaries(
            &core,
            input.contact_material,
            input.external_id,
            input.width,
        ),
        bond_summaries: bond_summaries(
            &core,
            input.contact_material,
            input.external_id,
            input.width,
            input.height,
        ),
        core,
        width: input.width,
        height: input.height,
        contact_material: input.contact_material.to_vec(),
        fracture_material: input.fracture_material.to_vec(),
        external_id: input.external_id.to_vec(),
        orientation: input.orientation.cloned(),
        default_bond_health: input.default_bond_health,
        default_tension_limit: input.default_tension_limit,
        default_shear_limit: input.default_shear_limit,
    };
    asset.validate_exact_cover()?;
    let local_proof = verify_local_repair_invariants(
        input.old_asset,
        input.old_family,
        &asset.core,
        &node_owners,
        &node_states,
        &bond_states,
        &affected,
    )?;
    let _ = input.old_voxel_owner;
    Ok(RepairBuildOutput {
        asset,
        node_owners,
        node_states,
        bond_states,
        node_lineage,
        local_proof,
        reused_nodes,
        new_nodes,
    })
}

fn node_state_sources(
    components: &[KeyedComponent<RuntimeKey>],
    input: &RepairBuildInput<'_>,
) -> BTreeMap<usize, SupportNodeId> {
    components
        .iter()
        .enumerate()
        .filter_map(|(component_idx, component)| {
            let old_node = component.key.old_node?;
            let old_owner = input.old_family.node_owner(old_node)?;
            (old_owner == component.key.owner).then_some((component_idx, old_node))
        })
        .collect()
}

fn choose_node_id_reusers(
    components: &[KeyedComponent<RuntimeKey>],
    input: &RepairBuildInput<'_>,
) -> BTreeMap<usize, SupportNodeId> {
    let mut best_by_old = BTreeMap::<SupportNodeId, (usize, usize, GridCoord)>::new();
    for (component_idx, component) in components.iter().enumerate() {
        let first_coord = component
            .voxels
            .first()
            .copied()
            .unwrap_or(GridCoord::new(u32::MAX, u32::MAX));
        let mut overlap = BTreeMap::<SupportNodeId, usize>::new();
        for &coord in &component.voxels {
            if let Some(old_node) = input.old_asset.node_at(coord) {
                *overlap.entry(old_node).or_default() += 1;
            }
        }
        for (old_node, count) in overlap {
            let old_owner = input.old_family.node_owner(old_node);
            if old_owner != Some(component.key.owner) {
                continue;
            }
            let candidate = (count, component_idx, first_coord);
            let replace =
                best_by_old
                    .get(&old_node)
                    .is_none_or(|&(best_count, best_idx, best_first)| {
                        count > best_count
                            || (count == best_count && first_coord < best_first)
                            || (count == best_count
                                && first_coord == best_first
                                && component_idx < best_idx)
                    });
            if replace {
                best_by_old.insert(old_node, candidate);
            }
        }
    }

    best_by_old
        .into_iter()
        .map(|(old_node, (_, component_idx, _))| (component_idx, old_node))
        .collect()
}

fn old_node_surviving_overlap(
    components: &[KeyedComponent<RuntimeKey>],
    input: &RepairBuildInput<'_>,
) -> BTreeMap<SupportNodeId, usize> {
    let mut totals = BTreeMap::<SupportNodeId, usize>::new();
    for component in components {
        let Some(old_node) = component.key.old_node else {
            continue;
        };
        if input.old_family.node_owner(old_node) != Some(component.key.owner) {
            continue;
        }
        *totals.entry(old_node).or_default() += component.voxels.len();
    }
    totals
}

fn transfer_node_state(
    old_state: &NodeRuntimeState,
    overlap_count: usize,
    total_surviving_overlap: usize,
) -> NodeRuntimeState {
    let ratio = if total_surviving_overlap > 0 {
        overlap_count as f32 / total_surviving_overlap as f32
    } else {
        0.0
    };
    NodeRuntimeState {
        health: old_state.health * ratio,
        accumulated_damage: old_state.accumulated_damage * ratio,
    }
}

fn initial_node_state() -> NodeRuntimeState {
    NodeRuntimeState {
        health: 1.0,
        accumulated_damage: 0.0,
    }
}

fn verify_local_repair_invariants(
    old_asset: &FxAsset,
    old_family: &FxFamily,
    new_asset: &FxAsset,
    node_owners: &[(SupportNodeId, FxActorId)],
    node_states: &[(SupportNodeId, NodeRuntimeState)],
    bond_states: &[BondRuntimeState],
    affected: &BTreeSet<SupportNodeId>,
) -> Result<LocalRepairProof, VoxelError> {
    let owner_map = node_owners.iter().copied().collect::<BTreeMap<_, _>>();
    let state_map = node_states
        .iter()
        .map(|(node, state)| (*node, state.clone()))
        .collect::<BTreeMap<_, _>>();
    let mut unchanged_nodes = Vec::new();
    for old_node in old_asset.support_nodes() {
        if affected.contains(&old_node.id) {
            continue;
        }
        let Some(new_node) = new_asset.node(old_node.id) else {
            return Err(VoxelError::LocalRepairInvariantViolated(
                "unaffected node disappeared",
            ));
        };
        if new_node != old_node {
            return Err(VoxelError::LocalRepairInvariantViolated(
                "unaffected node metadata changed",
            ));
        }
        let old_owner = old_family
            .node_owner(old_node.id)
            .ok_or(VoxelError::MissingOldNodeOwner(old_node.id))?;
        if owner_map.get(&old_node.id) != Some(&old_owner) {
            return Err(VoxelError::LocalRepairInvariantViolated(
                "unaffected node owner changed",
            ));
        }
        let Some(old_state) = old_family.node_state(old_node.id) else {
            return Err(VoxelError::LocalRepairInvariantViolated(
                "unaffected node old state missing",
            ));
        };
        if state_map.get(&old_node.id) != Some(old_state) {
            return Err(VoxelError::LocalRepairInvariantViolated(
                "unaffected node runtime state changed",
            ));
        }
        unchanged_nodes.push(old_node.id);
    }

    let mut new_nodes_by_actor = BTreeMap::<FxActorId, Vec<SupportNodeId>>::new();
    for (node, actor) in node_owners {
        new_nodes_by_actor.entry(*actor).or_default().push(*node);
    }
    let mut unchanged_actors = Vec::new();
    for (actor_id, old_actor) in old_family.actors() {
        let mut new_nodes = new_nodes_by_actor.remove(actor_id).unwrap_or_default();
        new_nodes.sort_unstable();
        if old_actor.owned_nodes == new_nodes {
            unchanged_actors.push(*actor_id);
        }
    }

    let mut unchanged_bonds = Vec::new();
    for old_bond in old_asset.internal_bonds() {
        if affected.contains(&old_bond.node_a) || affected.contains(&old_bond.node_b) {
            continue;
        }
        let new_bond = new_asset
            .internal_bonds()
            .iter()
            .find(|candidate| {
                candidate.node_a == old_bond.node_a
                    && candidate.node_b == old_bond.node_b
                    && candidate.interface_edges == old_bond.interface_edges
            })
            .ok_or(VoxelError::LocalRepairInvariantViolated(
                "unaffected bond disappeared",
            ))?;
        let old_state =
            old_family
                .bond_state(old_bond.id)
                .ok_or(VoxelError::LocalRepairInvariantViolated(
                    "unaffected bond old state missing",
                ))?;
        if bond_states.get(new_bond.id.0 as usize) != Some(old_state) {
            return Err(VoxelError::LocalRepairInvariantViolated(
                "unaffected bond runtime state changed",
            ));
        }
        unchanged_bonds.push(UnchangedBondProof {
            old_bond: old_bond.id,
            new_bond: new_bond.id,
        });
    }

    Ok(LocalRepairProof {
        unaffected_region_preserved: true,
        unchanged_nodes,
        unchanged_bonds,
        unchanged_actors,
    })
}

fn transfer_bond_states(
    old_asset: &FxAsset,
    old_family: &FxFamily,
    new_asset: &FxAsset,
    old_node_lineage: &BTreeMap<SupportNodeId, SupportNodeId>,
    new_node_lineage: &BTreeMap<SupportNodeId, SupportNodeId>,
) -> Vec<BondRuntimeState> {
    new_asset
        .internal_bonds()
        .iter()
        .map(|new_bond| {
            let new_pair = lineage_pair(new_bond.node_a, new_bond.node_b, new_node_lineage);
            let new_edges: BTreeSet<_> = new_bond.interface_edges.iter().copied().collect();
            let mut best: Option<(usize, BondId)> = None;
            for old_bond in old_asset.internal_bonds() {
                if lineage_pair(old_bond.node_a, old_bond.node_b, old_node_lineage) != new_pair {
                    continue;
                }
                let overlap = old_bond
                    .interface_edges
                    .iter()
                    .filter(|edge| new_edges.contains(edge))
                    .count();
                best = better_bond_lineage_candidate(best, (overlap, old_bond.id));
            }
            if let Some((overlap, old_bond_id)) = best {
                let old_bond = old_asset.bond(old_bond_id).expect("old bond id is valid");
                let ratio = overlap as f32 / old_bond.interface_edges.len() as f32;
                let old_state =
                    old_family
                        .bond_state(old_bond_id)
                        .cloned()
                        .unwrap_or(BondRuntimeState {
                            health: old_bond.base_health,
                            effective_length: old_bond.length,
                            accumulated_damage: 0.0,
                        });
                transfer_bond_state(&old_state, old_bond, new_bond, ratio)
            } else {
                BondRuntimeState {
                    health: new_bond.base_health,
                    effective_length: new_bond.length,
                    accumulated_damage: 0.0,
                }
            }
        })
        .collect()
}

fn transfer_bond_state(
    old_state: &BondRuntimeState,
    old_bond: &fracture_core::Bond2D,
    new_bond: &fracture_core::Bond2D,
    interface_overlap_ratio: f32,
) -> BondRuntimeState {
    let health_ratio = if old_bond.base_health > 0.0 {
        (old_state.health / old_bond.base_health).clamp(0.0, 1.0)
    } else {
        0.0
    };
    BondRuntimeState {
        health: new_bond.base_health * health_ratio,
        effective_length: (old_state.effective_length * interface_overlap_ratio)
            .min(new_bond.length),
        accumulated_damage: old_state.accumulated_damage * interface_overlap_ratio,
    }
}

fn better_bond_lineage_candidate(
    best: Option<(usize, BondId)>,
    candidate: (usize, BondId),
) -> Option<(usize, BondId)> {
    let (overlap, old_bond_id) = candidate;
    if overlap == 0 {
        return best;
    }
    if best.is_none_or(|(best_count, best_id)| {
        overlap > best_count || (overlap == best_count && old_bond_id < best_id)
    }) {
        Some(candidate)
    } else {
        best
    }
}

fn lineage_pair(
    a: SupportNodeId,
    b: SupportNodeId,
    lineage: &BTreeMap<SupportNodeId, SupportNodeId>,
) -> (SupportNodeId, SupportNodeId) {
    let la = lineage.get(&a).copied().unwrap_or(a);
    let lb = lineage.get(&b).copied().unwrap_or(b);
    if la <= lb { (la, lb) } else { (lb, la) }
}

fn build_authoring_support_node_map(
    input: &VoxelAuthoringInput,
    options: &VoxelAuthoringOptions,
) -> Vec<Option<SupportNodeId>> {
    if input.support_node_hint.is_some() {
        return build_authoring_support_node_map_with_natural_voronoi_bins(input, options, None);
    }

    let natural_voronoi_bins = build_natural_voronoi_bins(input, options);
    build_authoring_support_node_map_with_natural_voronoi_bins(
        input,
        options,
        natural_voronoi_bins.as_deref(),
    )
}

fn build_authoring_support_node_map_with_natural_voronoi_bins(
    input: &VoxelAuthoringInput,
    options: &VoxelAuthoringOptions,
    natural_voronoi_bins: Option<&[Option<u32>]>,
) -> Vec<Option<SupportNodeId>> {
    let mut support_node_map = vec![None; cell_count(input.width, input.height)];
    if let Some(hints) = &input.support_node_hint {
        let components = connected_components_by_key(input.width, input.height, |coord| {
            let idx = index(input.width, coord);
            input.occupancy[idx].then_some(((hints[idx]?, input.fracture_material[idx]), None))
        });
        let mut used_ids = BTreeSet::new();
        let mut next_id = hints
            .iter()
            .filter_map(|hint| hint.map(|id| id + 1))
            .max()
            .unwrap_or(0);
        for component in components {
            let hint_id = SupportNodeId(component.key.0);
            let node = if used_ids.insert(hint_id) {
                hint_id
            } else {
                while used_ids.contains(&SupportNodeId(next_id)) {
                    next_id += 1;
                }
                let node = SupportNodeId(next_id);
                used_ids.insert(node);
                next_id += 1;
                node
            };
            for coord in component.voxels {
                support_node_map[index(input.width, coord)] = Some(node);
            }
        }
    } else {
        let components = connected_components_by_key(input.width, input.height, |coord| {
            let idx = index(input.width, coord);
            if !input.occupancy[idx] {
                return None;
            }
            Some((
                AuthorClusterKey {
                    material: input.fracture_material[idx],
                    bin: cluster_bin_for_voxel(input, coord, options, natural_voronoi_bins),
                },
                None,
            ))
        });
        for (node_idx, component) in components.iter().enumerate() {
            let node = SupportNodeId(node_idx as u32);
            for &coord in &component.voxels {
                support_node_map[index(input.width, coord)] = Some(node);
            }
        }
    }
    support_node_map
}

#[derive(Clone, Copy, Debug)]
struct NaturalVoronoiRuntimeSeed {
    position: GridCoord,
}

fn build_natural_voronoi_bins(
    input: &VoxelAuthoringInput,
    options: &VoxelAuthoringOptions,
) -> Option<Vec<Option<u32>>> {
    let has_natural_rule = options
        .material_cluster_rules
        .values()
        .any(|rule| matches!(rule.mode, VoxelClusterMode::NaturalVoronoi(_)));
    if !has_natural_rule {
        return None;
    }

    let mut bins = vec![None; cell_count(input.width, input.height)];
    for (material, rule) in &options.material_cluster_rules {
        let VoxelClusterMode::NaturalVoronoi(natural) = &rule.mode else {
            continue;
        };
        let seeds = natural_voronoi_seeds(input, *material, natural);
        if seeds.is_empty() {
            continue;
        }
        assign_natural_voronoi_bins_for_material(input, *material, natural, &seeds, &mut bins);
    }

    Some(bins)
}

fn assign_natural_voronoi_bins_for_material(
    input: &VoxelAuthoringInput,
    material: u16,
    natural: &NaturalVoronoi,
    seeds: &[NaturalVoronoiRuntimeSeed],
    bins: &mut [Option<u32>],
) {
    #[cfg(feature = "parallel")]
    {
        assign_natural_voronoi_bins_for_material_parallel(input, material, natural, seeds, bins);
    }
    #[cfg(not(feature = "parallel"))]
    {
        assign_natural_voronoi_bins_for_material_serial(input, material, natural, seeds, bins);
    }
}

#[cfg(any(not(feature = "parallel"), test))]
fn assign_natural_voronoi_bins_for_material_serial(
    input: &VoxelAuthoringInput,
    material: u16,
    natural: &NaturalVoronoi,
    seeds: &[NaturalVoronoiRuntimeSeed],
    bins: &mut [Option<u32>],
) {
    for coord in all_coords(input.width, input.height) {
        let idx = index(input.width, coord);
        if input.occupancy[idx] && input.fracture_material[idx] == material {
            bins[idx] = Some(assign_natural_voronoi_seed(coord, seeds, natural));
        }
    }
}

#[cfg(feature = "parallel")]
fn assign_natural_voronoi_bins_for_material_parallel(
    input: &VoxelAuthoringInput,
    material: u16,
    natural: &NaturalVoronoi,
    seeds: &[NaturalVoronoiRuntimeSeed],
    bins: &mut [Option<u32>],
) {
    let width = input.width as usize;
    bins.par_iter_mut().enumerate().for_each(|(idx, bin)| {
        if input.occupancy[idx] && input.fracture_material[idx] == material {
            let coord = GridCoord::new((idx % width) as u32, (idx / width) as u32);
            *bin = Some(assign_natural_voronoi_seed(coord, seeds, natural));
        }
    });
}

#[cfg(all(test, feature = "parallel"))]
fn build_natural_voronoi_bins_serial(
    input: &VoxelAuthoringInput,
    options: &VoxelAuthoringOptions,
) -> Option<Vec<Option<u32>>> {
    let has_natural_rule = options
        .material_cluster_rules
        .values()
        .any(|rule| matches!(rule.mode, VoxelClusterMode::NaturalVoronoi(_)));
    if !has_natural_rule {
        return None;
    }

    let mut bins = vec![None; cell_count(input.width, input.height)];
    for (material, rule) in &options.material_cluster_rules {
        let VoxelClusterMode::NaturalVoronoi(natural) = &rule.mode else {
            continue;
        };
        let seeds = natural_voronoi_seeds(input, *material, natural);
        if seeds.is_empty() {
            continue;
        }
        assign_natural_voronoi_bins_for_material_serial(
            input, *material, natural, &seeds, &mut bins,
        );
    }

    Some(bins)
}

fn natural_voronoi_seeds(
    input: &VoxelAuthoringInput,
    material: u16,
    natural: &NaturalVoronoi,
) -> Vec<NaturalVoronoiRuntimeSeed> {
    let occupied = all_coords(input.width, input.height)
        .filter(|&coord| {
            let idx = index(input.width, coord);
            input.occupancy[idx] && input.fracture_material[idx] == material
        })
        .collect::<Vec<_>>();
    if occupied.is_empty() {
        return Vec::new();
    }

    let occupied_set = occupied.iter().copied().collect::<BTreeSet<_>>();
    let mut used = BTreeSet::new();
    let mut seeds = Vec::new();
    match &natural.seeds {
        NaturalVoronoiSeeds::Explicit(explicit) => {
            for &position in explicit {
                if occupied_set.contains(&position) && used.insert(position) {
                    seeds.push(NaturalVoronoiRuntimeSeed { position });
                }
            }
        }
        NaturalVoronoiSeeds::Generated {
            seed_count,
            random_seed,
        } => {
            append_sampled_natural_seeds(
                &occupied,
                *seed_count,
                *random_seed ^ u64::from(material),
                &mut used,
                &mut seeds,
            );
        }
    }

    for (field_idx, field) in natural.fields.iter().enumerate() {
        let radius_sq = squared_u32(field.radius);
        let eligible = occupied
            .iter()
            .copied()
            .filter(|&coord| grid_distance_sq(coord, field.center) <= radius_sq)
            .collect::<Vec<_>>();
        append_sampled_natural_seeds(
            &eligible,
            field.extra_seed_count,
            field.random_seed ^ (u64::from(material) << 32) ^ field_idx as u64,
            &mut used,
            &mut seeds,
        );
    }

    if seeds.is_empty() {
        seeds.push(NaturalVoronoiRuntimeSeed {
            position: occupied[0],
        });
    }
    seeds
}

fn append_sampled_natural_seeds(
    eligible: &[GridCoord],
    seed_count: usize,
    random_seed: u64,
    used: &mut BTreeSet<GridCoord>,
    seeds: &mut Vec<NaturalVoronoiRuntimeSeed>,
) {
    if seed_count == 0 || eligible.is_empty() {
        return;
    }
    let mut weighted = eligible
        .iter()
        .copied()
        .map(|coord| (hash_grid_coord(random_seed, coord, 0), coord))
        .collect::<Vec<_>>();
    weighted.sort_unstable();
    for (_, position) in weighted.into_iter().take(seed_count) {
        if used.insert(position) {
            seeds.push(NaturalVoronoiRuntimeSeed { position });
        }
    }
}

fn assign_natural_voronoi_seed(
    coord: GridCoord,
    seeds: &[NaturalVoronoiRuntimeSeed],
    natural: &NaturalVoronoi,
) -> u32 {
    seeds
        .iter()
        .enumerate()
        .min_by_key(|(seed_idx, seed)| {
            (
                natural_voronoi_score(coord, seed, *seed_idx, natural),
                *seed_idx,
            )
        })
        .map(|(seed_idx, _)| seed_idx as u32)
        .unwrap_or(0)
}

fn natural_voronoi_score(
    coord: GridCoord,
    seed: &NaturalVoronoiRuntimeSeed,
    seed_idx: usize,
    natural: &NaturalVoronoi,
) -> i128 {
    let mut score = grid_distance_sq(coord, seed.position)
        .saturating_mul(i128::from(NaturalVoronoi::DISTANCE_SCALE_ONE));
    for field in &natural.fields {
        let radius_sq = squared_u32(field.radius);
        if grid_distance_sq(coord, field.center) <= radius_sq
            && grid_distance_sq(seed.position, field.center) <= radius_sq
        {
            let scale = i128::from(field.distance_scale.max(1));
            score = score.saturating_mul(scale) / i128::from(NaturalVoronoi::DISTANCE_SCALE_ONE);
            score = score.saturating_sub(i128::from(field.distance_bias));
        }
    }
    let amplitude = natural.noise.amplitude.saturating_abs();
    if amplitude > 0 {
        score = score.saturating_add(i128::from(signed_hash_noise(
            hash_grid_coord(
                natural.noise.seed ^ seed_idx as u64,
                coord,
                hash_grid_coord(natural.noise.seed, seed.position, 1),
            ),
            amplitude,
        )));
    }
    score
}

fn grid_distance_sq(a: GridCoord, b: GridCoord) -> i128 {
    let dx = i128::from(a.x) - i128::from(b.x);
    let dy = i128::from(a.y) - i128::from(b.y);
    dx * dx + dy * dy
}

fn squared_u32(value: u32) -> i128 {
    let value = i128::from(value);
    value * value
}

fn signed_hash_noise(hash: u64, amplitude: i64) -> i64 {
    let span = (i128::from(amplitude) * 2) + 1;
    let value = i128::from(hash) % span;
    (value - i128::from(amplitude)) as i64
}

fn hash_grid_coord(seed: u64, coord: GridCoord, salt: u64) -> u64 {
    splitmix64(
        seed ^ salt.rotate_left(17)
            ^ (u64::from(coord.x).wrapping_mul(0x9E37_79B9_7F4A_7C15))
            ^ (u64::from(coord.y).wrapping_mul(0xBF58_476D_1CE4_E5B9)),
    )
}

fn splitmix64(mut value: u64) -> u64 {
    value = value.wrapping_add(0x9E37_79B9_7F4A_7C15);
    value = (value ^ (value >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    value = (value ^ (value >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    value ^ (value >> 31)
}

fn cluster_bin_for_voxel(
    input: &VoxelAuthoringInput,
    coord: GridCoord,
    options: &VoxelAuthoringOptions,
    natural_voronoi_bins: Option<&[Option<u32>]>,
) -> ClusterBin {
    let idx = index(input.width, coord);
    let material = input.fracture_material[idx];
    let rule = options
        .material_cluster_rules
        .get(&material)
        .unwrap_or(&VoxelClusterPolicy {
            mode: VoxelClusterMode::MaterialComponents,
        });
    match &rule.mode {
        VoxelClusterMode::MaterialComponents => ClusterBin::MaterialComponent,
        VoxelClusterMode::Isotropic {
            max_extent,
            max_voxels,
        } => {
            let extent = bounded_isotropic_extent(*max_extent, *max_voxels);
            ClusterBin::Tile {
                axis: VoxelClusterAxis::X,
                direction: 0,
                x: coord.x / extent,
                y: coord.y / extent,
            }
        }
        VoxelClusterMode::Fiber {
            along_extent,
            cross_extent,
        } => {
            let angle = input.orientation.as_ref().map(|map| map[idx]).unwrap_or(0);
            let axis = orientation_major_axis(angle);
            elongated_cluster_bin(
                coord,
                axis,
                orientation_direction_bucket(angle),
                *along_extent,
                *cross_extent,
            )
        }
        VoxelClusterMode::StructuralBeam {
            axis,
            along_extent,
            cross_extent,
        } => elongated_cluster_bin(coord, *axis, 0, *along_extent, *cross_extent),
        VoxelClusterMode::NaturalVoronoi(_) => ClusterBin::NaturalVoronoi {
            seed: natural_voronoi_bins
                .and_then(|bins| bins[index(input.width, coord)])
                .unwrap_or(0),
        },
    }
}

fn bounded_isotropic_extent(max_extent: u32, max_voxels: usize) -> u32 {
    let max_voxels = max_voxels.max(1);
    let mut extent = max_extent.max(1);
    while (extent as usize).saturating_mul(extent as usize) > max_voxels && extent > 1 {
        extent -= 1;
    }
    extent
}

fn elongated_cluster_bin(
    coord: GridCoord,
    axis: VoxelClusterAxis,
    direction: u8,
    along_extent: u32,
    cross_extent: u32,
) -> ClusterBin {
    let along_extent = along_extent.max(1);
    let cross_extent = cross_extent.max(1);
    match axis {
        VoxelClusterAxis::X => ClusterBin::Tile {
            axis,
            direction,
            x: coord.x / along_extent,
            y: coord.y / cross_extent,
        },
        VoxelClusterAxis::Y => ClusterBin::Tile {
            axis,
            direction,
            x: coord.x / cross_extent,
            y: coord.y / along_extent,
        },
    }
}

fn orientation_direction_bucket(angle: u16) -> u8 {
    ((u32::from(angle) * 8) / (u32::from(u16::MAX) + 1)) as u8
}

fn orientation_major_axis(angle: u16) -> VoxelClusterAxis {
    let radians = (angle as f32 / u16::MAX as f32) * std::f32::consts::TAU;
    if radians.cos().abs() >= radians.sin().abs() {
        VoxelClusterAxis::X
    } else {
        VoxelClusterAxis::Y
    }
}

fn build_authored_chunks(
    _width: u32,
    fracture_material: &[u16],
    support_node_map: &[Option<SupportNodeId>],
    options: &VoxelAuthoringOptions,
) -> Option<Vec<Chunk2D>> {
    match options.hierarchy_policy {
        VoxelHierarchyPolicy::Flat => None,
        VoxelHierarchyPolicy::ParentChunksByMaterial => {
            let mut material_by_node = BTreeMap::<SupportNodeId, u16>::new();
            for (idx, node) in support_node_map.iter().enumerate() {
                let Some(node) = *node else {
                    continue;
                };
                material_by_node
                    .entry(node)
                    .or_insert(fracture_material[idx]);
            }
            if material_by_node.is_empty() {
                return Some(Vec::new());
            }

            let mut nodes_by_material = BTreeMap::<u16, Vec<SupportNodeId>>::new();
            for (node, material) in &material_by_node {
                nodes_by_material.entry(*material).or_default().push(*node);
            }
            let mut next_chunk_id = material_by_node
                .keys()
                .map(|node| node.0)
                .max()
                .unwrap_or(0)
                + 1;
            let mut parent_by_material = BTreeMap::<u16, ChunkId>::new();
            let mut chunks = Vec::new();
            for (material, nodes) in &nodes_by_material {
                let parent = ChunkId(next_chunk_id);
                next_chunk_id += 1;
                parent_by_material.insert(*material, parent);
                chunks.push(Chunk2D {
                    id: parent,
                    support_nodes: nodes.clone(),
                    parent: None,
                });
            }
            for (node, material) in material_by_node {
                chunks.push(Chunk2D {
                    id: ChunkId(node.0),
                    support_nodes: vec![],
                    parent: parent_by_material.get(&material).copied(),
                });
            }
            Some(chunks)
        }
    }
}

fn make_core_asset(
    width: u32,
    height: u32,
    voxel_size: f32,
    occupancy: Vec<bool>,
    fracture_material: Vec<u16>,
    orientation: Option<Vec<u16>>,
    support_node_map: Vec<Option<SupportNodeId>>,
    authored_chunks: Option<Vec<Chunk2D>>,
    default_bond_health: f32,
    default_tension_limit: f32,
    default_shear_limit: f32,
) -> Result<FxAsset, VoxelError> {
    let occupancy = fracture_core::DenseOccupancy::new(width, height, occupancy)?;
    let mut desc = FxAssetDesc::new(
        fracture_core::FxAssetId::new(1),
        voxel_size,
        occupancy,
        support_node_map,
    );
    desc.material_map = Some(fracture_material);
    desc.orientation_map = orientation;
    desc.authored_chunks = authored_chunks;
    desc.default_bond_health = default_bond_health;
    desc.default_tension_limit = default_tension_limit;
    desc.default_shear_limit = default_shear_limit;
    Ok(FxAsset::from_desc(desc)?)
}

fn connected_components_by_key<K, F>(
    width: u32,
    height: u32,
    mut key_at: F,
) -> Vec<KeyedComponent<K>>
where
    K: Clone + Ord,
    F: FnMut(GridCoord) -> Option<(K, Option<SupportNodeId>)>,
{
    let mut seen = vec![false; cell_count(width, height)];
    let mut components = Vec::new();
    for coord in all_coords(width, height) {
        let start_idx = index(width, coord);
        if seen[start_idx] {
            continue;
        }
        let Some((key, _)) = key_at(coord) else {
            seen[start_idx] = true;
            continue;
        };
        seen[start_idx] = true;
        let mut queue = VecDeque::from([coord]);
        let mut voxels = Vec::new();
        while let Some(here) = queue.pop_front() {
            voxels.push(here);
            for next in four_neighbors(width, height, here) {
                let next_idx = index(width, next);
                if seen[next_idx] {
                    continue;
                }
                if let Some((next_key, _)) = key_at(next) {
                    if next_key == key {
                        seen[next_idx] = true;
                        queue.push_back(next);
                    }
                } else {
                    seen[next_idx] = true;
                }
            }
        }
        voxels.sort_unstable();
        components.push(KeyedComponent { key, voxels });
    }
    components.sort_by_key(|component| component.voxels.first().copied());
    components
}

fn validate_input_maps(input: &VoxelAuthoringInput) -> Result<(), VoxelError> {
    let expected = cell_count(input.width, input.height);
    validate_len("occupancy", expected, input.occupancy.len())?;
    validate_len("fracture_material", expected, input.fracture_material.len())?;
    validate_len("contact_material", expected, input.contact_material.len())?;
    validate_len("external_id", expected, input.external_id.len())?;
    if let Some(orientation) = &input.orientation {
        validate_len("orientation", expected, orientation.len())?;
    }
    if let Some(hints) = &input.support_node_hint {
        validate_len("support_node_hint", expected, hints.len())?;
    }
    Ok(())
}

fn validate_len(name: &'static str, expected: usize, actual: usize) -> Result<(), VoxelError> {
    if expected == actual {
        Ok(())
    } else {
        Err(VoxelError::MapDimensionMismatch {
            map: name,
            expected,
            actual,
        })
    }
}

fn validate_exact_cover(
    width: u32,
    height: u32,
    occupancy: &[bool],
    node_map: &[Option<SupportNodeId>],
) -> Result<(), VoxelError> {
    for coord in all_coords(width, height) {
        let idx = index(width, coord);
        match (occupancy[idx], node_map[idx]) {
            (true, Some(_)) | (false, None) => {}
            (true, None) => return Err(VoxelError::MissingSupportCoverage(coord)),
            (false, Some(node)) => return Err(VoxelError::EmptyVoxelCovered { coord, node }),
        }
    }
    Ok(())
}

fn node_summaries(
    core: &FxAsset,
    contact_material: &[u16],
    external_id: &[u32],
    width: u32,
) -> Vec<NodeAuthoringSummary> {
    core.support_nodes()
        .iter()
        .map(|node| {
            let dominant_contact = dominant_by_count(node.voxels.iter().map(|coord| {
                let idx = index(width, *coord);
                contact_material[idx]
            }))
            .unwrap_or_default();
            let mut ids = node
                .voxels
                .iter()
                .map(|coord| external_id[index(width, *coord)]);
            let first = ids.next().unwrap_or_default();
            let (external_id_min, external_id_max) = ids
                .fold((first, first), |(min_id, max_id), id| {
                    (min_id.min(id), max_id.max(id))
                });
            NodeAuthoringSummary {
                node_id: node.id,
                fracture_material: node.material_id,
                contact_material_summary: dominant_contact,
                external_id_min,
                external_id_max,
                orientation_summary: node.orientation_summary,
                anisotropy_axis: node.anisotropy_axis,
                stable_seed: node.stable_seed,
                voxel_count: node.voxels.len(),
            }
        })
        .collect()
}

fn bond_summaries(
    core: &FxAsset,
    contact_material: &[u16],
    external_id: &[u32],
    width: u32,
    height: u32,
) -> Vec<BondAuthoringSummary> {
    core.internal_bonds()
        .iter()
        .map(|bond| {
            let mut contact_material_pairs = Vec::new();
            let mut external_id_pairs = Vec::new();
            for edge in &bond.interface_edges {
                let Some((coord_a, coord_b)) =
                    edge_voxels_for_bond(core, width, height, bond.node_a, bond.node_b, *edge)
                else {
                    continue;
                };
                let idx_a = index(width, coord_a);
                let idx_b = index(width, coord_b);
                contact_material_pairs.push((contact_material[idx_a], contact_material[idx_b]));
                external_id_pairs.push((external_id[idx_a], external_id[idx_b]));
            }
            contact_material_pairs.sort_unstable();
            contact_material_pairs.dedup();
            external_id_pairs.sort_unstable();
            external_id_pairs.dedup();
            BondAuthoringSummary {
                bond_id: bond.id,
                node_a: bond.node_a,
                node_b: bond.node_b,
                fracture_material_pair: bond.material_pair,
                contact_material_pairs,
                external_id_pairs,
            }
        })
        .collect()
}

fn edge_voxels_for_bond(
    core: &FxAsset,
    width: u32,
    height: u32,
    node_a: SupportNodeId,
    node_b: SupportNodeId,
    edge: fracture_core::InterfaceEdge,
) -> Option<(GridCoord, GridCoord)> {
    let candidates = edge_adjacent_voxels(width, height, edge);
    for (left, right) in candidates {
        let left_node = core.node_at(left)?;
        let right_node = core.node_at(right)?;
        if left_node == node_a && right_node == node_b {
            return Some((left, right));
        }
        if left_node == node_b && right_node == node_a {
            return Some((right, left));
        }
    }
    None
}

fn edge_adjacent_voxels(
    width: u32,
    height: u32,
    edge: fracture_core::InterfaceEdge,
) -> Vec<(GridCoord, GridCoord)> {
    let mut out = Vec::with_capacity(1);
    if edge.start.x == edge.end.x {
        let x = edge.start.x;
        let y = edge.start.y.min(edge.end.y);
        if x > 0 && x < width && y < height {
            out.push((GridCoord::new(x - 1, y), GridCoord::new(x, y)));
        }
    } else if edge.start.y == edge.end.y {
        let x = edge.start.x.min(edge.end.x);
        let y = edge.start.y;
        if y > 0 && y < height && x < width {
            out.push((GridCoord::new(x, y - 1), GridCoord::new(x, y)));
        }
    }
    out
}

fn dominant_by_count<I>(values: I) -> Option<u16>
where
    I: IntoIterator<Item = u16>,
{
    let mut counts = BTreeMap::<u16, usize>::new();
    for value in values {
        *counts.entry(value).or_default() += 1;
    }
    counts
        .into_iter()
        .max_by(|(value_a, count_a), (value_b, count_b)| {
            count_a.cmp(count_b).then_with(|| value_b.cmp(value_a))
        })
        .map(|(value, _)| value)
}

fn affected_nodes(core: &FxAsset, dirty_bbox: GridAabb) -> Vec<SupportNodeId> {
    let mut nodes = BTreeSet::new();
    for y in dirty_bbox.min.y..=dirty_bbox.max.y {
        for x in dirty_bbox.min.x..=dirty_bbox.max.x {
            if let Some(node) = core.node_at(GridCoord::new(x, y)) {
                nodes.insert(node);
            }
        }
    }
    nodes.into_iter().collect()
}

fn bbox_for_edits(width: u32, height: u32, edits: &[GridCoord]) -> Option<GridAabb> {
    let first = edits.first().copied()?;
    let mut min = first;
    let mut max = first;
    for &coord in edits.iter().skip(1) {
        min = GridCoord::new(min.x.min(coord.x), min.y.min(coord.y));
        max = GridCoord::new(max.x.max(coord.x), max.y.max(coord.y));
    }
    Some(GridAabb {
        min: GridCoord::new(min.x.saturating_sub(1), min.y.saturating_sub(1)),
        max: GridCoord::new((max.x + 1).min(width - 1), (max.y + 1).min(height - 1)),
    })
}

fn all_coords(width: u32, height: u32) -> impl Iterator<Item = GridCoord> {
    (0..height).flat_map(move |y| (0..width).map(move |x| GridCoord::new(x, y)))
}

fn four_neighbors(width: u32, height: u32, coord: GridCoord) -> impl Iterator<Item = GridCoord> {
    let mut out = Vec::with_capacity(4);
    if coord.x > 0 {
        out.push(GridCoord::new(coord.x - 1, coord.y));
    }
    if coord.y > 0 {
        out.push(GridCoord::new(coord.x, coord.y - 1));
    }
    if coord.x + 1 < width {
        out.push(GridCoord::new(coord.x + 1, coord.y));
    }
    if coord.y + 1 < height {
        out.push(GridCoord::new(coord.x, coord.y + 1));
    }
    out.into_iter()
}

fn index(width: u32, coord: GridCoord) -> usize {
    coord.y as usize * width as usize + coord.x as usize
}

fn cell_count(width: u32, height: u32) -> usize {
    width as usize * height as usize
}

#[derive(Error, Debug, Clone, PartialEq)]
pub enum VoxelError {
    #[error("map {map} dimension mismatch: expected {expected}, got {actual}")]
    MapDimensionMismatch {
        map: &'static str,
        expected: usize,
        actual: usize,
    },
    #[error("coordinate out of bounds: {0:?}")]
    CoordinateOutOfBounds(GridCoord),
    #[error("cannot add voxel at occupied coordinate {0:?}")]
    OccupiedVoxel(GridCoord),
    #[error("unknown actor {0:?}")]
    UnknownActor(FxActorId),
    #[error("old node {0:?} has no actor owner")]
    MissingOldNodeOwner(SupportNodeId),
    #[error("occupied voxel {0:?} is missing support coverage")]
    MissingSupportCoverage(GridCoord),
    #[error("empty voxel {coord:?} is covered by support node {node:?}")]
    EmptyVoxelCovered {
        coord: GridCoord,
        node: SupportNodeId,
    },
    #[error("local repair invariant violated: {0}")]
    LocalRepairInvariantViolated(&'static str),
    #[error(transparent)]
    CoreValidation(#[from] ValidationError),
    #[error(transparent)]
    CoreRepair(#[from] RepairError),
}

#[cfg(test)]
mod tests {
    use super::*;
    use fracture_core::{
        CommandId, DamageSource, DeterministicOrderKey, FractureCommand, FractureTarget,
        InterfaceEdge, apply_fracture_commands,
    };

    fn input_from_rows(rows: &[&str], materials: &[u16]) -> VoxelAuthoringInput {
        let height = rows.len() as u32;
        let width = rows.first().map_or(0, |row| row.len()) as u32;
        let mut occupancy = Vec::new();
        for row in rows {
            assert_eq!(row.len() as u32, width);
            for byte in row.bytes() {
                occupancy.push(matches!(byte, b'#' | b'1' | b'A' | b'B'));
            }
        }
        assert_eq!(materials.len(), occupancy.len());
        let mut input = VoxelAuthoringInput::new(
            width,
            height,
            1.0,
            occupancy,
            materials.to_vec(),
            vec![0; materials.len()],
            (0..materials.len() as u32).collect(),
        );
        input.default_bond_health = 10.0;
        input
    }

    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() <= 0.000_01
    }

    #[test]
    fn transfer_node_state_weights_health_and_damage_by_overlap() {
        let old_state = NodeRuntimeState {
            health: 0.6,
            accumulated_damage: 0.4,
        };
        let half = transfer_node_state(&old_state, 1, 2);
        assert!(approx_eq(half.health, 0.3));
        assert!(approx_eq(half.accumulated_damage, 0.2));

        let zero = transfer_node_state(&old_state, 0, 2);
        assert_eq!(zero.health, 0.0);
        assert_eq!(zero.accumulated_damage, 0.0);
    }

    fn voxel_bbox_size(voxels: &[GridCoord]) -> (u32, u32) {
        let first = voxels[0];
        let mut min = first;
        let mut max = first;
        for &coord in voxels.iter().skip(1) {
            min = GridCoord::new(min.x.min(coord.x), min.y.min(coord.y));
            max = GridCoord::new(max.x.max(coord.x), max.y.max(coord.y));
        }
        (max.x - min.x + 1, max.y - min.y + 1)
    }

    fn node_assignment(asset: &AuthoredVoxelAsset) -> Vec<Option<SupportNodeId>> {
        all_coords(asset.width, asset.height)
            .map(|coord| asset.core().node_at(coord))
            .collect()
    }

    #[cfg(feature = "parallel")]
    fn author_voxel_asset_with_serial_natural_voronoi(
        input: VoxelAuthoringInput,
        options: VoxelAuthoringOptions,
    ) -> Result<AuthoredVoxelAsset, VoxelError> {
        validate_input_maps(&input)?;
        let natural_voronoi_bins = if input.support_node_hint.is_some() {
            None
        } else {
            build_natural_voronoi_bins_serial(&input, &options)
        };
        let support_node_map = build_authoring_support_node_map_with_natural_voronoi_bins(
            &input,
            &options,
            natural_voronoi_bins.as_deref(),
        );
        let authored_chunks = build_authored_chunks(
            input.width,
            &input.fracture_material,
            &support_node_map,
            &options,
        );
        let core = make_core_asset(
            input.width,
            input.height,
            input.voxel_size,
            input.occupancy.clone(),
            input.fracture_material.clone(),
            input.orientation.clone(),
            support_node_map,
            authored_chunks,
            input.default_bond_health,
            input.default_tension_limit,
            input.default_shear_limit,
        )?;
        let asset = AuthoredVoxelAsset {
            summaries: node_summaries(
                &core,
                &input.contact_material,
                &input.external_id,
                input.width,
            ),
            bond_summaries: bond_summaries(
                &core,
                &input.contact_material,
                &input.external_id,
                input.width,
                input.height,
            ),
            core,
            width: input.width,
            height: input.height,
            contact_material: input.contact_material,
            fracture_material: input.fracture_material,
            external_id: input.external_id,
            orientation: input.orientation,
            default_bond_health: input.default_bond_health,
            default_tension_limit: input.default_tension_limit,
            default_shear_limit: input.default_shear_limit,
        };
        asset.validate_exact_cover()?;
        Ok(asset)
    }

    #[test]
    fn authoring_exact_cover_property_small_grids() {
        for width in 1..=3 {
            for height in 1..=3 {
                let cells = (width * height) as usize;
                for mask in 0..(1u32 << cells) {
                    let occupancy = (0..cells)
                        .map(|bit| (mask & (1 << bit)) != 0)
                        .collect::<Vec<_>>();
                    let input = VoxelAuthoringInput::new(
                        width,
                        height,
                        1.0,
                        occupancy,
                        vec![1; cells],
                        vec![0; cells],
                        vec![0; cells],
                    );
                    let asset = author_voxel_asset(input).unwrap();
                    asset.validate_exact_cover().unwrap();
                }
            }
        }
    }

    #[test]
    fn material_boundaries_do_not_merge() {
        let asset = author_voxel_asset(input_from_rows(&["##"], &[1, 2])).unwrap();
        assert_eq!(asset.core().support_nodes().len(), 2);
        assert_eq!(asset.core().internal_bonds().len(), 1);
    }

    #[test]
    fn cluster_authoring_brittle_isotropic_splits_large_material_component() {
        let input = input_from_rows(&["####", "####", "####", "####"], &[1; 16]);
        let options = VoxelAuthoringOptions::default()
            .with_material_rule(1, VoxelClusterPolicy::brittle_isotropic(2, 4));

        let asset = author_voxel_asset_with_options(input.clone(), options.clone()).unwrap();
        let repeated = author_voxel_asset_with_options(input, options).unwrap();

        assert_eq!(asset.core().support_nodes().len(), 4);
        assert!(
            asset
                .core()
                .support_nodes()
                .iter()
                .all(|node| node.voxels.len() == 4 && voxel_bbox_size(&node.voxels) == (2, 2))
        );
        asset.validate_exact_cover().unwrap();
        assert_eq!(
            asset.core().support_nodes(),
            repeated.core().support_nodes()
        );
        assert_eq!(
            asset.core().internal_bonds(),
            repeated.core().internal_bonds()
        );
        assert_eq!(asset.core().internal_bonds().len(), 4);
        assert!(
            asset
                .core()
                .internal_bonds()
                .iter()
                .all(|bond| !bond.interface_edges.is_empty())
        );
    }

    #[test]
    fn cluster_authoring_generates_bonds_from_policy_node_boundaries() {
        let input = input_from_rows(&["####"], &[1, 1, 1, 1]);
        let options = VoxelAuthoringOptions::default()
            .with_material_rule(1, VoxelClusterPolicy::isotropic(2, 4));

        let asset = author_voxel_asset_with_options(input, options).unwrap();

        assert_eq!(asset.core().support_nodes().len(), 2);
        assert_eq!(asset.core().internal_bonds().len(), 1);
        let bond = &asset.core().internal_bonds()[0];
        assert_eq!(bond.node_a, SupportNodeId(0));
        assert_eq!(bond.node_b, SupportNodeId(1));
        assert_eq!(bond.length, 1.0);
        assert_eq!(bond.interface_edges.len(), 1);
        asset.validate_exact_cover().unwrap();
    }

    #[test]
    fn natural_voronoi_generates_non_rectangular_support_node() {
        let input = input_from_rows(&["#..", "#..", "###"], &[1, 0, 0, 1, 0, 0, 1, 1, 1]);
        let options = VoxelAuthoringOptions::default().with_material_rule(
            1,
            VoxelClusterPolicy::natural_voronoi(vec![GridCoord::new(0, 0)]),
        );

        let asset = author_voxel_asset_with_options(input, options).unwrap();

        assert_eq!(asset.core().support_nodes().len(), 1);
        let node = &asset.core().support_nodes()[0];
        let (bbox_width, bbox_height) = voxel_bbox_size(&node.voxels);
        assert!(bbox_width as usize * bbox_height as usize > node.voxels.len());
        asset.validate_exact_cover().unwrap();
    }

    #[test]
    fn natural_voronoi_is_shape_aware_on_shape_with_hole() {
        let input = input_from_rows(&["###", "#.#", "###"], &[1, 1, 1, 1, 0, 1, 1, 1, 1]);
        let options = VoxelAuthoringOptions::default().with_material_rule(
            1,
            VoxelClusterPolicy::natural_voronoi(vec![
                GridCoord::new(0, 0),
                GridCoord::new(2, 0),
                GridCoord::new(0, 2),
                GridCoord::new(2, 2),
            ]),
        );

        let asset = author_voxel_asset_with_options(input, options).unwrap();

        asset.validate_exact_cover().unwrap();
        assert_eq!(asset.core().node_at(GridCoord::new(1, 1)), None);
        let covered = asset
            .core()
            .support_nodes()
            .iter()
            .flat_map(|node| node.voxels.iter().copied())
            .collect::<BTreeSet<_>>();
        let occupied = all_coords(asset.width, asset.height)
            .filter(|&coord| asset.core().occupancy().is_occupied(coord))
            .collect::<BTreeSet<_>>();
        assert_eq!(covered, occupied);
    }

    #[test]
    fn natural_voronoi_repeat_is_deterministic() {
        let input = input_from_rows(&["######", "######", "######"], &[1; 18]);
        let options = VoxelAuthoringOptions::default().with_material_rule(
            1,
            VoxelClusterPolicy {
                mode: VoxelClusterMode::NaturalVoronoi(
                    NaturalVoronoi::generated(5, 1234).with_noise(77, 600),
                ),
            },
        );

        let first = author_voxel_asset_with_options(input.clone(), options.clone()).unwrap();
        let second = author_voxel_asset_with_options(input, options).unwrap();

        assert_eq!(node_assignment(&first), node_assignment(&second));
        assert_eq!(first.core().support_nodes(), second.core().support_nodes());
        assert_eq!(
            first.core().internal_bonds(),
            second.core().internal_bonds()
        );
    }

    #[cfg(feature = "parallel")]
    #[test]
    fn natural_voronoi_parallel_matches_serial_generated_noise_authoring() {
        let width = 32;
        let height = 32;
        let cells = cell_count(width, height);
        let input = VoxelAuthoringInput::new(
            width,
            height,
            1.0,
            vec![true; cells],
            vec![1; cells],
            vec![0; cells],
            (0..cells as u32).collect(),
        );
        let options = VoxelAuthoringOptions::default().with_material_rule(
            1,
            VoxelClusterPolicy {
                mode: VoxelClusterMode::NaturalVoronoi(
                    NaturalVoronoi::generated(29, 0xA11C_E501)
                        .with_noise(0x51A7_EE55, 700)
                        .with_field(
                            NaturalVoronoiClusterField::new(GridCoord::new(11, 10), 9)
                                .with_extra_seeds(6, 0xF1E1_D501)
                                .with_distance_bias(384, 224),
                        ),
                ),
            },
        );

        let serial =
            author_voxel_asset_with_serial_natural_voronoi(input.clone(), options.clone()).unwrap();
        let parallel = author_voxel_asset_with_options(input, options).unwrap();

        assert_eq!(node_assignment(&parallel), node_assignment(&serial));
        assert_eq!(
            parallel.core().support_nodes().len(),
            serial.core().support_nodes().len()
        );
        assert_eq!(
            parallel.core().internal_bonds().len(),
            serial.core().internal_bonds().len()
        );
        assert_eq!(
            parallel.core().support_nodes(),
            serial.core().support_nodes()
        );
        assert_eq!(
            parallel.core().internal_bonds(),
            serial.core().internal_bonds()
        );
    }

    #[test]
    fn natural_voronoi_noise_seed_or_amplitude_changes_assignment() {
        let input = input_from_rows(&["#####", "#####", "#####"], &[1; 15]);
        let base = NaturalVoronoi::explicit(vec![GridCoord::new(1, 1), GridCoord::new(3, 1)]);
        let quiet = VoxelAuthoringOptions::default().with_material_rule(
            1,
            VoxelClusterPolicy {
                mode: VoxelClusterMode::NaturalVoronoi(base.clone()),
            },
        );
        let noisy_a = VoxelAuthoringOptions::default().with_material_rule(
            1,
            VoxelClusterPolicy {
                mode: VoxelClusterMode::NaturalVoronoi(base.clone().with_noise(11, 4096)),
            },
        );
        let noisy_b = VoxelAuthoringOptions::default().with_material_rule(
            1,
            VoxelClusterPolicy {
                mode: VoxelClusterMode::NaturalVoronoi(base.with_noise(12, 4096)),
            },
        );

        let quiet = author_voxel_asset_with_options(input.clone(), quiet).unwrap();
        let noisy_a = author_voxel_asset_with_options(input.clone(), noisy_a).unwrap();
        let noisy_b = author_voxel_asset_with_options(input, noisy_b).unwrap();

        assert_ne!(node_assignment(&quiet), node_assignment(&noisy_a));
        assert_ne!(node_assignment(&noisy_a), node_assignment(&noisy_b));
        quiet.validate_exact_cover().unwrap();
        noisy_a.validate_exact_cover().unwrap();
        noisy_b.validate_exact_cover().unwrap();
    }

    #[test]
    fn natural_voronoi_cluster_field_adds_local_fragments() {
        let input = input_from_rows(&["########", "########", "########", "########"], &[1; 32]);
        let base = VoxelAuthoringOptions::default()
            .with_material_rule(1, VoxelClusterPolicy::natural_voronoi_generated(2, 91));
        let with_field = VoxelAuthoringOptions::default().with_material_rule(
            1,
            VoxelClusterPolicy {
                mode: VoxelClusterMode::NaturalVoronoi(
                    NaturalVoronoi::generated(2, 91).with_field(
                        NaturalVoronoiClusterField::new(GridCoord::new(1, 1), 2)
                            .with_extra_seeds(4, 333)
                            .with_distance_bias(512, 256),
                    ),
                ),
            },
        );

        let base = author_voxel_asset_with_options(input.clone(), base).unwrap();
        let with_field = author_voxel_asset_with_options(input, with_field).unwrap();

        assert!(with_field.core().support_nodes().len() > base.core().support_nodes().len());
        assert_ne!(node_assignment(&base), node_assignment(&with_field));
        with_field.validate_exact_cover().unwrap();
    }

    #[test]
    fn natural_voronoi_per_material_seed_indices_do_not_merge_across_boundary() {
        let input = input_from_rows(&["AABB", "AABB"], &[1, 1, 2, 2, 1, 1, 2, 2]);
        let options = VoxelAuthoringOptions::default()
            .with_material_rule(1, VoxelClusterPolicy::natural_voronoi_generated(2, 77))
            .with_material_rule(2, VoxelClusterPolicy::natural_voronoi_generated(2, 77));

        let asset = author_voxel_asset_with_options(input, options).unwrap();

        assert!(asset.core().support_nodes().len() >= 4);
        assert!(
            asset
                .core()
                .support_nodes()
                .iter()
                .all(
                    |node| node.voxels.iter().all(|coord| asset.fracture_material_map()
                        [index(asset.width, *coord)]
                        == node.material_id)
                )
        );
        assert!(asset.core().internal_bonds().iter().any(|bond| {
            let material_a = asset.core().node(bond.node_a).unwrap().material_id;
            let material_b = asset.core().node(bond.node_b).unwrap().material_id;
            material_a != material_b
        }));
        asset.validate_exact_cover().unwrap();
    }

    #[test]
    fn natural_voronoi_distance_only_field_changes_ownership() {
        let input = input_from_rows(&["#####"], &[1; 5]);
        let base_rule = NaturalVoronoi::explicit(vec![GridCoord::new(0, 0), GridCoord::new(4, 0)]);
        let base = VoxelAuthoringOptions::default().with_material_rule(
            1,
            VoxelClusterPolicy {
                mode: VoxelClusterMode::NaturalVoronoi(base_rule.clone()),
            },
        );
        let with_field = VoxelAuthoringOptions::default().with_material_rule(
            1,
            VoxelClusterPolicy {
                mode: VoxelClusterMode::NaturalVoronoi(
                    base_rule.with_field(
                        NaturalVoronoiClusterField::new(GridCoord::new(0, 0), 3)
                            .with_distance_bias(1, 0),
                    ),
                ),
            },
        );

        let base = author_voxel_asset_with_options(input.clone(), base).unwrap();
        let with_field = author_voxel_asset_with_options(input, with_field).unwrap();

        assert_eq!(base.core().support_nodes().len(), 2);
        assert_eq!(with_field.core().support_nodes().len(), 2);
        assert_ne!(node_assignment(&base), node_assignment(&with_field));
        assert_eq!(
            base.core().node_at(GridCoord::new(3, 0)),
            base.core().node_at(GridCoord::new(4, 0))
        );
        assert_eq!(
            with_field.core().node_at(GridCoord::new(3, 0)),
            with_field.core().node_at(GridCoord::new(0, 0))
        );
        base.validate_exact_cover().unwrap();
        with_field.validate_exact_cover().unwrap();
    }

    #[test]
    fn natural_voronoi_multi_seed_noise_can_generate_non_rectangular_node() {
        let input = input_from_rows(
            &["######", "#....#", "######"],
            &[1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1],
        );
        let options = VoxelAuthoringOptions::default().with_material_rule(
            1,
            VoxelClusterPolicy {
                mode: VoxelClusterMode::NaturalVoronoi(
                    NaturalVoronoi::generated(4, 901).with_noise(444, 300),
                ),
            },
        );

        let asset = author_voxel_asset_with_options(input, options).unwrap();

        assert!(asset.core().support_nodes().len() > 1);
        assert!(asset.core().support_nodes().iter().any(|node| {
            let (bbox_width, bbox_height) = voxel_bbox_size(&node.voxels);
            bbox_width as usize * bbox_height as usize > node.voxels.len()
        }));
        asset.validate_exact_cover().unwrap();
    }

    #[test]
    fn natural_voronoi_generates_bonds_from_cell_boundaries() {
        let input = input_from_rows(&["####", "####"], &[1; 8]);
        let options = VoxelAuthoringOptions::default().with_material_rule(
            1,
            VoxelClusterPolicy::natural_voronoi(vec![GridCoord::new(0, 0), GridCoord::new(3, 1)]),
        );

        let asset = author_voxel_asset_with_options(input, options).unwrap();

        assert!(asset.core().support_nodes().len() >= 2);
        assert!(!asset.core().internal_bonds().is_empty());
        assert!(
            asset
                .core()
                .internal_bonds()
                .iter()
                .all(|bond| !bond.interface_edges.is_empty())
        );
        asset.validate_exact_cover().unwrap();
    }

    #[test]
    fn cluster_authoring_fiber_orientation_elongs_clusters() {
        let mut horizontal = input_from_rows(&["####", "####", "####", "####"], &[1; 16]);
        horizontal.orientation = Some(vec![0; 16]);
        let mut vertical = horizontal.clone();
        vertical.orientation = Some(vec![u16::MAX / 4; 16]);
        let options =
            VoxelAuthoringOptions::default().with_material_rule(1, VoxelClusterPolicy::fiber(4, 1));

        let horizontal = author_voxel_asset_with_options(horizontal, options.clone()).unwrap();
        let vertical = author_voxel_asset_with_options(vertical, options).unwrap();

        assert_eq!(horizontal.core().support_nodes().len(), 4);
        assert!(
            horizontal
                .core()
                .support_nodes()
                .iter()
                .all(|node| voxel_bbox_size(&node.voxels) == (4, 1))
        );
        assert_eq!(
            horizontal.core().node_at(GridCoord::new(0, 0)),
            horizontal.core().node_at(GridCoord::new(3, 0))
        );
        assert_ne!(
            horizontal.core().node_at(GridCoord::new(0, 0)),
            horizontal.core().node_at(GridCoord::new(0, 1))
        );

        assert_eq!(vertical.core().support_nodes().len(), 4);
        assert!(
            vertical
                .core()
                .support_nodes()
                .iter()
                .all(|node| voxel_bbox_size(&node.voxels) == (1, 4))
        );
        assert_eq!(
            vertical.core().node_at(GridCoord::new(0, 0)),
            vertical.core().node_at(GridCoord::new(0, 3))
        );
        assert_ne!(
            vertical.core().node_at(GridCoord::new(0, 0)),
            vertical.core().node_at(GridCoord::new(1, 0))
        );
        horizontal.validate_exact_cover().unwrap();
        vertical.validate_exact_cover().unwrap();
    }

    #[test]
    fn cluster_authoring_structural_beam_keeps_column_clusters() {
        let input = input_from_rows(&["##", "##", "##", "##", "##", "##"], &[1; 12]);
        let options = VoxelAuthoringOptions::default().with_material_rule(
            1,
            VoxelClusterPolicy::structural_beam(VoxelClusterAxis::Y, 8, 1),
        );

        let asset = author_voxel_asset_with_options(input, options).unwrap();

        assert_eq!(asset.core().support_nodes().len(), 2);
        assert!(
            asset
                .core()
                .support_nodes()
                .iter()
                .all(|node| node.voxels.len() == 6 && voxel_bbox_size(&node.voxels) == (1, 6))
        );
        assert_eq!(
            asset.core().node_at(GridCoord::new(0, 0)),
            asset.core().node_at(GridCoord::new(0, 5))
        );
        assert_ne!(
            asset.core().node_at(GridCoord::new(0, 0)),
            asset.core().node_at(GridCoord::new(1, 0))
        );
        asset.validate_exact_cover().unwrap();
    }

    #[test]
    fn cluster_authoring_material_rules_mix_without_cross_material_merge() {
        let materials = [
            1, 1, 1, 1, 2, 2, 2, 3, 1, 1, 1, 1, 2, 2, 2, 3, 1, 1, 1, 1, 2, 2, 2, 3, 1, 1, 1, 1, 2,
            2, 2, 3,
        ];
        let mut input = input_from_rows(
            &["########", "########", "########", "########"],
            &materials,
        );
        input.orientation = Some(
            materials
                .iter()
                .map(|material| if *material == 2 { u16::MAX / 4 } else { 0 })
                .collect(),
        );
        let options = VoxelAuthoringOptions::default()
            .with_material_rule(1, VoxelClusterPolicy::brittle_isotropic(2, 4))
            .with_material_rule(2, VoxelClusterPolicy::fiber(8, 1))
            .with_material_rule(
                3,
                VoxelClusterPolicy::structural_beam(VoxelClusterAxis::Y, 8, 1),
            );

        let asset = author_voxel_asset_with_options(input, options).unwrap();

        let material1 = asset
            .core()
            .support_nodes()
            .iter()
            .filter(|node| node.material_id == 1)
            .collect::<Vec<_>>();
        let material2 = asset
            .core()
            .support_nodes()
            .iter()
            .filter(|node| node.material_id == 2)
            .collect::<Vec<_>>();
        let material3 = asset
            .core()
            .support_nodes()
            .iter()
            .filter(|node| node.material_id == 3)
            .collect::<Vec<_>>();

        assert_eq!(material1.len(), 4);
        assert!(
            material1
                .iter()
                .all(|node| voxel_bbox_size(&node.voxels) == (2, 2))
        );
        assert_eq!(material2.len(), 3);
        assert!(
            material2
                .iter()
                .all(|node| voxel_bbox_size(&node.voxels) == (1, 4))
        );
        assert_eq!(material3.len(), 1);
        assert_eq!(voxel_bbox_size(&material3[0].voxels), (1, 4));
        assert!(
            asset
                .core()
                .support_nodes()
                .iter()
                .all(
                    |node| node.voxels.iter().all(|coord| asset.fracture_material_map()
                        [index(asset.width, *coord)]
                        == node.material_id)
                )
        );
        asset.validate_exact_cover().unwrap();
    }

    #[test]
    fn cluster_authoring_fiber_orientation_discontinuity_splits_growth() {
        let mut input = input_from_rows(&["####", "####"], &[1; 8]);
        input.orientation = Some(vec![
            u16::MAX / 8,
            u16::MAX / 8,
            3 * (u16::MAX / 8),
            3 * (u16::MAX / 8),
            u16::MAX / 8,
            u16::MAX / 8,
            3 * (u16::MAX / 8),
            3 * (u16::MAX / 8),
        ]);
        let options =
            VoxelAuthoringOptions::default().with_material_rule(1, VoxelClusterPolicy::fiber(8, 2));

        let asset = author_voxel_asset_with_options(input, options).unwrap();

        assert_eq!(
            asset.core().node_at(GridCoord::new(0, 0)),
            asset.core().node_at(GridCoord::new(1, 0))
        );
        assert_eq!(
            asset.core().node_at(GridCoord::new(2, 0)),
            asset.core().node_at(GridCoord::new(3, 0))
        );
        assert_ne!(
            asset.core().node_at(GridCoord::new(1, 0)),
            asset.core().node_at(GridCoord::new(2, 0))
        );
        assert_eq!(asset.core().support_nodes().len(), 2);
        asset.validate_exact_cover().unwrap();
    }

    #[test]
    fn voxel_authoring_parent_chunks_by_material_creates_non_leaf_support_chunks() {
        let input = input_from_rows(&["####", "####"], &[1, 1, 2, 2, 1, 1, 2, 2]);
        let options = VoxelAuthoringOptions {
            material_cluster_rules: BTreeMap::from([
                (1, VoxelClusterPolicy::isotropic(1, 1)),
                (2, VoxelClusterPolicy::isotropic(1, 1)),
            ]),
            hierarchy_policy: VoxelHierarchyPolicy::ParentChunksByMaterial,
        };

        let asset = author_voxel_asset_with_options(input, options).unwrap();
        let parent_ids = asset
            .core()
            .chunks()
            .iter()
            .filter_map(|chunk| chunk.parent)
            .collect::<BTreeSet<_>>();
        let leaves = asset
            .core()
            .chunks()
            .iter()
            .filter(|chunk| !parent_ids.contains(&chunk.id))
            .collect::<Vec<_>>();
        let parents = asset
            .core()
            .chunks()
            .iter()
            .filter(|chunk| parent_ids.contains(&chunk.id))
            .collect::<Vec<_>>();

        assert_eq!(asset.core().support_nodes().len(), 8);
        assert_eq!(leaves.len(), asset.core().support_nodes().len());
        assert_eq!(parents.len(), 2);
        assert!(
            leaves
                .iter()
                .all(|chunk| chunk.parent.is_some() && chunk.support_nodes.is_empty())
        );
        for node in asset.core().support_nodes() {
            assert!(parents.iter().any(|chunk| {
                chunk.id == node.chunk_id && chunk.support_nodes.contains(&node.id)
            }));
        }
        for parent in parents {
            assert!(!parent.support_nodes.is_empty());
            let covered_materials = parent
                .support_nodes
                .iter()
                .map(|node| asset.core().node(*node).unwrap().material_id)
                .collect::<BTreeSet<_>>();
            assert_eq!(covered_materials.len(), 1);
            assert!(leaves.iter().any(|leaf| leaf.parent == Some(parent.id)));
        }
        asset.core().validate().unwrap();
        asset.validate_exact_cover().unwrap();
    }

    #[test]
    fn asset_metrics_count_occupied_voxels_and_support_nodes() {
        let asset =
            author_voxel_asset(input_from_rows(&["#.#", "###"], &[1, 0, 2, 1, 1, 2])).unwrap();
        assert_eq!(
            asset.metrics(),
            VoxelAssetMetrics {
                occupied_voxels: 5,
                support_nodes: 2,
            }
        );
    }

    #[test]
    fn support_node_hint_does_not_merge_different_materials() {
        let mut input = input_from_rows(&["##"], &[1, 2]);
        input.support_node_hint = Some(vec![Some(7), Some(7)]);
        let asset = author_voxel_asset(input).unwrap();
        assert_eq!(asset.core().support_nodes().len(), 2);
        assert_eq!(asset.core().support_nodes()[0].material_id, 1);
        assert_eq!(asset.core().support_nodes()[1].material_id, 2);
        assert_eq!(asset.core().internal_bonds().len(), 1);
    }

    #[test]
    fn cluster_authoring_support_node_hint_forces_map_without_cross_material_merge() {
        let mut input = input_from_rows(&["####"], &[1, 1, 2, 2]);
        input.support_node_hint = Some(vec![Some(7), Some(7), Some(7), Some(7)]);
        let options = VoxelAuthoringOptions::default()
            .with_material_rule(1, VoxelClusterPolicy::isotropic(1, 1))
            .with_material_rule(2, VoxelClusterPolicy::isotropic(1, 1));

        let asset = author_voxel_asset_with_options(input, options).unwrap();

        assert_eq!(asset.core().support_nodes().len(), 2);
        assert_eq!(
            asset.core().node_at(GridCoord::new(0, 0)),
            Some(SupportNodeId(7))
        );
        assert_eq!(
            asset.core().node_at(GridCoord::new(1, 0)),
            Some(SupportNodeId(7))
        );
        assert_eq!(
            asset.core().node_at(GridCoord::new(2, 0)),
            Some(SupportNodeId(8))
        );
        assert_eq!(
            asset.core().node_at(GridCoord::new(3, 0)),
            Some(SupportNodeId(8))
        );
        assert_eq!(asset.core().support_nodes()[0].material_id, 1);
        assert_eq!(asset.core().support_nodes()[1].material_id, 2);
        asset.validate_exact_cover().unwrap();
    }

    #[test]
    fn contact_material_and_external_id_are_observable_in_authoring_summaries() {
        let input = VoxelAuthoringInput {
            width: 2,
            height: 1,
            voxel_size: 1.0,
            occupancy: vec![true, true],
            fracture_material: vec![1, 2],
            contact_material: vec![7, 8],
            external_id: vec![42, 99],
            orientation: Some(vec![11, 22]),
            support_node_hint: None,
            default_bond_health: 10.0,
            default_tension_limit: 10.0,
            default_shear_limit: 10.0,
        };
        let asset = author_voxel_asset(input).unwrap();
        assert_eq!(asset.contact_material_map(), &[7, 8]);
        assert_eq!(asset.external_id_map(), &[42, 99]);
        assert_eq!(asset.orientation_map(), Some(&[11, 22][..]));

        let left = asset.voxel_metadata(GridCoord::new(0, 0)).unwrap();
        assert_eq!(left.node, Some(SupportNodeId(0)));
        assert_eq!(left.contact_material, 7);
        assert_eq!(left.external_id, 42);
        assert_eq!(left.orientation, Some(11));

        assert_eq!(asset.node_summaries()[0].contact_material_summary, 7);
        assert_eq!(asset.node_summaries()[0].external_id_min, 42);
        assert_eq!(asset.node_summaries()[1].contact_material_summary, 8);
        assert_eq!(asset.node_summaries()[1].external_id_max, 99);

        let bond = &asset.bond_summaries()[0];
        assert_eq!(bond.fracture_material_pair, (1, 2));
        assert_eq!(bond.contact_material_pairs, vec![(7, 8)]);
        assert_eq!(bond.external_id_pairs, vec![(42, 99)]);
    }

    #[test]
    fn orientation_map_affects_node_summary_stably() {
        let mut a = input_from_rows(&["##"], &[1, 1]);
        a.orientation = Some(vec![0, 0]);
        let mut b = a.clone();
        b.orientation = Some(vec![1024, 1024]);
        let a0 = author_voxel_asset(a.clone()).unwrap();
        let a1 = author_voxel_asset(a).unwrap();
        let b0 = author_voxel_asset(b).unwrap();
        assert_eq!(a0.node_summaries(), a1.node_summaries());
        assert_ne!(
            a0.node_summaries()[0].stable_seed,
            b0.node_summaries()[0].stable_seed
        );
        assert_eq!(b0.node_summaries()[0].orientation_summary, Some(1024));
    }

    #[test]
    fn authoring_map_dimension_mismatches_are_rejected() {
        let cases = [
            "fracture_material",
            "contact_material",
            "external_id",
            "orientation",
            "support_node_hint",
        ];
        for map in cases {
            let mut input = input_from_rows(&["##"], &[1, 1]);
            match map {
                "fracture_material" => {
                    input.fracture_material.pop();
                }
                "contact_material" => {
                    input.contact_material.pop();
                }
                "external_id" => {
                    input.external_id.pop();
                }
                "orientation" => {
                    input.orientation = Some(vec![0]);
                }
                "support_node_hint" => {
                    input.support_node_hint = Some(vec![Some(0)]);
                }
                _ => unreachable!(),
            }
            assert!(matches!(
                author_voxel_asset(input),
                Err(VoxelError::MapDimensionMismatch { map: actual, .. }) if actual == map
            ));
        }
    }

    #[test]
    fn bond_generation_from_authoring_edge_scan() {
        let asset = author_voxel_asset(input_from_rows(&["##"], &[1, 2])).unwrap();
        let bond = &asset.core().internal_bonds()[0];
        assert_eq!(bond.length, 1.0);
        assert_eq!(bond.interface_edges.len(), 1);
    }

    #[test]
    fn disconnected_interface_expands_bonds_from_authoring() {
        let occupancy = vec![
            true, true, true, true, true, true, false, true, true, true, true, true,
        ];
        let hints = vec![
            Some(0),
            Some(1),
            Some(1),
            Some(1),
            Some(0),
            Some(0),
            None,
            Some(1),
            Some(0),
            Some(1),
            Some(1),
            Some(1),
        ];
        let materials = hints
            .iter()
            .map(|hint| match hint {
                Some(0) => 1,
                Some(_) => 2,
                None => 0,
            })
            .collect::<Vec<_>>();
        let mut input =
            VoxelAuthoringInput::new(4, 3, 1.0, occupancy, materials, vec![0; 12], vec![0; 12]);
        input.support_node_hint = Some(hints);
        let asset = author_voxel_asset(input).unwrap();
        assert_eq!(asset.core().internal_bonds().len(), 2);
    }

    #[test]
    fn remove_voxel_splits_node() {
        let asset = author_voxel_asset(input_from_rows(&["###"], &[1, 1, 1])).unwrap();
        let mut runtime = VoxelRuntime::instantiate(FxFamilyId(1), asset);
        let damage = FractureCommand {
            order_key: DeterministicOrderKey::new(
                1,
                1,
                runtime.family().id,
                FxActorId(0),
                CommandId(0),
            ),
            actor: FxActorId(0),
            target: FractureTarget::Node(SupportNodeId(0)),
            health_loss: 0.4,
            effective_length_loss: 0.0,
            source: DamageSource::Script,
        };
        apply_fracture_commands(&mut runtime.family, &[damage]);
        let report = runtime
            .apply_edit(RuntimeEdit::RemoveVoxels {
                voxels: vec![GridCoord::new(1, 0)],
            })
            .unwrap();
        assert_eq!(report.affected_old_nodes, vec![SupportNodeId(0)]);
        assert_eq!(runtime.family().actor_count(), 1);
        let healths = runtime
            .family()
            .node_states()
            .map(|(_, state)| (state.health, state.accumulated_damage))
            .collect::<Vec<_>>();
        assert_eq!(
            healths
                .iter()
                .filter(|(health, damage)| approx_eq(*health, 0.3) && approx_eq(*damage, 0.2))
                .count(),
            2
        );
        let splits = runtime.split_dirty_actors();
        assert_eq!(splits.len(), 1);
        assert_eq!(runtime.family().actor_count(), 2);
    }

    #[test]
    fn add_voxel_no_auto_merge() {
        let mut input = input_from_rows(&["##."], &[1, 1, 0]);
        input.support_node_hint = Some(vec![Some(0), Some(1), None]);
        let asset = author_voxel_asset(input).unwrap();
        let mut runtime = VoxelRuntime::instantiate(FxFamilyId(1), asset);
        assert_eq!(runtime.family().actor_count(), 1);
        let report = runtime
            .apply_edit(RuntimeEdit::AddVoxels {
                actor: FxActorId(0),
                voxels: vec![VoxelAdd {
                    coord: GridCoord::new(2, 0),
                    fracture_material: 1,
                    contact_material: 0,
                    external_id: 9,
                    orientation: None,
                }],
            })
            .unwrap();
        assert_eq!(runtime.family().actor_count(), 1);
        assert_eq!(runtime.asset().core().support_nodes().len(), 3);
        assert_eq!(
            runtime.asset().core().node_at(GridCoord::new(1, 0)),
            Some(SupportNodeId(1))
        );
        let added_node = runtime
            .asset()
            .core()
            .node_at(GridCoord::new(2, 0))
            .unwrap();
        assert_ne!(added_node, SupportNodeId(1));
        assert!(report.new_nodes.contains(&added_node));
        assert_eq!(
            runtime.family().node_owner(SupportNodeId(1)),
            Some(FxActorId(0))
        );
    }

    #[test]
    fn add_voxel_rejects_occupied_cell() {
        let asset = author_voxel_asset(input_from_rows(&["#"], &[1])).unwrap();
        let mut runtime = VoxelRuntime::instantiate(FxFamilyId(1), asset);
        let err = runtime
            .apply_edit(RuntimeEdit::AddVoxels {
                actor: FxActorId(0),
                voxels: vec![VoxelAdd {
                    coord: GridCoord::new(0, 0),
                    fracture_material: 1,
                    contact_material: 0,
                    external_id: 9,
                    orientation: None,
                }],
            })
            .unwrap_err();
        assert_eq!(err, VoxelError::OccupiedVoxel(GridCoord::new(0, 0)));
    }

    #[test]
    fn repair_preserves_bond_damage() {
        let asset = author_voxel_asset(input_from_rows(&["AAB"], &[1, 1, 2])).unwrap();
        let mut runtime = VoxelRuntime::instantiate(FxFamilyId(1), asset);
        let damage = FractureCommand {
            order_key: DeterministicOrderKey::new(
                1,
                1,
                runtime.family().id,
                FxActorId(0),
                CommandId(0),
            ),
            actor: FxActorId(0),
            target: FractureTarget::Bond(BondId(0)),
            health_loss: 3.0,
            effective_length_loss: 0.25,
            source: DamageSource::Script,
        };
        apply_fracture_commands(&mut runtime.family, &[damage]);
        let old = runtime.family().bond_state(BondId(0)).unwrap().clone();
        runtime
            .apply_edit(RuntimeEdit::SetMaterial {
                voxels: vec![GridCoord::new(0, 0)],
                fracture_material: 1,
            })
            .unwrap();
        let new_state = runtime.family().bond_state(BondId(0)).unwrap();
        assert_eq!(new_state.health, old.health);
        assert_eq!(new_state.accumulated_damage, old.accumulated_damage);
    }

    #[test]
    fn bond_lineage_transfer_by_interface_overlap() {
        let asset = author_voxel_asset(input_from_rows(&["AB", "AB"], &[1, 2, 1, 2])).unwrap();
        let mut runtime = VoxelRuntime::instantiate(FxFamilyId(1), asset);
        let damage = FractureCommand {
            order_key: DeterministicOrderKey::new(
                1,
                1,
                runtime.family().id,
                FxActorId(0),
                CommandId(0),
            ),
            actor: FxActorId(0),
            target: FractureTarget::Bond(BondId(0)),
            health_loss: 4.0,
            effective_length_loss: 0.0,
            source: DamageSource::Script,
        };
        apply_fracture_commands(&mut runtime.family, &[damage]);
        runtime
            .apply_edit(RuntimeEdit::RemoveVoxels {
                voxels: vec![GridCoord::new(0, 0)],
            })
            .unwrap();
        assert_eq!(runtime.family().bond_state(BondId(0)).unwrap().health, 6.0);
        assert_eq!(
            runtime
                .family()
                .bond_state(BondId(0))
                .unwrap()
                .effective_length,
            1.0
        );
        assert_eq!(
            runtime
                .family()
                .bond_state(BondId(0))
                .unwrap()
                .accumulated_damage,
            2.0
        );
    }

    #[test]
    fn bond_lineage_transfer_splits_broken_interface_to_multiple_new_bonds() {
        let asset =
            author_voxel_asset(input_from_rows(&["AB", "AB", "AB"], &[1, 2, 1, 2, 1, 2])).unwrap();
        let mut runtime = VoxelRuntime::instantiate(FxFamilyId(1), asset);
        let damage = FractureCommand {
            order_key: DeterministicOrderKey::new(
                1,
                1,
                runtime.family().id,
                FxActorId(0),
                CommandId(0),
            ),
            actor: FxActorId(0),
            target: FractureTarget::Bond(BondId(0)),
            health_loss: 10.0,
            effective_length_loss: 1.0,
            source: DamageSource::Script,
        };
        apply_fracture_commands(&mut runtime.family, &[damage]);
        runtime
            .apply_edit(RuntimeEdit::RemoveVoxels {
                voxels: vec![GridCoord::new(0, 1)],
            })
            .unwrap();

        let inherited = runtime
            .family()
            .bond_states()
            .iter()
            .filter(|state| state.health == 0.0 && state.accumulated_damage > 0.0)
            .collect::<Vec<_>>();
        assert_eq!(inherited.len(), 2);
        assert!(
            inherited
                .iter()
                .all(|state| approx_eq(state.effective_length, 2.0 / 3.0))
        );
        assert!(
            inherited
                .iter()
                .all(|state| approx_eq(state.accumulated_damage, 10.0 / 3.0))
        );
        let total_accumulated = inherited
            .iter()
            .map(|state| state.accumulated_damage)
            .sum::<f32>();
        assert!(approx_eq(total_accumulated, 20.0 / 3.0));
    }

    #[test]
    fn bond_lineage_tie_break_prefers_lowest_old_bond_id() {
        let best = better_bond_lineage_candidate(Some((2, BondId(4))), (2, BondId(1)));
        assert_eq!(best, Some((2, BondId(1))));
        let best = better_bond_lineage_candidate(best, (1, BondId(0)));
        assert_eq!(best, Some((2, BondId(1))));
        let best = better_bond_lineage_candidate(best, (3, BondId(9)));
        assert_eq!(best, Some((3, BondId(9))));
        let best = better_bond_lineage_candidate(best, (0, BondId(0)));
        assert_eq!(best, Some((3, BondId(9))));
    }

    #[test]
    fn runtime_edit_revalidates_exact_cover() {
        let asset = author_voxel_asset(input_from_rows(&["##"], &[1, 1])).unwrap();
        let mut runtime = VoxelRuntime::instantiate(FxFamilyId(1), asset);
        let report = runtime
            .apply_edit(RuntimeEdit::RemoveVoxels {
                voxels: vec![GridCoord::new(0, 0)],
            })
            .unwrap();
        assert!(report.exact_cover_validated);
        runtime.asset().validate_exact_cover().unwrap();
    }

    #[test]
    fn repair_new_interface_gets_initial_health() {
        let asset = author_voxel_asset(input_from_rows(&["A.B"], &[1, 0, 2])).unwrap();
        let mut runtime = VoxelRuntime::instantiate(FxFamilyId(1), asset);
        runtime
            .apply_edit(RuntimeEdit::AddVoxels {
                actor: FxActorId(0),
                voxels: vec![VoxelAdd {
                    coord: GridCoord::new(1, 0),
                    fracture_material: 1,
                    contact_material: 0,
                    external_id: 0,
                    orientation: None,
                }],
            })
            .unwrap();
        let fresh = runtime
            .family()
            .bond_states()
            .iter()
            .any(|state| state.health == 10.0 && state.accumulated_damage == 0.0);
        assert!(fresh);
    }

    #[test]
    fn repair_preserves_unaffected_region_and_dirty_membership() {
        let asset = author_voxel_asset(input_from_rows(&["AB..AB"], &[1, 2, 0, 0, 1, 2])).unwrap();
        let mut runtime = VoxelRuntime::instantiate(FxFamilyId(1), asset);
        let seed_before = runtime
            .asset()
            .core()
            .node(SupportNodeId(2))
            .unwrap()
            .stable_seed;
        let owner_before = runtime.family().node_owner(SupportNodeId(2));
        let state_before = runtime
            .family()
            .node_state(SupportNodeId(2))
            .cloned()
            .unwrap();
        let damage = FractureCommand {
            order_key: DeterministicOrderKey::new(
                1,
                1,
                runtime.family().id,
                FxActorId(1),
                CommandId(0),
            ),
            actor: FxActorId(1),
            target: FractureTarget::Bond(BondId(1)),
            health_loss: 4.0,
            effective_length_loss: 0.0,
            source: DamageSource::Script,
        };
        apply_fracture_commands(&mut runtime.family, &[damage]);
        let right_bond_after_damage = runtime.family().bond_state(BondId(1)).unwrap().clone();
        let report = runtime
            .apply_edit(RuntimeEdit::AddVoxels {
                actor: FxActorId(0),
                voxels: vec![VoxelAdd {
                    coord: GridCoord::new(2, 0),
                    fracture_material: 1,
                    contact_material: 0,
                    external_id: 0,
                    orientation: None,
                }],
            })
            .unwrap();
        assert!(report.unaffected_region_preserved);
        assert!(runtime.family().is_dirty(FxActorId(1)));
        assert!(report.preserved_dirty_actors.contains(&FxActorId(1)));
        assert!(report.unchanged_nodes.contains(&SupportNodeId(2)));
        assert!(report.unchanged_actors.contains(&FxActorId(1)));
        let unchanged_right_bond = report
            .unchanged_bonds
            .iter()
            .find(|proof| proof.old_bond == BondId(1))
            .unwrap();
        assert_eq!(
            runtime
                .family()
                .bond_state(unchanged_right_bond.new_bond)
                .unwrap(),
            &right_bond_after_damage
        );
        assert_eq!(
            runtime
                .asset()
                .core()
                .node(SupportNodeId(2))
                .unwrap()
                .stable_seed,
            seed_before
        );
        assert_eq!(runtime.family().node_owner(SupportNodeId(2)), owner_before);
        assert_eq!(
            runtime.family().node_state(SupportNodeId(2)).unwrap(),
            &state_before
        );
    }

    #[test]
    fn runtime_edit_preserves_unrelated_dirty_actor_for_later_split() {
        let mut input = input_from_rows(&["#..###"], &[1, 0, 0, 1, 1, 1]);
        input.support_node_hint = Some(vec![Some(0), None, None, Some(1), Some(2), Some(3)]);
        let asset = author_voxel_asset(input).unwrap();
        let mut runtime = VoxelRuntime::instantiate(FxFamilyId(1), asset);
        assert_eq!(runtime.family().actor_count(), 2);

        let break_right_actor = FractureCommand {
            order_key: DeterministicOrderKey::new(
                1,
                1,
                runtime.family().id,
                FxActorId(1),
                CommandId(0),
            ),
            actor: FxActorId(1),
            target: FractureTarget::Bond(BondId(0)),
            health_loss: 10.0,
            effective_length_loss: 1.0,
            source: DamageSource::Script,
        };
        apply_fracture_commands(&mut runtime.family, &[break_right_actor]);
        assert!(runtime.family().is_dirty(FxActorId(1)));

        let report = runtime
            .apply_edit(RuntimeEdit::AddVoxels {
                actor: FxActorId(0),
                voxels: vec![VoxelAdd {
                    coord: GridCoord::new(1, 0),
                    fracture_material: 1,
                    contact_material: 0,
                    external_id: 12,
                    orientation: None,
                }],
            })
            .unwrap();

        assert!(report.unaffected_region_preserved);
        assert!(report.unchanged_actors.contains(&FxActorId(1)));
        assert!(report.preserved_dirty_actors.contains(&FxActorId(1)));
        assert!(runtime.family().is_dirty(FxActorId(1)));

        let split_events = runtime.split_dirty_actors();
        assert_eq!(split_events.len(), 1);
        assert_eq!(split_events[0].parent_actor, FxActorId(1));
        assert!(!runtime.family().is_dirty(FxActorId(1)));
    }

    #[allow(dead_code)]
    fn _edge_set(edges: &[InterfaceEdge]) -> BTreeSet<InterfaceEdge> {
        edges.iter().copied().collect()
    }
}
