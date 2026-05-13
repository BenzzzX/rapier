//! Engine-neutral 2D voxel fracture core.
//!
//! This crate intentionally has no Rapier or Parry dependency. It implements
//! the Phase 1 in-memory loop: asset fixture -> family/actor -> bond health ->
//! damage/stress generate -> apply -> split -> deterministic digest.

use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet, VecDeque};

use thiserror::Error;

pub mod replay;
pub mod snapshot;
pub use replay::{ReplayCommand, ReplayError, ReplayTickTrace, run_replay_ticks};
pub use snapshot::{FxCoreSnapshotError, SnapshotMode};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

impl Vec2 {
    pub const ZERO: Self = Self { x: 0.0, y: 0.0 };

    pub const fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    pub fn dot(self, rhs: Self) -> f32 {
        self.x * rhs.x + self.y * rhs.y
    }

    pub fn perp(self) -> Self {
        Self::new(-self.y, self.x)
    }

    pub fn length(self) -> f32 {
        self.dot(self).sqrt()
    }

    pub fn normalized_or_zero(self) -> Self {
        let len = self.length();
        if len > 0.0 {
            Self::new(self.x / len, self.y / len)
        } else {
            Self::ZERO
        }
    }
}

impl std::ops::Add for Vec2 {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.x + rhs.x, self.y + rhs.y)
    }
}

impl std::ops::AddAssign for Vec2 {
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}

impl std::ops::Sub for Vec2 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(self.x - rhs.x, self.y - rhs.y)
    }
}

impl std::ops::Mul<f32> for Vec2 {
    type Output = Self;

    fn mul(self, rhs: f32) -> Self::Output {
        Self::new(self.x * rhs, self.y * rhs)
    }
}

macro_rules! id_type {
    ($name:ident) => {
        #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
        pub struct $name(pub u32);

        impl $name {
            pub const fn new(value: u32) -> Self {
                Self(value)
            }
        }
    };
}

id_type!(FxAssetId);
id_type!(FxFamilyId);
id_type!(FxActorId);
id_type!(SupportNodeId);
id_type!(ChunkId);
id_type!(BondId);
id_type!(ConnectionId);
id_type!(ExternalBondId);
id_type!(ExternalTargetToken);
id_type!(CommandId);
id_type!(EventId);

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct GridCoord {
    pub x: u32,
    pub y: u32,
}

impl GridCoord {
    pub const fn new(x: u32, y: u32) -> Self {
        Self { x, y }
    }

    pub fn center(self, voxel_size: f32) -> Vec2 {
        Vec2::new(
            (self.x as f32 + 0.5) * voxel_size,
            (self.y as f32 + 0.5) * voxel_size,
        )
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct LatticePoint {
    pub x: u32,
    pub y: u32,
}

impl LatticePoint {
    pub const fn new(x: u32, y: u32) -> Self {
        Self { x, y }
    }

    pub fn to_vec2(self, voxel_size: f32) -> Vec2 {
        Vec2::new(self.x as f32 * voxel_size, self.y as f32 * voxel_size)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct InterfaceEdge {
    pub start: LatticePoint,
    pub end: LatticePoint,
}

impl InterfaceEdge {
    pub fn new(a: LatticePoint, b: LatticePoint) -> Self {
        if a <= b {
            Self { start: a, end: b }
        } else {
            Self { start: b, end: a }
        }
    }

    fn midpoint(self, voxel_size: f32) -> Vec2 {
        (self.start.to_vec2(voxel_size) + self.end.to_vec2(voxel_size)) * 0.5
    }

    fn touches(self, rhs: Self) -> bool {
        self.start == rhs.start
            || self.start == rhs.end
            || self.end == rhs.start
            || self.end == rhs.end
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct DenseOccupancy {
    width: u32,
    height: u32,
    cells: Vec<bool>,
}

impl DenseOccupancy {
    pub fn new(width: u32, height: u32, cells: Vec<bool>) -> Result<Self, ValidationError> {
        let expected = width as usize * height as usize;
        if cells.len() != expected {
            return Err(ValidationError::GridSizeMismatch {
                expected,
                actual: cells.len(),
            });
        }
        Ok(Self {
            width,
            height,
            cells,
        })
    }

    pub fn from_rows(rows: &[&str]) -> Result<Self, ValidationError> {
        let height = rows.len() as u32;
        let width = rows.first().map_or(0, |row| row.len()) as u32;
        let mut cells = Vec::with_capacity(width as usize * height as usize);
        for row in rows {
            if row.len() as u32 != width {
                return Err(ValidationError::RaggedRows);
            }
            for byte in row.bytes() {
                cells.push(match byte {
                    b'#' | b'1' | b'X' => true,
                    b'.' | b'0' | b' ' => false,
                    other => {
                        return Err(ValidationError::InvalidOccupancyByte(other));
                    }
                });
            }
        }
        Self::new(width, height, cells)
    }

    pub fn width(&self) -> u32 {
        self.width
    }

    pub fn height(&self) -> u32 {
        self.height
    }

    pub fn contains(&self, coord: GridCoord) -> bool {
        coord.x < self.width && coord.y < self.height
    }

    pub fn is_occupied(&self, coord: GridCoord) -> bool {
        self.contains(coord) && self.cells[self.index(coord)]
    }

    fn index(&self, coord: GridCoord) -> usize {
        coord.y as usize * self.width as usize + coord.x as usize
    }

    pub fn occupied_voxels(&self) -> impl Iterator<Item = GridCoord> + '_ {
        (0..self.height).flat_map(move |y| {
            (0..self.width).filter_map(move |x| {
                let coord = GridCoord::new(x, y);
                self.is_occupied(coord).then_some(coord)
            })
        })
    }

    pub fn cell_count(&self) -> usize {
        self.cells.len()
    }

    pub fn index_of(&self, coord: GridCoord) -> Option<usize> {
        self.contains(coord).then(|| self.index(coord))
    }

    pub fn cells(&self) -> &[bool] {
        &self.cells
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct SupportNode2D {
    pub id: SupportNodeId,
    /// The chunk that owns this node in the support hierarchy. This is a
    /// support chunk id, not necessarily a visible leaf chunk id.
    pub chunk_id: ChunkId,
    pub voxels: Vec<GridCoord>,
    pub material_id: u16,
    pub orientation_summary: Option<u16>,
    pub anisotropy_axis: Vec2,
    pub stable_seed: u64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Chunk2D {
    pub id: ChunkId,
    /// Non-empty for support chunks. Empty chunks are visible or grouping
    /// chunks and do not participate in support exact-cover validation.
    pub support_nodes: Vec<SupportNodeId>,
    pub parent: Option<ChunkId>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Bond2D {
    pub id: BondId,
    pub node_a: SupportNodeId,
    pub node_b: SupportNodeId,
    pub centroid: Vec2,
    pub normal: Vec2,
    pub tangent: Vec2,
    pub length: f32,
    pub base_health: f32,
    pub tension_limit: f32,
    pub shear_limit: f32,
    pub material_pair: (u16, u16),
    pub interface_edges: Vec<InterfaceEdge>,
}

impl Bond2D {
    pub fn other(&self, node: SupportNodeId) -> Option<SupportNodeId> {
        if self.node_a == node {
            Some(self.node_b)
        } else if self.node_b == node {
            Some(self.node_a)
        } else {
            None
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct FxAsset {
    id: FxAssetId,
    voxel_size: f32,
    occupancy: DenseOccupancy,
    support_nodes: Vec<SupportNode2D>,
    chunks: Vec<Chunk2D>,
    internal_bonds: Vec<Bond2D>,
    voxel_to_node: Vec<Option<SupportNodeId>>,
}

#[derive(Clone, Debug)]
pub struct FxAssetDesc {
    pub id: FxAssetId,
    pub voxel_size: f32,
    pub occupancy: DenseOccupancy,
    pub support_node_map: Vec<Option<SupportNodeId>>,
    pub authored_chunks: Option<Vec<Chunk2D>>,
    pub material_map: Option<Vec<u16>>,
    pub orientation_map: Option<Vec<u16>>,
    pub default_bond_health: f32,
    pub default_tension_limit: f32,
    pub default_shear_limit: f32,
}

impl FxAssetDesc {
    pub fn new(
        id: FxAssetId,
        voxel_size: f32,
        occupancy: DenseOccupancy,
        support_node_map: Vec<Option<SupportNodeId>>,
    ) -> Self {
        Self {
            id,
            voxel_size,
            occupancy,
            support_node_map,
            authored_chunks: None,
            material_map: None,
            orientation_map: None,
            default_bond_health: 1.0,
            default_tension_limit: 10.0,
            default_shear_limit: 10.0,
        }
    }
}

impl FxAsset {
    pub fn id(&self) -> FxAssetId {
        self.id
    }

    pub fn voxel_size(&self) -> f32 {
        self.voxel_size
    }

    pub fn occupancy(&self) -> &DenseOccupancy {
        &self.occupancy
    }

    pub fn support_nodes(&self) -> &[SupportNode2D] {
        &self.support_nodes
    }

    pub fn chunks(&self) -> &[Chunk2D] {
        &self.chunks
    }

    pub fn internal_bonds(&self) -> &[Bond2D] {
        &self.internal_bonds
    }

    pub fn voxel_to_node_map(&self) -> &[Option<SupportNodeId>] {
        &self.voxel_to_node
    }

    pub fn from_desc(desc: FxAssetDesc) -> Result<Self, ValidationError> {
        if desc.voxel_size <= 0.0 {
            return Err(ValidationError::InvalidVoxelSize);
        }

        let cell_count = desc.occupancy.width as usize * desc.occupancy.height as usize;
        if desc.support_node_map.len() != cell_count {
            return Err(ValidationError::GridSizeMismatch {
                expected: cell_count,
                actual: desc.support_node_map.len(),
            });
        }
        if let Some(material_map) = &desc.material_map {
            if material_map.len() != cell_count {
                return Err(ValidationError::GridSizeMismatch {
                    expected: cell_count,
                    actual: material_map.len(),
                });
            }
        }
        if let Some(orientation_map) = &desc.orientation_map {
            if orientation_map.len() != cell_count {
                return Err(ValidationError::GridSizeMismatch {
                    expected: cell_count,
                    actual: orientation_map.len(),
                });
            }
        }

        let mut grouped: BTreeMap<SupportNodeId, Vec<GridCoord>> = BTreeMap::new();
        for y in 0..desc.occupancy.height {
            for x in 0..desc.occupancy.width {
                let coord = GridCoord::new(x, y);
                let idx = desc.occupancy.index(coord);
                match (desc.occupancy.cells[idx], desc.support_node_map[idx]) {
                    (true, Some(node)) => {
                        grouped.entry(node).or_default().push(coord);
                    }
                    (true, None) => {
                        return Err(ValidationError::MissingSupportCoverage(coord));
                    }
                    (false, Some(node)) => {
                        return Err(ValidationError::EmptyVoxelCovered { coord, node });
                    }
                    (false, None) => {}
                }
            }
        }

        let mut support_nodes = Vec::with_capacity(grouped.len());
        for (node_id, mut voxels) in grouped {
            voxels.sort_unstable();
            validate_four_neighbor_connected(&voxels)?;
            let material_id =
                dominant_material(&voxels, &desc.occupancy, desc.material_map.as_ref());
            let orientation_summary =
                dominant_orientation(&voxels, &desc.occupancy, desc.orientation_map.as_ref());
            let anisotropy_axis = orientation_summary
                .map(angle16_to_axis)
                .unwrap_or(Vec2::new(1.0, 0.0));
            let stable_seed = stable_node_seed(node_id, &voxels, material_id, orientation_summary);
            support_nodes.push(SupportNode2D {
                id: node_id,
                chunk_id: ChunkId(node_id.0),
                voxels,
                material_id,
                orientation_summary,
                anisotropy_axis,
                stable_seed,
            });
        }
        let mut chunks = desc.authored_chunks.unwrap_or_else(|| {
            support_nodes
                .iter()
                .map(|node| Chunk2D {
                    id: ChunkId(node.id.0),
                    support_nodes: vec![node.id],
                    parent: None,
                })
                .collect()
        });
        normalize_chunks(&mut chunks);
        let node_ids = support_nodes
            .iter()
            .map(|node| node.id)
            .collect::<BTreeSet<_>>();
        let support_chunks = validate_chunk_hierarchy(&chunks, &node_ids)?;
        for node in &mut support_nodes {
            node.chunk_id = support_chunks
                .get(&node.id)
                .copied()
                .ok_or(ValidationError::ChunkHierarchyNotExact)?;
        }

        let mut asset = Self {
            id: desc.id,
            voxel_size: desc.voxel_size,
            occupancy: desc.occupancy,
            support_nodes,
            chunks,
            internal_bonds: Vec::new(),
            voxel_to_node: desc.support_node_map,
        };
        asset.internal_bonds = asset.generate_internal_bonds(
            desc.default_bond_health,
            desc.default_tension_limit,
            desc.default_shear_limit,
        )?;
        asset.validate()?;
        Ok(asset)
    }

    pub fn node_at(&self, coord: GridCoord) -> Option<SupportNodeId> {
        self.occupancy
            .contains(coord)
            .then(|| self.voxel_to_node[self.occupancy.index(coord)])
            .flatten()
    }

    pub fn node(&self, id: SupportNodeId) -> Option<&SupportNode2D> {
        self.support_nodes
            .binary_search_by_key(&id, |node| node.id)
            .ok()
            .map(|idx| &self.support_nodes[idx])
    }

    pub fn bond(&self, id: BondId) -> Option<&Bond2D> {
        self.internal_bonds
            .binary_search_by_key(&id, |bond| bond.id)
            .ok()
            .map(|idx| &self.internal_bonds[idx])
    }

    pub fn chunk(&self, id: ChunkId) -> Option<&Chunk2D> {
        self.chunks
            .binary_search_by_key(&id, |chunk| chunk.id)
            .ok()
            .map(|idx| &self.chunks[idx])
    }

    pub fn validate(&self) -> Result<(), ValidationError> {
        let mut covered =
            vec![None; self.occupancy.width as usize * self.occupancy.height as usize];
        let mut node_ids = BTreeSet::new();
        for node in &self.support_nodes {
            if !node_ids.insert(node.id) {
                return Err(ValidationError::DuplicateSupportNodeId(node.id));
            }
            if node.voxels.is_empty() {
                return Err(ValidationError::EmptySupportNode(node.id));
            }
            validate_four_neighbor_connected(&node.voxels)?;
            for &coord in &node.voxels {
                if !self.occupancy.is_occupied(coord) {
                    return Err(ValidationError::EmptyVoxelCovered {
                        coord,
                        node: node.id,
                    });
                }
                let slot = &mut covered[self.occupancy.index(coord)];
                if let Some(existing) = *slot {
                    return Err(ValidationError::OverlappingSupportCoverage {
                        coord,
                        first: existing,
                        second: node.id,
                    });
                }
                *slot = Some(node.id);
            }
        }
        for coord in self.occupancy.occupied_voxels() {
            if covered[self.occupancy.index(coord)].is_none() {
                return Err(ValidationError::MissingSupportCoverage(coord));
            }
        }
        let support_chunks = validate_chunk_hierarchy(&self.chunks, &node_ids)?;
        for node in &self.support_nodes {
            if support_chunks.get(&node.id) != Some(&node.chunk_id) {
                return Err(ValidationError::ChunkHierarchyNotExact);
            }
        }
        let mut bond_ids = BTreeSet::new();
        for (index, bond) in self.internal_bonds.iter().enumerate() {
            if !bond_ids.insert(bond.id) {
                return Err(ValidationError::DuplicateInternalBondId(bond.id));
            }
            let expected = BondId(index as u32);
            if bond.id != expected {
                return Err(ValidationError::NonContiguousInternalBondId {
                    expected,
                    actual: bond.id,
                });
            }
            if bond.node_a == bond.node_b {
                return Err(ValidationError::SelfBond(bond.node_a));
            }
            if bond.node_a > bond.node_b {
                return Err(ValidationError::NonCanonicalBondEndpoints(bond.id));
            }
            if !node_ids.contains(&bond.node_a) {
                return Err(ValidationError::BondEndpointMissing(bond.node_a));
            }
            if !node_ids.contains(&bond.node_b) {
                return Err(ValidationError::BondEndpointMissing(bond.node_b));
            }
            if !valid_vec2(bond.centroid)
                || !valid_runtime_scalar(bond.base_health)
                || !valid_runtime_scalar(bond.tension_limit)
                || !valid_runtime_scalar(bond.shear_limit)
            {
                return Err(ValidationError::InvalidBondScalar(bond.id));
            }
            if !bond.length.is_finite() || bond.length <= 0.0 {
                return Err(ValidationError::NonPositiveBondLength(bond.id));
            }
            validate_internal_bond_direction(bond)?;
            if bond.interface_edges.is_empty() {
                return Err(ValidationError::EmptyBondInterface(bond.id));
            }
        }
        Ok(())
    }

    fn generate_internal_bonds(
        &self,
        base_health: f32,
        tension_limit: f32,
        shear_limit: f32,
    ) -> Result<Vec<Bond2D>, ValidationError> {
        let mut grouped: BTreeMap<
            (SupportNodeId, SupportNodeId),
            Vec<impl_edge_sample::EdgeSample>,
        > = BTreeMap::new();
        for y in 0..self.occupancy.height {
            for x in 0..self.occupancy.width {
                let here = GridCoord::new(x, y);
                let Some(here_node) = self.node_at(here) else {
                    continue;
                };
                if x + 1 < self.occupancy.width {
                    let there = GridCoord::new(x + 1, y);
                    if let Some(there_node) = self.node_at(there) {
                        self.record_interface_edge(
                            &mut grouped,
                            here_node,
                            there_node,
                            InterfaceEdge::new(
                                LatticePoint::new(x + 1, y),
                                LatticePoint::new(x + 1, y + 1),
                            ),
                            Vec2::new(1.0, 0.0),
                        );
                    }
                }
                if y + 1 < self.occupancy.height {
                    let there = GridCoord::new(x, y + 1);
                    if let Some(there_node) = self.node_at(there) {
                        self.record_interface_edge(
                            &mut grouped,
                            here_node,
                            there_node,
                            InterfaceEdge::new(
                                LatticePoint::new(x, y + 1),
                                LatticePoint::new(x + 1, y + 1),
                            ),
                            Vec2::new(0.0, 1.0),
                        );
                    }
                }
            }
        }

        let mut bonds = Vec::new();
        for ((node_a, node_b), mut samples) in grouped {
            samples.sort_by_key(|sample| sample.edge);
            for island in split_edge_islands(&samples) {
                let mut centroid = Vec2::ZERO;
                let mut normal = Vec2::ZERO;
                let mut edges = Vec::with_capacity(island.len());
                for sample in island {
                    centroid += sample.edge.midpoint(self.voxel_size);
                    normal += sample.normal;
                    edges.push(sample.edge);
                }
                centroid = centroid * (1.0 / edges.len() as f32);
                normal = normal.normalized_or_zero();
                let tangent = normal.perp();
                let material_pair = (
                    self.node(node_a)
                        .ok_or(ValidationError::BondEndpointMissing(node_a))?
                        .material_id,
                    self.node(node_b)
                        .ok_or(ValidationError::BondEndpointMissing(node_b))?
                        .material_id,
                );
                bonds.push(Bond2D {
                    id: BondId(bonds.len() as u32),
                    node_a,
                    node_b,
                    centroid,
                    normal,
                    tangent,
                    length: edges.len() as f32 * self.voxel_size,
                    base_health,
                    tension_limit,
                    shear_limit,
                    material_pair,
                    interface_edges: edges,
                });
            }
        }
        Ok(bonds)
    }

    fn record_interface_edge(
        &self,
        grouped: &mut BTreeMap<(SupportNodeId, SupportNodeId), Vec<impl_edge_sample::EdgeSample>>,
        a: SupportNodeId,
        b: SupportNodeId,
        edge: InterfaceEdge,
        normal_a_to_b: Vec2,
    ) {
        if a == b {
            return;
        }
        let (node_a, node_b, normal) = if a < b {
            (a, b, normal_a_to_b)
        } else {
            (b, a, normal_a_to_b * -1.0)
        };
        grouped
            .entry((node_a, node_b))
            .or_default()
            .push(impl_edge_sample::EdgeSample { edge, normal });
    }
}

mod impl_edge_sample {
    use super::{InterfaceEdge, Vec2};

    #[derive(Clone, Copy)]
    pub(super) struct EdgeSample {
        pub(super) edge: InterfaceEdge,
        pub(super) normal: Vec2,
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct BondRuntimeState {
    pub health: f32,
    pub effective_length: f32,
    pub accumulated_damage: f32,
}

impl BondRuntimeState {
    pub fn is_broken(&self) -> bool {
        self.health <= 0.0 || self.effective_length <= 0.000_001
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum ExternalTargetKind {
    World,
    Static,
    Kinematic,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ExternalTarget2D {
    pub kind: ExternalTargetKind,
    pub token: ExternalTargetToken,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DynamicConnectionPolicy {
    GraphOnly,
    CustomHardConstraint,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ExternalBond2D {
    pub id: ExternalBondId,
    pub node: SupportNodeId,
    pub target: ExternalTarget2D,
    pub anchor: Vec2,
    pub normal: Vec2,
    pub tangent: Vec2,
    pub base_health: f32,
    pub tension_limit: f32,
    pub shear_limit: f32,
    pub runtime: BondRuntimeState,
}

#[derive(Clone, Debug, PartialEq)]
pub struct DynamicStructuralBond2D {
    pub id: ConnectionId,
    pub node_a: SupportNodeId,
    pub node_b: SupportNodeId,
    pub policy: DynamicConnectionPolicy,
    pub centroid: Vec2,
    pub normal: Vec2,
    pub tangent: Vec2,
    pub base_health: f32,
    pub tension_limit: f32,
    pub shear_limit: f32,
    pub runtime: BondRuntimeState,
}

#[derive(Clone, Debug, PartialEq)]
pub struct StaticAnchorDesc {
    pub id: ExternalBondId,
    pub node: SupportNodeId,
    pub target: ExternalTarget2D,
    pub anchor: Vec2,
    pub normal: Vec2,
    pub health: f32,
    pub effective_length: f32,
    pub tension_limit: f32,
    pub shear_limit: f32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct DynamicStructuralBondDesc {
    pub id: ConnectionId,
    pub node_a: SupportNodeId,
    pub node_b: SupportNodeId,
    pub centroid: Vec2,
    pub normal: Vec2,
    pub health: f32,
    pub effective_length: f32,
    pub tension_limit: f32,
    pub shear_limit: f32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct MergeActorsResult {
    pub kept_actor: FxActorId,
    pub removed_actor: FxActorId,
    pub owned_nodes: Vec<SupportNodeId>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct NodeRuntimeState {
    pub health: f32,
    pub accumulated_damage: f32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ChunkRuntimeState {
    pub health: f32,
    pub accumulated_damage: f32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct FxActor {
    pub id: FxActorId,
    pub owned_nodes: Vec<SupportNodeId>,
    pub mass: f32,
    pub local_com: Vec2,
    pub inertia: f32,
    pub bounds: Option<GridAabb>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GridAabb {
    pub min: GridCoord,
    pub max: GridCoord,
}

#[derive(Clone, Debug, PartialEq)]
pub struct FxFamily {
    pub id: FxFamilyId,
    asset: FxAsset,
    actors: BTreeMap<FxActorId, FxActor>,
    node_owner: BTreeMap<SupportNodeId, FxActorId>,
    node_states: BTreeMap<SupportNodeId, NodeRuntimeState>,
    chunk_states: BTreeMap<ChunkId, ChunkRuntimeState>,
    bond_states: Vec<BondRuntimeState>,
    external_bonds: BTreeMap<ExternalBondId, ExternalBond2D>,
    dynamic_structural_bonds: BTreeMap<ConnectionId, DynamicStructuralBond2D>,
    dirty_actors: BTreeSet<FxActorId>,
    next_actor_id: u32,
    next_event_id: u32,
}

impl FxFamily {
    pub fn instantiate(id: FxFamilyId, asset: FxAsset) -> Self {
        let all_nodes: Vec<_> = asset.support_nodes.iter().map(|node| node.id).collect();
        let bond_states: Vec<_> = asset
            .internal_bonds
            .iter()
            .map(|bond| BondRuntimeState {
                health: bond.base_health,
                effective_length: bond.length,
                accumulated_damage: 0.0,
            })
            .collect();
        let node_states = asset
            .support_nodes
            .iter()
            .map(|node| {
                (
                    node.id,
                    NodeRuntimeState {
                        health: 1.0,
                        accumulated_damage: 0.0,
                    },
                )
            })
            .collect();
        let chunk_states = asset
            .chunks
            .iter()
            .map(|chunk| (chunk.id, initial_chunk_state()))
            .collect();

        let mut family = Self {
            id,
            asset,
            actors: BTreeMap::new(),
            node_owner: BTreeMap::new(),
            node_states,
            chunk_states,
            bond_states,
            external_bonds: BTreeMap::new(),
            dynamic_structural_bonds: BTreeMap::new(),
            dirty_actors: BTreeSet::new(),
            next_actor_id: 0,
            next_event_id: 0,
        };

        let initial_actor = FxActor {
            id: FxActorId(u32::MAX),
            owned_nodes: all_nodes,
            mass: 0.0,
            local_com: Vec2::ZERO,
            inertia: 0.0,
            bounds: None,
        };
        let components = actor_components(&initial_actor, &family);
        for component in components {
            let actor_id = FxActorId(family.next_actor_id);
            family.next_actor_id += 1;
            let actor = build_actor(actor_id, &component, &family.asset);
            for &node in &component {
                family.node_owner.insert(node, actor_id);
            }
            family.actors.insert(actor_id, actor);
        }
        family
    }

    pub fn asset(&self) -> &FxAsset {
        &self.asset
    }

    pub fn actors(&self) -> impl Iterator<Item = (&FxActorId, &FxActor)> {
        self.actors.iter()
    }

    pub fn actor_count(&self) -> usize {
        self.actors.len()
    }

    pub fn node_owner(&self, node: SupportNodeId) -> Option<FxActorId> {
        self.node_owner.get(&node).copied()
    }

    pub fn bond_states(&self) -> &[BondRuntimeState] {
        &self.bond_states
    }

    pub fn external_bonds(&self) -> impl Iterator<Item = (&ExternalBondId, &ExternalBond2D)> {
        self.external_bonds.iter()
    }

    pub fn external_bond(&self, id: ExternalBondId) -> Option<&ExternalBond2D> {
        self.external_bonds.get(&id)
    }

    pub fn dynamic_structural_bonds(
        &self,
    ) -> impl Iterator<Item = (&ConnectionId, &DynamicStructuralBond2D)> {
        self.dynamic_structural_bonds.iter()
    }

    pub fn dynamic_structural_bond(&self, id: ConnectionId) -> Option<&DynamicStructuralBond2D> {
        self.dynamic_structural_bonds.get(&id)
    }

    pub fn node_state(&self, node: SupportNodeId) -> Option<&NodeRuntimeState> {
        self.node_states.get(&node)
    }

    pub fn node_states(&self) -> impl Iterator<Item = (SupportNodeId, &NodeRuntimeState)> + '_ {
        self.node_states.iter().map(|(node, state)| (*node, state))
    }

    pub fn chunk_state(&self, chunk: ChunkId) -> Option<&ChunkRuntimeState> {
        self.chunk_states.get(&chunk)
    }

    pub fn chunk_states(&self) -> impl Iterator<Item = (ChunkId, &ChunkRuntimeState)> + '_ {
        self.chunk_states
            .iter()
            .map(|(chunk, state)| (*chunk, state))
    }

    pub fn dirty_actors(&self) -> impl Iterator<Item = FxActorId> + '_ {
        self.dirty_actors.iter().copied()
    }

    pub fn is_dirty(&self, actor: FxActorId) -> bool {
        self.dirty_actors.contains(&actor)
    }

    #[cfg(test)]
    fn mark_actor_dirty_for_test(&mut self, actor: FxActorId) {
        self.dirty_actors.insert(actor);
    }

    #[cfg(test)]
    fn next_actor_id_for_test(&self) -> u32 {
        self.next_actor_id
    }

    #[cfg(test)]
    fn next_event_id_for_test(&self) -> u32 {
        self.next_event_id
    }

    pub fn actor(&self, id: FxActorId) -> Option<&FxActor> {
        self.actors.get(&id)
    }

    pub fn bond_state(&self, id: BondId) -> Option<&BondRuntimeState> {
        self.bond_states.get(id.0 as usize)
    }

    pub fn external_bond_state(&self, id: ExternalBondId) -> Option<&BondRuntimeState> {
        self.external_bonds.get(&id).map(|bond| &bond.runtime)
    }

    pub fn dynamic_structural_bond_state(&self, id: ConnectionId) -> Option<&BondRuntimeState> {
        self.dynamic_structural_bonds
            .get(&id)
            .map(|bond| &bond.runtime)
    }

    pub fn connect_static_anchor(
        &mut self,
        desc: StaticAnchorDesc,
    ) -> Result<ExternalBondId, ConnectionError> {
        self.validate_owned_node(desc.node)?;
        if self.external_bonds.contains_key(&desc.id) {
            return Err(ConnectionError::DuplicateExternalBond(desc.id));
        }
        validate_connection_scalars(
            desc.health,
            desc.effective_length,
            desc.tension_limit,
            desc.shear_limit,
        )
        .map_err(|_| ConnectionError::InvalidExternalBondRuntime(desc.id))?;
        let normal = validate_direction(desc.normal)
            .ok_or(ConnectionError::InvalidExternalBondRuntime(desc.id))?;
        let tangent = normal.perp();
        if !valid_vec2(desc.anchor) {
            return Err(ConnectionError::InvalidExternalBondRuntime(desc.id));
        }
        let id = desc.id;
        self.external_bonds.insert(
            id,
            ExternalBond2D {
                id,
                node: desc.node,
                target: desc.target,
                anchor: desc.anchor,
                normal,
                tangent,
                base_health: desc.health,
                tension_limit: desc.tension_limit,
                shear_limit: desc.shear_limit,
                runtime: BondRuntimeState {
                    health: desc.health,
                    effective_length: desc.effective_length,
                    accumulated_damage: 0.0,
                },
            },
        );
        Ok(id)
    }

    pub fn connect_dynamic_structural_bond_graph_only(
        &mut self,
        desc: DynamicStructuralBondDesc,
    ) -> Result<ConnectionId, ConnectionError> {
        self.validate_owned_node(desc.node_a)?;
        self.validate_owned_node(desc.node_b)?;
        if desc.node_a == desc.node_b {
            return Err(ConnectionError::SelfConnection(desc.node_a));
        }
        if self.dynamic_structural_bonds.contains_key(&desc.id) {
            return Err(ConnectionError::DuplicateConnection(desc.id));
        }
        validate_connection_scalars(
            desc.health,
            desc.effective_length,
            desc.tension_limit,
            desc.shear_limit,
        )
        .map_err(|_| ConnectionError::InvalidConnectionRuntime(desc.id))?;
        let normal = validate_direction(desc.normal)
            .ok_or(ConnectionError::InvalidConnectionRuntime(desc.id))?;
        if !valid_vec2(desc.centroid) {
            return Err(ConnectionError::InvalidConnectionRuntime(desc.id));
        }
        let tangent = normal.perp();
        let (node_a, node_b, normal, tangent) = if desc.node_a <= desc.node_b {
            (desc.node_a, desc.node_b, normal, tangent)
        } else {
            (desc.node_b, desc.node_a, normal * -1.0, tangent * -1.0)
        };
        let id = desc.id;
        self.dynamic_structural_bonds.insert(
            id,
            DynamicStructuralBond2D {
                id,
                node_a,
                node_b,
                policy: DynamicConnectionPolicy::GraphOnly,
                centroid: desc.centroid,
                normal,
                tangent,
                base_health: desc.health,
                tension_limit: desc.tension_limit,
                shear_limit: desc.shear_limit,
                runtime: BondRuntimeState {
                    health: desc.health,
                    effective_length: desc.effective_length,
                    accumulated_damage: 0.0,
                },
            },
        );
        Ok(id)
    }

    pub fn merge_actors(
        &mut self,
        actor_a: FxActorId,
        actor_b: FxActorId,
    ) -> Result<MergeActorsResult, ConnectionError> {
        if actor_a == actor_b {
            return Err(ConnectionError::SelfMerge(actor_a));
        }
        let actor_a_nodes = self
            .actors
            .get(&actor_a)
            .ok_or(ConnectionError::UnknownActor(actor_a))?
            .owned_nodes
            .clone();
        let actor_b_nodes = self
            .actors
            .get(&actor_b)
            .ok_or(ConnectionError::UnknownActor(actor_b))?
            .owned_nodes
            .clone();
        if !self.has_unbroken_graph_connection_between(&actor_a_nodes, &actor_b_nodes) {
            return Err(ConnectionError::MissingMergeConnection { actor_a, actor_b });
        }
        let (kept_actor, removed_actor, mut owned_nodes) = if actor_a <= actor_b {
            let mut nodes = actor_a_nodes;
            nodes.extend(actor_b_nodes);
            (actor_a, actor_b, nodes)
        } else {
            let mut nodes = actor_b_nodes;
            nodes.extend(actor_a_nodes);
            (actor_b, actor_a, nodes)
        };
        owned_nodes.sort_unstable();
        owned_nodes.dedup();
        for node in &owned_nodes {
            self.node_owner.insert(*node, kept_actor);
        }
        self.actors.insert(
            kept_actor,
            build_actor(kept_actor, &owned_nodes, &self.asset),
        );
        self.actors.remove(&removed_actor);
        self.dirty_actors.remove(&actor_a);
        self.dirty_actors.remove(&actor_b);
        if self
            .actors
            .get(&kept_actor)
            .is_some_and(|actor| actor_components(actor, self).len() > 1)
        {
            self.dirty_actors.insert(kept_actor);
        }
        Ok(MergeActorsResult {
            kept_actor,
            removed_actor,
            owned_nodes,
        })
    }

    fn has_unbroken_graph_connection_between(
        &self,
        actor_a_nodes: &[SupportNodeId],
        actor_b_nodes: &[SupportNodeId],
    ) -> bool {
        let actor_a_nodes = actor_a_nodes.iter().copied().collect::<BTreeSet<_>>();
        let actor_b_nodes = actor_b_nodes.iter().copied().collect::<BTreeSet<_>>();
        self.dynamic_structural_bonds.values().any(|bond| {
            bond.policy == DynamicConnectionPolicy::GraphOnly
                && !bond.runtime.is_broken()
                && ((actor_a_nodes.contains(&bond.node_a) && actor_b_nodes.contains(&bond.node_b))
                    || (actor_a_nodes.contains(&bond.node_b)
                        && actor_b_nodes.contains(&bond.node_a)))
        })
    }

    pub fn deterministic_state_digest(&self) -> u64 {
        let mut hasher = Fnva64::default();
        hasher.write_u32(self.id.0);
        hasher.write_u32(self.asset.id.0);
        hasher.write_u32(self.next_actor_id);
        hasher.write_u32(self.next_event_id);
        hasher.write_f32(self.asset.voxel_size);
        for node in &self.asset.support_nodes {
            hasher.write_u32(node.id.0);
            hasher.write_u32(node.chunk_id.0);
            hasher.write_u32(node.material_id as u32);
            hasher.write_u32(node.orientation_summary.map_or(u32::MAX, u32::from));
            hasher.write_f32(node.anisotropy_axis.x);
            hasher.write_f32(node.anisotropy_axis.y);
            hasher.write_u32((node.stable_seed & 0xffff_ffff) as u32);
            hasher.write_u32((node.stable_seed >> 32) as u32);
            for voxel in &node.voxels {
                hasher.write_u32(voxel.x);
                hasher.write_u32(voxel.y);
            }
        }
        for chunk in &self.asset.chunks {
            hasher.write_u32(chunk.id.0);
            hasher.write_u32(chunk.support_nodes.len() as u32);
            for node in &chunk.support_nodes {
                hasher.write_u32(node.0);
            }
            hasher.write_u32(chunk.parent.map_or(u32::MAX, |parent| parent.0));
        }
        for bond in &self.asset.internal_bonds {
            hasher.write_u32(bond.id.0);
            hasher.write_u32(bond.node_a.0);
            hasher.write_u32(bond.node_b.0);
            hasher.write_f32(bond.length);
            hasher.write_f32(bond.tension_limit);
            hasher.write_f32(bond.shear_limit);
            for edge in &bond.interface_edges {
                hasher.write_u32(edge.start.x);
                hasher.write_u32(edge.start.y);
                hasher.write_u32(edge.end.x);
                hasher.write_u32(edge.end.y);
            }
        }
        for (actor_id, actor) in &self.actors {
            hasher.write_u32(actor_id.0);
            for node in &actor.owned_nodes {
                hasher.write_u32(node.0);
            }
            hasher.write_f32(actor.mass);
            hasher.write_f32(actor.local_com.x);
            hasher.write_f32(actor.local_com.y);
            hasher.write_f32(actor.inertia);
            if let Some(bounds) = actor.bounds {
                hasher.write_u32(bounds.min.x);
                hasher.write_u32(bounds.min.y);
                hasher.write_u32(bounds.max.x);
                hasher.write_u32(bounds.max.y);
            } else {
                hasher.write_u32(u32::MAX);
                hasher.write_u32(u32::MAX);
                hasher.write_u32(u32::MAX);
                hasher.write_u32(u32::MAX);
            }
        }
        for (node, owner) in &self.node_owner {
            hasher.write_u32(node.0);
            hasher.write_u32(owner.0);
        }
        for (node, state) in &self.node_states {
            hasher.write_u32(node.0);
            hasher.write_f32(state.health);
            hasher.write_f32(state.accumulated_damage);
        }
        for (chunk, state) in &self.chunk_states {
            hasher.write_u32(chunk.0);
            hasher.write_f32(state.health);
            hasher.write_f32(state.accumulated_damage);
        }
        for state in &self.bond_states {
            hasher.write_f32(state.health);
            hasher.write_f32(state.effective_length);
            hasher.write_f32(state.accumulated_damage);
        }
        for (id, bond) in &self.external_bonds {
            hasher.write_u32(id.0);
            hasher.write_u32(bond.node.0);
            hasher.write_u32(match bond.target.kind {
                ExternalTargetKind::World => 0,
                ExternalTargetKind::Static => 1,
                ExternalTargetKind::Kinematic => 2,
            });
            hasher.write_u32(bond.target.token.0);
            hasher.write_f32(bond.anchor.x);
            hasher.write_f32(bond.anchor.y);
            hasher.write_f32(bond.normal.x);
            hasher.write_f32(bond.normal.y);
            hasher.write_f32(bond.tangent.x);
            hasher.write_f32(bond.tangent.y);
            hasher.write_f32(bond.base_health);
            hasher.write_f32(bond.tension_limit);
            hasher.write_f32(bond.shear_limit);
            hasher.write_f32(bond.runtime.health);
            hasher.write_f32(bond.runtime.effective_length);
            hasher.write_f32(bond.runtime.accumulated_damage);
        }
        for (id, bond) in &self.dynamic_structural_bonds {
            hasher.write_u32(id.0);
            hasher.write_u32(bond.node_a.0);
            hasher.write_u32(bond.node_b.0);
            hasher.write_u32(match bond.policy {
                DynamicConnectionPolicy::GraphOnly => 0,
                DynamicConnectionPolicy::CustomHardConstraint => 1,
            });
            hasher.write_f32(bond.centroid.x);
            hasher.write_f32(bond.centroid.y);
            hasher.write_f32(bond.normal.x);
            hasher.write_f32(bond.normal.y);
            hasher.write_f32(bond.tangent.x);
            hasher.write_f32(bond.tangent.y);
            hasher.write_f32(bond.base_health);
            hasher.write_f32(bond.tension_limit);
            hasher.write_f32(bond.shear_limit);
            hasher.write_f32(bond.runtime.health);
            hasher.write_f32(bond.runtime.effective_length);
            hasher.write_f32(bond.runtime.accumulated_damage);
        }
        for actor in &self.dirty_actors {
            hasher.write_u32(actor.0);
        }
        hasher.finish()
    }

    fn next_event_id(&mut self) -> EventId {
        let id = EventId(self.next_event_id);
        self.next_event_id += 1;
        id
    }

    fn mark_bond_dirty(&mut self, bond: &Bond2D) {
        if let Some(actor) = self.node_owner.get(&bond.node_a) {
            self.dirty_actors.insert(*actor);
        }
        if let Some(actor) = self.node_owner.get(&bond.node_b) {
            self.dirty_actors.insert(*actor);
        }
    }

    fn validate_owned_node(&self, node: SupportNodeId) -> Result<(), ConnectionError> {
        if self.asset.node(node).is_some() && self.node_owner.contains_key(&node) {
            Ok(())
        } else {
            Err(ConnectionError::UnknownNode(node))
        }
    }

    fn mark_connection_dirty(&mut self, node_a: SupportNodeId, node_b: SupportNodeId) {
        if let Some(actor) = self.node_owner.get(&node_a) {
            self.dirty_actors.insert(*actor);
        }
        if let Some(actor) = self.node_owner.get(&node_b) {
            self.dirty_actors.insert(*actor);
        }
    }

    pub fn apply_repair_plan(
        &mut self,
        plan: RepairPlan,
    ) -> Result<RepairCommitSummary, RepairError> {
        plan.asset.validate()?;

        let node_ids: BTreeSet<_> = plan
            .asset
            .support_nodes()
            .iter()
            .map(|node| node.id)
            .collect();
        let mut node_owner = BTreeMap::new();
        for (node, actor) in plan.node_owners {
            if !node_ids.contains(&node) {
                return Err(RepairError::UnknownNode(node));
            }
            if !self.actors.contains_key(&actor) {
                return Err(RepairError::MissingNodeOwnerActor { node, actor });
            }
            if node_owner.insert(node, actor).is_some() {
                return Err(RepairError::DuplicateNodeOwner(node));
            }
        }
        for node in &node_ids {
            if !node_owner.contains_key(node) {
                return Err(RepairError::MissingNodeOwner(*node));
            }
        }

        if plan.bond_states.len() != plan.asset.internal_bonds().len() {
            return Err(RepairError::BondStateCountMismatch {
                expected: plan.asset.internal_bonds().len(),
                actual: plan.bond_states.len(),
            });
        }

        let mut node_states = BTreeMap::new();
        for (node, state) in plan.node_states {
            if !node_ids.contains(&node) {
                return Err(RepairError::UnknownNode(node));
            }
            if !valid_runtime_scalar(state.health)
                || !valid_runtime_scalar(state.accumulated_damage)
            {
                return Err(RepairError::InvalidNodeState(node));
            }
            if node_states.insert(node, state).is_some() {
                return Err(RepairError::DuplicateNodeState(node));
            }
        }
        for node in &node_ids {
            if !node_states.contains_key(node) {
                return Err(RepairError::MissingNodeState(*node));
            }
        }
        self.validate_connection_endpoints_for_repair(&plan.asset, &node_owner)?;

        let mut grouped: BTreeMap<FxActorId, Vec<SupportNodeId>> = BTreeMap::new();
        for (node, actor) in &node_owner {
            grouped.entry(*actor).or_default().push(*node);
        }

        let mut actors = BTreeMap::new();
        for (actor, mut nodes) in grouped {
            nodes.sort_unstable();
            actors.insert(actor, build_actor(actor, &nodes, &plan.asset));
        }

        for (idx, state) in plan.bond_states.iter().enumerate() {
            if !valid_runtime_scalar(state.health)
                || !valid_runtime_scalar(state.effective_length)
                || !valid_runtime_scalar(state.accumulated_damage)
            {
                return Err(RepairError::InvalidBondState(BondId(idx as u32)));
            }
        }
        let chunk_states = reconcile_chunk_states(&self.chunk_states, &plan.asset);

        let mut dirty_actors = BTreeSet::new();
        for actor in plan.dirty_actors {
            if !actors.contains_key(&actor) {
                return Err(RepairError::UnknownDirtyActor(actor));
            }
            dirty_actors.insert(actor);
        }
        for (actor_id, old_actor) in &self.actors {
            let new_nodes = actors
                .get(actor_id)
                .map(|actor| actor.owned_nodes.as_slice())
                .unwrap_or(&[]);
            if old_actor.owned_nodes.as_slice() != new_nodes && !dirty_actors.contains(actor_id) {
                return Err(RepairError::ActorChangedNotDirty(*actor_id));
            }
        }
        let temp_family = Self {
            id: self.id,
            asset: plan.asset.clone(),
            actors: actors.clone(),
            node_owner: node_owner.clone(),
            node_states: node_states.clone(),
            chunk_states: chunk_states.clone(),
            bond_states: plan.bond_states.clone(),
            external_bonds: self.external_bonds.clone(),
            dynamic_structural_bonds: self.dynamic_structural_bonds.clone(),
            dirty_actors: dirty_actors.clone(),
            next_actor_id: self.next_actor_id,
            next_event_id: self.next_event_id,
        };
        for (actor_id, actor) in &temp_family.actors {
            if actor_components(actor, &temp_family).len() > 1 && !dirty_actors.contains(actor_id) {
                return Err(RepairError::DisconnectedActorNotDirty(*actor_id));
            }
        }
        let actor_order = actors.keys().copied().collect::<Vec<_>>();

        self.asset = plan.asset;
        self.actors = actors;
        self.node_owner = node_owner;
        self.node_states = node_states;
        self.chunk_states = chunk_states;
        self.bond_states = plan.bond_states;
        self.dirty_actors = dirty_actors;
        self.next_actor_id = self.next_actor_id.max(
            self.actors
                .keys()
                .map(|actor| actor.0 + 1)
                .max()
                .unwrap_or(0),
        );

        Ok(RepairCommitSummary {
            dirty_actors: self.dirty_actors.iter().copied().collect(),
            actor_order,
        })
    }

    fn validate_connection_endpoints_for_repair(
        &self,
        asset: &FxAsset,
        node_owner: &BTreeMap<SupportNodeId, FxActorId>,
    ) -> Result<(), RepairError> {
        for bond in self.external_bonds.values() {
            if asset.node(bond.node).is_none() || !node_owner.contains_key(&bond.node) {
                return Err(RepairError::StaleExternalBondEndpoint {
                    bond: bond.id,
                    node: bond.node,
                });
            }
        }
        for bond in self.dynamic_structural_bonds.values() {
            if asset.node(bond.node_a).is_none() || !node_owner.contains_key(&bond.node_a) {
                return Err(RepairError::StaleDynamicConnectionEndpoint {
                    connection: bond.id,
                    node: bond.node_a,
                });
            }
            if asset.node(bond.node_b).is_none() || !node_owner.contains_key(&bond.node_b) {
                return Err(RepairError::StaleDynamicConnectionEndpoint {
                    connection: bond.id,
                    node: bond.node_b,
                });
            }
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct RepairPlan {
    pub asset: FxAsset,
    pub node_owners: Vec<(SupportNodeId, FxActorId)>,
    pub node_states: Vec<(SupportNodeId, NodeRuntimeState)>,
    pub bond_states: Vec<BondRuntimeState>,
    pub dirty_actors: Vec<FxActorId>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct RepairCommitSummary {
    pub dirty_actors: Vec<FxActorId>,
    pub actor_order: Vec<FxActorId>,
}

#[derive(Error, Debug, Clone, PartialEq)]
pub enum RepairError {
    #[error(transparent)]
    Validation(#[from] ValidationError),
    #[error("repair plan is missing owner for node {0:?}")]
    MissingNodeOwner(SupportNodeId),
    #[error("repair plan references unknown node {0:?}")]
    UnknownNode(SupportNodeId),
    #[error("repair plan assigns node {node:?} to missing actor {actor:?}")]
    MissingNodeOwnerActor {
        node: SupportNodeId,
        actor: FxActorId,
    },
    #[error("repair plan assigns node {0:?} more than once")]
    DuplicateNodeOwner(SupportNodeId),
    #[error("repair plan is missing runtime state for node {0:?}")]
    MissingNodeState(SupportNodeId),
    #[error("repair plan provides node state for node {0:?} more than once")]
    DuplicateNodeState(SupportNodeId),
    #[error("repair plan has invalid runtime state for node {0:?}")]
    InvalidNodeState(SupportNodeId),
    #[error("repair plan has invalid runtime state for bond {0:?}")]
    InvalidBondState(BondId),
    #[error("repair plan marks unknown dirty actor {0:?}")]
    UnknownDirtyActor(FxActorId),
    #[error("repair plan changes actor {0:?} without marking it dirty")]
    ActorChangedNotDirty(FxActorId),
    #[error("repair plan leaves actor {0:?} disconnected without marking it dirty")]
    DisconnectedActorNotDirty(FxActorId),
    #[error("repair plan bond state count mismatch: expected {expected}, got {actual}")]
    BondStateCountMismatch { expected: usize, actual: usize },
    #[error("repair plan leaves external bond {bond:?} endpoint {node:?} stale")]
    StaleExternalBondEndpoint {
        bond: ExternalBondId,
        node: SupportNodeId,
    },
    #[error("repair plan leaves dynamic connection {connection:?} endpoint {node:?} stale")]
    StaleDynamicConnectionEndpoint {
        connection: ConnectionId,
        node: SupportNodeId,
    },
}

#[derive(Error, Debug, Clone, PartialEq)]
pub enum ConnectionError {
    #[error("connection references unknown node {0:?}")]
    UnknownNode(SupportNodeId),
    #[error("external bond id {0:?} is already active")]
    DuplicateExternalBond(ExternalBondId),
    #[error("dynamic connection id {0:?} is already active")]
    DuplicateConnection(ConnectionId),
    #[error("connection cannot connect support node {0:?} to itself")]
    SelfConnection(SupportNodeId),
    #[error("external bond {0:?} has invalid finite nonnegative runtime values")]
    InvalidExternalBondRuntime(ExternalBondId),
    #[error("dynamic connection {0:?} has invalid finite nonnegative runtime values")]
    InvalidConnectionRuntime(ConnectionId),
    #[error("merge references unknown actor {0:?}")]
    UnknownActor(FxActorId),
    #[error("actor {0:?} cannot be merged with itself")]
    SelfMerge(FxActorId),
    #[error("actors {actor_a:?} and {actor_b:?} have no unbroken graph connection to merge")]
    MissingMergeConnection {
        actor_a: FxActorId,
        actor_b: FxActorId,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct DeterministicOrderKey {
    pub tick: u64,
    pub source_priority: u16,
    pub family_id: FxFamilyId,
    pub actor_id: FxActorId,
    pub command_id: CommandId,
}

impl DeterministicOrderKey {
    pub const fn new(
        tick: u64,
        source_priority: u16,
        family_id: FxFamilyId,
        actor_id: FxActorId,
        command_id: CommandId,
    ) -> Self {
        Self {
            tick,
            source_priority,
            family_id,
            actor_id,
            command_id,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DamageSource {
    Script,
    ContactImpulse,
    JointFeedback,
    Stress,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum FractureTarget {
    Bond(BondId),
    Chunk(ChunkId),
    Node(SupportNodeId),
    ExternalBond(ExternalBondId),
    Connection(ConnectionId),
}

#[derive(Clone, Debug, PartialEq)]
pub struct DamageInput {
    pub order_key: DeterministicOrderKey,
    pub actor: FxActorId,
    pub target: FractureTarget,
    pub health_loss: f32,
    pub effective_length_loss: f32,
    pub source: DamageSource,
    pub position: Vec2,
    pub radius: f32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct FractureCommand {
    pub order_key: DeterministicOrderKey,
    pub actor: FxActorId,
    pub target: FractureTarget,
    pub health_loss: f32,
    pub effective_length_loss: f32,
    pub source: DamageSource,
}

impl FractureCommand {
    fn stable_cmp(&self, rhs: &Self) -> Ordering {
        self.order_key
            .cmp(&rhs.order_key)
            .then_with(|| self.target.cmp(&rhs.target))
            .then_with(|| self.health_loss.to_bits().cmp(&rhs.health_loss.to_bits()))
            .then_with(|| {
                self.effective_length_loss
                    .to_bits()
                    .cmp(&rhs.effective_length_loss.to_bits())
            })
    }
}

pub fn sort_fracture_commands(commands: &mut [FractureCommand]) {
    commands.sort_by(|a, b| a.stable_cmp(b));
}

pub fn generate_damage_commands(family: &FxFamily, inputs: &[DamageInput]) -> Vec<FractureCommand> {
    let mut sorted_inputs = inputs.to_vec();
    sorted_inputs.sort_by_key(|input| input.order_key);
    let mut out = Vec::new();
    for input in sorted_inputs {
        let health_loss = input.health_loss.max(0.0);
        let effective_length_loss = input.effective_length_loss.max(0.0);
        if health_loss == 0.0 && effective_length_loss == 0.0 {
            continue;
        }
        if !actor_owns_target(family, input.actor, input.target) {
            continue;
        }
        match input.target {
            FractureTarget::Bond(bond_id) => {
                if family.asset.bond(bond_id).is_some() {
                    out.push(FractureCommand {
                        order_key: input.order_key,
                        actor: input.actor,
                        target: input.target,
                        health_loss,
                        effective_length_loss,
                        source: input.source,
                    });
                }
            }
            FractureTarget::Chunk(chunk_id) => {
                if health_loss > 0.0 && family.chunk_states.contains_key(&chunk_id) {
                    out.push(FractureCommand {
                        order_key: input.order_key,
                        actor: input.actor,
                        target: input.target,
                        health_loss,
                        effective_length_loss,
                        source: input.source,
                    });
                }
            }
            FractureTarget::Node(node_id) => {
                for bond in &family.asset.internal_bonds {
                    let target = FractureTarget::Bond(bond.id);
                    if (bond.node_a == node_id || bond.node_b == node_id)
                        && actor_owns_target(family, input.actor, target)
                    {
                        out.push(FractureCommand {
                            order_key: input.order_key,
                            actor: input.actor,
                            target,
                            health_loss,
                            effective_length_loss,
                            source: input.source,
                        });
                    }
                }
                for bond in family.external_bonds.values() {
                    let target = FractureTarget::ExternalBond(bond.id);
                    if !bond.runtime.is_broken()
                        && bond.node == node_id
                        && actor_owns_target(family, input.actor, target)
                    {
                        out.push(FractureCommand {
                            order_key: input.order_key,
                            actor: input.actor,
                            target,
                            health_loss,
                            effective_length_loss,
                            source: input.source,
                        });
                    }
                }
                for bond in family.dynamic_structural_bonds.values() {
                    let target = FractureTarget::Connection(bond.id);
                    if !bond.runtime.is_broken()
                        && (bond.node_a == node_id || bond.node_b == node_id)
                        && actor_owns_target(family, input.actor, target)
                    {
                        out.push(FractureCommand {
                            order_key: input.order_key,
                            actor: input.actor,
                            target,
                            health_loss,
                            effective_length_loss,
                            source: input.source,
                        });
                    }
                }
            }
            FractureTarget::ExternalBond(bond_id) => {
                if family
                    .external_bonds
                    .get(&bond_id)
                    .is_some_and(|bond| !bond.runtime.is_broken())
                {
                    out.push(FractureCommand {
                        order_key: input.order_key,
                        actor: input.actor,
                        target: input.target,
                        health_loss,
                        effective_length_loss,
                        source: input.source,
                    });
                }
            }
            FractureTarget::Connection(connection_id) => {
                if family
                    .dynamic_structural_bonds
                    .get(&connection_id)
                    .is_some_and(|bond| !bond.runtime.is_broken())
                {
                    out.push(FractureCommand {
                        order_key: input.order_key,
                        actor: input.actor,
                        target: input.target,
                        health_loss,
                        effective_length_loss,
                        source: input.source,
                    });
                }
            }
        }
    }
    sort_fracture_commands(&mut out);
    out
}

#[derive(Clone, Debug, PartialEq)]
pub struct FractureEvent {
    pub event_id: EventId,
    pub order_key: DeterministicOrderKey,
    pub family: FxFamilyId,
    pub actor: FxActorId,
    pub target: FractureTarget,
    pub old_health: f32,
    pub new_health: f32,
    pub old_effective_length: f32,
    pub new_effective_length: f32,
    pub source: DamageSource,
}

pub fn apply_fracture_commands(
    family: &mut FxFamily,
    commands: &[FractureCommand],
) -> Vec<FractureEvent> {
    let mut sorted = commands.to_vec();
    sort_fracture_commands(&mut sorted);
    let mut events = Vec::new();
    for command in sorted {
        let health_loss = command.health_loss.max(0.0);
        let effective_length_loss = command.effective_length_loss.max(0.0);
        if health_loss == 0.0 && effective_length_loss == 0.0 {
            continue;
        }
        if !actor_owns_target(family, command.actor, command.target) {
            continue;
        }
        match command.target {
            FractureTarget::Bond(bond_id) => {
                let Some(bond) = family.asset.bond(bond_id).cloned() else {
                    continue;
                };
                let Some(state) = family.bond_states.get_mut(bond_id.0 as usize) else {
                    continue;
                };
                let old_health = state.health;
                let old_effective_length = state.effective_length;
                state.health = (state.health - health_loss).max(0.0);
                state.effective_length = (state.effective_length - effective_length_loss).max(0.0);
                state.accumulated_damage += health_loss;
                let new_health = state.health;
                let new_effective_length = state.effective_length;
                family.mark_bond_dirty(&bond);
                events.push(FractureEvent {
                    event_id: family.next_event_id(),
                    order_key: command.order_key,
                    family: family.id,
                    actor: command.actor,
                    target: command.target,
                    old_health,
                    new_health,
                    old_effective_length,
                    new_effective_length,
                    source: command.source,
                });
            }
            FractureTarget::Chunk(chunk_id) => {
                if health_loss == 0.0 {
                    continue;
                }
                let covered_nodes = chunk_target_support_nodes(&family.asset, chunk_id);
                let Some(state) = family.chunk_states.get_mut(&chunk_id) else {
                    continue;
                };
                let old_health = state.health;
                state.health = (state.health - health_loss).max(0.0);
                state.accumulated_damage += health_loss;
                let new_health = state.health;
                for node in covered_nodes {
                    if let Some(actor) = family.node_owner.get(&node) {
                        family.dirty_actors.insert(*actor);
                    }
                }
                events.push(FractureEvent {
                    event_id: family.next_event_id(),
                    order_key: command.order_key,
                    family: family.id,
                    actor: command.actor,
                    target: command.target,
                    old_health,
                    new_health,
                    old_effective_length: 0.0,
                    new_effective_length: 0.0,
                    source: command.source,
                });
            }
            FractureTarget::Node(node_id) => {
                if health_loss == 0.0 {
                    continue;
                }
                let Some(state) = family.node_states.get_mut(&node_id) else {
                    continue;
                };
                let old_health = state.health;
                state.health = (state.health - health_loss).max(0.0);
                state.accumulated_damage += health_loss;
                let new_health = state.health;
                if let Some(actor) = family.node_owner.get(&node_id) {
                    family.dirty_actors.insert(*actor);
                }
                events.push(FractureEvent {
                    event_id: family.next_event_id(),
                    order_key: command.order_key,
                    family: family.id,
                    actor: command.actor,
                    target: command.target,
                    old_health,
                    new_health,
                    old_effective_length: 0.0,
                    new_effective_length: 0.0,
                    source: command.source,
                });
            }
            FractureTarget::ExternalBond(bond_id) => {
                let Some(bond) = family.external_bonds.get_mut(&bond_id) else {
                    continue;
                };
                if bond.runtime.is_broken() {
                    continue;
                }
                let node = bond.node;
                let old_health = bond.runtime.health;
                let old_effective_length = bond.runtime.effective_length;
                bond.runtime.health = (bond.runtime.health - health_loss).max(0.0);
                bond.runtime.effective_length =
                    (bond.runtime.effective_length - effective_length_loss).max(0.0);
                bond.runtime.accumulated_damage += health_loss;
                let new_health = bond.runtime.health;
                let new_effective_length = bond.runtime.effective_length;
                if bond.runtime.is_broken() {
                    if let Some(actor) = family.node_owner.get(&node) {
                        family.dirty_actors.insert(*actor);
                    }
                }
                events.push(FractureEvent {
                    event_id: family.next_event_id(),
                    order_key: command.order_key,
                    family: family.id,
                    actor: command.actor,
                    target: command.target,
                    old_health,
                    new_health,
                    old_effective_length,
                    new_effective_length,
                    source: command.source,
                });
            }
            FractureTarget::Connection(connection_id) => {
                let Some(bond) = family.dynamic_structural_bonds.get_mut(&connection_id) else {
                    continue;
                };
                if bond.runtime.is_broken() {
                    continue;
                }
                let node_a = bond.node_a;
                let node_b = bond.node_b;
                let old_health = bond.runtime.health;
                let old_effective_length = bond.runtime.effective_length;
                bond.runtime.health = (bond.runtime.health - health_loss).max(0.0);
                bond.runtime.effective_length =
                    (bond.runtime.effective_length - effective_length_loss).max(0.0);
                bond.runtime.accumulated_damage += health_loss;
                let new_health = bond.runtime.health;
                let new_effective_length = bond.runtime.effective_length;
                family.mark_connection_dirty(node_a, node_b);
                events.push(FractureEvent {
                    event_id: family.next_event_id(),
                    order_key: command.order_key,
                    family: family.id,
                    actor: command.actor,
                    target: command.target,
                    old_health,
                    new_health,
                    old_effective_length,
                    new_effective_length,
                    source: command.source,
                });
            }
        }
    }
    events
}

fn actor_owns_target(family: &FxFamily, actor: FxActorId, target: FractureTarget) -> bool {
    match target {
        FractureTarget::Bond(bond_id) => {
            let Some(bond) = family.asset.bond(bond_id) else {
                return false;
            };
            family.node_owner.get(&bond.node_a) == Some(&actor)
                && family.node_owner.get(&bond.node_b) == Some(&actor)
        }
        FractureTarget::Chunk(chunk_id) => {
            let support_nodes = chunk_target_support_nodes(&family.asset, chunk_id);
            family.chunk_states.contains_key(&chunk_id)
                && !support_nodes.is_empty()
                && support_nodes
                    .iter()
                    .all(|node| family.node_owner.get(node) == Some(&actor))
        }
        FractureTarget::Node(node_id) => family.node_owner.get(&node_id) == Some(&actor),
        FractureTarget::ExternalBond(bond_id) => family
            .external_bonds
            .get(&bond_id)
            .is_some_and(|bond| family.node_owner.get(&bond.node) == Some(&actor)),
        FractureTarget::Connection(connection_id) => family
            .dynamic_structural_bonds
            .get(&connection_id)
            .is_some_and(|bond| {
                family.node_owner.get(&bond.node_a) == Some(&actor)
                    || family.node_owner.get(&bond.node_b) == Some(&actor)
            }),
    }
}

fn chunk_target_support_nodes(asset: &FxAsset, chunk_id: ChunkId) -> BTreeSet<SupportNodeId> {
    let Some(chunk) = asset.chunk(chunk_id) else {
        return BTreeSet::new();
    };
    if !chunk.support_nodes.is_empty() {
        return chunk.support_nodes.iter().copied().collect();
    }

    let children_by_parent = chunk_children_by_parent(asset.chunks());
    let mut descendant_support_nodes = BTreeSet::new();
    let mut stack = children_by_parent
        .get(&chunk_id)
        .cloned()
        .unwrap_or_default();
    while let Some(child_id) = stack.pop() {
        let Some(child) = asset.chunk(child_id) else {
            continue;
        };
        descendant_support_nodes.extend(child.support_nodes.iter().copied());
        if let Some(children) = children_by_parent.get(&child_id) {
            stack.extend(children.iter().copied());
        }
    }
    if !descendant_support_nodes.is_empty() {
        return descendant_support_nodes;
    }

    let parent_by_chunk = asset
        .chunks()
        .iter()
        .map(|chunk| (chunk.id, chunk.parent))
        .collect::<BTreeMap<_, _>>();
    let mut current = chunk.parent;
    while let Some(parent_id) = current {
        let Some(parent) = asset.chunk(parent_id) else {
            break;
        };
        if !parent.support_nodes.is_empty() {
            return parent.support_nodes.iter().copied().collect();
        }
        current = parent_by_chunk.get(&parent_id).copied().flatten();
    }
    BTreeSet::new()
}

fn chunk_children_by_parent(chunks: &[Chunk2D]) -> BTreeMap<ChunkId, Vec<ChunkId>> {
    let mut children = BTreeMap::<ChunkId, Vec<ChunkId>>::new();
    for chunk in chunks {
        if let Some(parent) = chunk.parent {
            children.entry(parent).or_default().push(chunk.id);
        }
    }
    children
}

#[derive(Clone, Debug, PartialEq)]
pub struct StressInput {
    pub order_key: DeterministicOrderKey,
    pub actor: FxActorId,
    pub node: SupportNodeId,
    pub force: Vec2,
    pub source: DamageSource,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StressNodeContext2D {
    pub node: SupportNodeId,
    pub mass: f32,
    pub position: Vec2,
    pub fixed: bool,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct StressContext2D {
    pub nodes: Vec<StressNodeContext2D>,
    pub gravity: Vec2,
    pub fallback_order_keys: Vec<(FxActorId, DeterministicOrderKey)>,
}

impl StressContext2D {
    pub fn from_family(family: &FxFamily) -> Self {
        let fixed_nodes = family
            .external_bonds
            .values()
            .filter(|bond| !bond.runtime.is_broken())
            .map(|bond| bond.node)
            .collect::<BTreeSet<_>>();
        let nodes = family
            .asset
            .support_nodes()
            .iter()
            .map(|node| {
                let (mass, weighted_position) =
                    node.voxels
                        .iter()
                        .fold((0.0, Vec2::ZERO), |(mass, position), voxel| {
                            (
                                mass + 1.0,
                                position + voxel.center(family.asset.voxel_size()),
                            )
                        });
                StressNodeContext2D {
                    node: node.id,
                    mass,
                    position: if mass > 0.0 {
                        weighted_position * (1.0 / mass)
                    } else {
                        Vec2::ZERO
                    },
                    fixed: fixed_nodes.contains(&node.id),
                }
            })
            .collect();
        Self {
            nodes,
            gravity: Vec2::ZERO,
            fallback_order_keys: Vec::new(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StressSettings {
    pub tension_limit_scale: f32,
    pub shear_limit_scale: f32,
    pub damage_per_overload: f32,
    pub max_fractures_per_frame: u16,
    pub max_iterations: u16,
    pub convergence_epsilon: f32,
    pub enable_gravity: bool,
}

impl Default for StressSettings {
    fn default() -> Self {
        Self {
            tension_limit_scale: 1.0,
            shear_limit_scale: 1.0,
            damage_per_overload: 1.0,
            max_fractures_per_frame: u16::MAX,
            max_iterations: 8,
            convergence_epsilon: 0.000_1,
            enable_gravity: true,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct StressSolver2D {
    pub settings: StressSettings,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct StressProfile {
    pub input_count: usize,
    pub actor_count_visited: usize,
    pub actors_with_input: usize,
    pub internal_bond_candidates: usize,
    pub internal_bonds_tested: usize,
    pub external_bond_candidates: usize,
    pub external_bonds_tested: usize,
    pub dynamic_structural_bonds_tested: usize,
    pub generated_commands_before_cap: usize,
    pub generated_commands_after_cap: usize,
    pub frame_cap: u16,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct StressSolveReport {
    pub commands: Vec<FractureCommand>,
    pub profile: StressProfile,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct SignedStressLoad2D {
    tension: f32,
    compression: f32,
    shear: f32,
}

impl SignedStressLoad2D {
    fn node_a_side(force: Vec2, normal: Vec2, tangent: Vec2) -> Self {
        Self::from_signed_components(force.dot(normal), force.dot(tangent))
    }

    fn node_b_side(force: Vec2, normal: Vec2, tangent: Vec2) -> Self {
        Self::from_signed_components(-force.dot(normal), force.dot(tangent))
    }

    fn external_node_side(force: Vec2, normal: Vec2, tangent: Vec2) -> Self {
        Self::from_signed_components(force.dot(normal), force.dot(tangent))
    }

    fn from_signed_components(normal_load: f32, shear_load: f32) -> Self {
        Self {
            tension: normal_load.max(0.0),
            compression: (-normal_load).max(0.0),
            shear: shear_load.abs(),
        }
    }

    fn combine_sides(a: Self, b: Self) -> Self {
        Self {
            tension: a.tension.max(b.tension),
            compression: a.compression.max(b.compression),
            shear: a.shear.max(b.shear),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum StressEdgeKind {
    Internal,
    External,
    DynamicStructural,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct StressEdgeKey {
    kind: StressEdgeKind,
    id: u32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum StressEdgeTarget {
    Bond(BondId),
    ExternalBond(ExternalBondId),
    Connection(ConnectionId),
}

#[derive(Clone, Debug, PartialEq)]
struct StressGraphEdge {
    key: StressEdgeKey,
    node_a: SupportNodeId,
    node_b: Option<SupportNodeId>,
    normal: Vec2,
    tangent: Vec2,
    base_health: f32,
    tension_limit: f32,
    shear_limit: f32,
    effective_length: f32,
    health: f32,
    target: StressEdgeTarget,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct StressEdgeLoad {
    node_a_force: Vec2,
    node_b_force: Vec2,
}

#[derive(Clone, Debug, PartialEq)]
struct StressFrontierLoad {
    node: SupportNodeId,
    force: Vec2,
    came_from: Option<StressEdgeKey>,
    visited_nodes: BTreeSet<SupportNodeId>,
}

impl StressSolver2D {
    pub fn new(settings: StressSettings) -> Self {
        Self { settings }
    }

    pub fn generate(&self, family: &FxFamily, inputs: &[StressInput]) -> Vec<FractureCommand> {
        self.generate_with_profile(family, inputs).commands
    }

    pub fn generate_with_profile(
        &self,
        family: &FxFamily,
        inputs: &[StressInput],
    ) -> StressSolveReport {
        let context = StressContext2D::from_family(family);
        self.generate_with_context_and_profile(family, &context, inputs)
    }

    pub fn generate_with_context(
        &self,
        family: &FxFamily,
        context: &StressContext2D,
        inputs: &[StressInput],
    ) -> Vec<FractureCommand> {
        self.generate_with_context_and_profile(family, context, inputs)
            .commands
    }

    pub fn generate_with_context_and_profile(
        &self,
        family: &FxFamily,
        context: &StressContext2D,
        inputs: &[StressInput],
    ) -> StressSolveReport {
        let mut initial_force_by_node: BTreeMap<SupportNodeId, Vec2> = BTreeMap::new();
        let mut first_key_by_actor: BTreeMap<FxActorId, DeterministicOrderKey> = BTreeMap::new();
        let mut source_by_actor: BTreeMap<FxActorId, DamageSource> = BTreeMap::new();
        let mut sorted_inputs = inputs.to_vec();
        sorted_inputs.sort_by_key(|input| input.order_key);
        for input in sorted_inputs {
            *initial_force_by_node
                .entry(input.node)
                .or_insert(Vec2::ZERO) += input.force;
            first_key_by_actor
                .entry(input.actor)
                .and_modify(|key| *key = (*key).min(input.order_key))
                .or_insert(input.order_key);
            source_by_actor.entry(input.actor).or_insert(input.source);
        }

        let mut node_context = context.nodes.clone();
        node_context.sort_by_key(|node| node.node);
        let mut node_context_by_id = BTreeMap::new();
        for node in node_context {
            if family.node_owner(node.node).is_none() || !valid_vec2(node.position) {
                continue;
            }
            node_context_by_id.insert(node.node, node);
        }
        let gravity_enabled = self.settings.enable_gravity
            && valid_vec2(context.gravity)
            && (context.gravity.x != 0.0 || context.gravity.y != 0.0);
        if gravity_enabled {
            let fixed_reachable_nodes = stress_fixed_reachable_nodes(family, &node_context_by_id);
            let fallback_order_keys = context
                .fallback_order_keys
                .iter()
                .copied()
                .collect::<BTreeMap<_, _>>();
            for (node, node_context) in &node_context_by_id {
                if !fixed_reachable_nodes.contains(node) {
                    continue;
                }
                if !node_context.mass.is_finite() || node_context.mass <= 0.0 {
                    continue;
                }
                let Some(actor) = family.node_owner(*node) else {
                    continue;
                };
                *initial_force_by_node.entry(*node).or_insert(Vec2::ZERO) +=
                    context.gravity * node_context.mass;
                if let Some(order_key) = fallback_order_keys.get(&actor).copied() {
                    first_key_by_actor
                        .entry(actor)
                        .and_modify(|key| *key = (*key).min(order_key))
                        .or_insert(order_key);
                }
                source_by_actor.entry(actor).or_insert(DamageSource::Stress);
            }
        }

        let mut dynamic_connection_actor: BTreeMap<
            ConnectionId,
            (FxActorId, DeterministicOrderKey),
        > = BTreeMap::new();
        for bond in family.dynamic_structural_bonds.values() {
            if bond.policy != DynamicConnectionPolicy::GraphOnly || bond.runtime.is_broken() {
                continue;
            }
            let mut candidates = Vec::new();
            for node in [bond.node_a, bond.node_b] {
                let Some(actor) = family.node_owner.get(&node).copied() else {
                    continue;
                };
                let Some(order_key) = first_key_by_actor.get(&actor).copied() else {
                    continue;
                };
                candidates.push((order_key, actor));
            }
            candidates.sort_unstable();
            candidates.dedup();
            if let Some((order_key, actor)) = candidates.first().copied() {
                dynamic_connection_actor.insert(bond.id, (actor, order_key));
            }
        }

        let mut profile = StressProfile {
            input_count: inputs.len(),
            actors_with_input: first_key_by_actor.len(),
            frame_cap: self.settings.max_fractures_per_frame,
            ..StressProfile::default()
        };

        let edges = stress_graph_edges(family, &mut profile);
        for _ in &family.actors {
            profile.actor_count_visited += 1;
        }

        let fixed_reachable_nodes = stress_fixed_reachable_nodes(family, &node_context_by_id);
        let mut edge_loads = stress_solve_anchored_residuals(
            &edges,
            &node_context_by_id,
            &fixed_reachable_nodes,
            &initial_force_by_node,
            self.settings.max_iterations,
            self.settings.convergence_epsilon,
        );
        stress_solve_free_residual_paths(
            &edges,
            &fixed_reachable_nodes,
            &initial_force_by_node,
            self.settings.max_iterations,
            self.settings.convergence_epsilon,
            &mut edge_loads,
        );

        let mut commands_by_actor = BTreeMap::<FxActorId, Vec<FractureCommand>>::new();
        for edge in &edges {
            let Some(load) = edge_loads.get(&edge.key).copied() else {
                continue;
            };
            let Some((actor, order_key)) = stress_command_actor_and_order(
                family,
                edge,
                &first_key_by_actor,
                &dynamic_connection_actor,
            ) else {
                continue;
            };
            let (normal, tangent) = stress_edge_axes(edge, &node_context_by_id);
            let signed_load = match edge.target {
                StressEdgeTarget::ExternalBond(_) => {
                    SignedStressLoad2D::external_node_side(load.node_a_force, normal, tangent)
                }
                StressEdgeTarget::Bond(_) | StressEdgeTarget::Connection(_) => {
                    SignedStressLoad2D::combine_sides(
                        SignedStressLoad2D::node_a_side(load.node_a_force, normal, tangent),
                        SignedStressLoad2D::node_b_side(load.node_b_force, normal, tangent),
                    )
                }
            };
            let health_ratio = if edge.base_health > 0.0 {
                (edge.health / edge.base_health).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let tension_limit =
                edge.tension_limit * self.settings.tension_limit_scale * health_ratio.max(0.001);
            let shear_limit =
                edge.shear_limit * self.settings.shear_limit_scale * health_ratio.max(0.001);
            if signed_load.tension <= tension_limit && signed_load.shear <= shear_limit {
                continue;
            }
            commands_by_actor
                .entry(actor)
                .or_default()
                .push(FractureCommand {
                    order_key,
                    actor,
                    target: match edge.target {
                        StressEdgeTarget::Bond(id) => FractureTarget::Bond(id),
                        StressEdgeTarget::ExternalBond(id) => FractureTarget::ExternalBond(id),
                        StressEdgeTarget::Connection(id) => FractureTarget::Connection(id),
                    },
                    health_loss: self.settings.damage_per_overload,
                    effective_length_loss: if signed_load.tension > tension_limit {
                        edge.effective_length
                    } else {
                        0.0
                    },
                    source: *source_by_actor.get(&actor).unwrap_or(&DamageSource::Stress),
                });
        }

        let mut commands = Vec::new();
        for actor_id in family.actors.keys() {
            let Some(mut actor_commands) = commands_by_actor.remove(actor_id) else {
                continue;
            };
            sort_fracture_commands(&mut actor_commands);
            profile.generated_commands_before_cap += actor_commands.len();
            actor_commands.truncate(self.settings.max_fractures_per_frame as usize);
            profile.generated_commands_after_cap += actor_commands.len();
            commands.extend(actor_commands);
        }
        sort_fracture_commands(&mut commands);
        StressSolveReport { commands, profile }
    }
}

fn force_is_active(force: Vec2, epsilon: f32) -> bool {
    force.length() > epsilon.max(0.0)
}

fn sort_frontier(frontier: &mut [StressFrontierLoad]) {
    frontier.sort_by_key(|load| {
        (
            load.node,
            load.came_from
                .map(|key| (key.kind, key.id))
                .unwrap_or((StressEdgeKind::Internal, u32::MAX)),
        )
    });
}

fn stress_solve_anchored_residuals(
    edges: &[StressGraphEdge],
    node_context_by_id: &BTreeMap<SupportNodeId, StressNodeContext2D>,
    fixed_reachable_nodes: &BTreeSet<SupportNodeId>,
    initial_force_by_node: &BTreeMap<SupportNodeId, Vec2>,
    max_iterations: u16,
    convergence_epsilon: f32,
) -> BTreeMap<StressEdgeKey, StressEdgeLoad> {
    let mut potentials = fixed_reachable_nodes
        .iter()
        .copied()
        .map(|node| (node, Vec2::ZERO))
        .collect::<BTreeMap<_, _>>();
    let incident = stress_incident_edges(edges);
    let epsilon = convergence_epsilon.max(0.0);
    for _ in 0..max_iterations {
        let mut max_residual = 0.0f32;
        for node in fixed_reachable_nodes {
            let Some(incident_edges) = incident.get(node) else {
                continue;
            };
            let mut degree = 0.0f32;
            let mut neighbor_sum = Vec2::ZERO;
            for edge in incident_edges {
                if edge.node_b.is_none() {
                    degree += 1.0;
                    continue;
                }
                let Some(other) = stress_edge_other_node(edge, *node) else {
                    continue;
                };
                if !fixed_reachable_nodes.contains(&other)
                    || !node_context_by_id.contains_key(&other)
                {
                    continue;
                }
                degree += 1.0;
                neighbor_sum += potentials.get(&other).copied().unwrap_or(Vec2::ZERO);
            }
            if degree <= 0.0 {
                continue;
            }
            let old = potentials.get(node).copied().unwrap_or(Vec2::ZERO);
            let load = initial_force_by_node
                .get(node)
                .copied()
                .unwrap_or(Vec2::ZERO);
            let next = (load + neighbor_sum) * (1.0 / degree);
            max_residual = max_residual.max((next - old).length());
            potentials.insert(*node, next);
        }
        if max_residual <= epsilon {
            break;
        }
    }

    let mut edge_loads = BTreeMap::new();
    for edge in edges {
        match edge.node_b {
            Some(node_b)
                if fixed_reachable_nodes.contains(&edge.node_a)
                    && fixed_reachable_nodes.contains(&node_b) =>
            {
                let potential_a = potentials.get(&edge.node_a).copied().unwrap_or(Vec2::ZERO);
                let potential_b = potentials.get(&node_b).copied().unwrap_or(Vec2::ZERO);
                edge_loads.insert(
                    edge.key,
                    StressEdgeLoad {
                        node_a_force: potential_a - potential_b,
                        node_b_force: potential_b - potential_a,
                    },
                );
            }
            None if fixed_reachable_nodes.contains(&edge.node_a) => {
                edge_loads.insert(
                    edge.key,
                    StressEdgeLoad {
                        node_a_force: potentials.get(&edge.node_a).copied().unwrap_or(Vec2::ZERO),
                        node_b_force: Vec2::ZERO,
                    },
                );
            }
            _ => {}
        }
    }
    edge_loads
}

fn stress_solve_free_residual_paths(
    edges: &[StressGraphEdge],
    fixed_reachable_nodes: &BTreeSet<SupportNodeId>,
    initial_force_by_node: &BTreeMap<SupportNodeId, Vec2>,
    max_iterations: u16,
    convergence_epsilon: f32,
    edge_loads: &mut BTreeMap<StressEdgeKey, StressEdgeLoad>,
) {
    let incident = stress_incident_edges(edges);
    let mut frontier = initial_force_by_node
        .iter()
        .filter_map(|(node, force)| {
            if fixed_reachable_nodes.contains(node) || !force_is_active(*force, convergence_epsilon)
            {
                return None;
            }
            Some(StressFrontierLoad {
                node: *node,
                force: *force,
                came_from: None,
                visited_nodes: BTreeSet::from([*node]),
            })
        })
        .collect::<Vec<_>>();
    sort_frontier(&mut frontier);
    let mut processed = BTreeSet::<(SupportNodeId, Option<StressEdgeKey>)>::new();

    for _ in 0..max_iterations {
        if frontier.is_empty() {
            break;
        }
        let mut max_residual = 0.0f32;
        let mut next = Vec::new();
        for load in frontier {
            if !processed.insert((load.node, load.came_from)) {
                continue;
            }
            let outgoing = incident
                .get(&load.node)
                .map(|edges| {
                    edges
                        .iter()
                        .filter(|edge| {
                            edge.node_b.is_some()
                                && Some(edge.key) != load.came_from
                                && stress_edge_other_node(edge, load.node)
                                    .is_some_and(|other| !load.visited_nodes.contains(&other))
                        })
                        .copied()
                        .collect::<Vec<_>>()
                })
                .unwrap_or_default();
            if outgoing.is_empty() {
                continue;
            }
            let share = load.force * (1.0 / outgoing.len() as f32);
            for edge in outgoing {
                let Some(other) = stress_edge_other_node(edge, load.node) else {
                    continue;
                };
                stress_accumulate_edge_load(edge_loads, edge.key, load.node == edge.node_a, share);
                max_residual = max_residual.max(share.length());
                let mut visited_nodes = load.visited_nodes.clone();
                visited_nodes.insert(other);
                next.push(StressFrontierLoad {
                    node: other,
                    force: share,
                    came_from: Some(edge.key),
                    visited_nodes,
                });
            }
        }
        if max_residual <= convergence_epsilon.max(0.0) {
            break;
        }
        frontier = next
            .into_iter()
            .filter(|load| force_is_active(load.force, convergence_epsilon))
            .collect();
        sort_frontier(&mut frontier);
    }
}

fn stress_incident_edges(
    edges: &[StressGraphEdge],
) -> BTreeMap<SupportNodeId, Vec<&StressGraphEdge>> {
    let mut incident = BTreeMap::<SupportNodeId, Vec<&StressGraphEdge>>::new();
    for edge in edges {
        incident.entry(edge.node_a).or_default().push(edge);
        if let Some(node_b) = edge.node_b {
            incident.entry(node_b).or_default().push(edge);
        }
    }
    for edges in incident.values_mut() {
        edges.sort_by_key(|edge| edge.key);
    }
    incident
}

fn stress_edge_other_node(edge: &StressGraphEdge, node: SupportNodeId) -> Option<SupportNodeId> {
    if edge.node_a == node {
        edge.node_b
    } else if edge.node_b == Some(node) {
        Some(edge.node_a)
    } else {
        None
    }
}

fn stress_fixed_reachable_nodes(
    family: &FxFamily,
    node_context_by_id: &BTreeMap<SupportNodeId, StressNodeContext2D>,
) -> BTreeSet<SupportNodeId> {
    let mut seen = BTreeSet::new();
    let mut queue = VecDeque::new();
    for (node, context) in node_context_by_id {
        if context.fixed && seen.insert(*node) {
            queue.push_back(*node);
        }
    }
    while let Some(node) = queue.pop_front() {
        for bond in &family.asset.internal_bonds {
            if family
                .bond_state(bond.id)
                .is_none_or(BondRuntimeState::is_broken)
            {
                continue;
            }
            let Some(other) = bond.other(node) else {
                continue;
            };
            if node_context_by_id.contains_key(&other) && seen.insert(other) {
                queue.push_back(other);
            }
        }
        for bond in family.dynamic_structural_bonds.values() {
            if bond.policy != DynamicConnectionPolicy::GraphOnly || bond.runtime.is_broken() {
                continue;
            }
            let other = if bond.node_a == node {
                Some(bond.node_b)
            } else if bond.node_b == node {
                Some(bond.node_a)
            } else {
                None
            };
            let Some(other) = other else {
                continue;
            };
            if node_context_by_id.contains_key(&other) && seen.insert(other) {
                queue.push_back(other);
            }
        }
    }
    seen
}

fn stress_accumulate_edge_load(
    edge_loads: &mut BTreeMap<StressEdgeKey, StressEdgeLoad>,
    key: StressEdgeKey,
    node_a_side: bool,
    force: Vec2,
) {
    let load = edge_loads.entry(key).or_default();
    if node_a_side {
        load.node_a_force += force;
    } else {
        load.node_b_force += force;
    }
}

fn stress_graph_edges(family: &FxFamily, profile: &mut StressProfile) -> Vec<StressGraphEdge> {
    let mut edges = Vec::new();
    for bond in &family.asset.internal_bonds {
        profile.internal_bond_candidates += 1;
        let Some(state) = family.bond_state(bond.id) else {
            continue;
        };
        if state.is_broken() {
            continue;
        }
        profile.internal_bonds_tested += 1;
        edges.push(StressGraphEdge {
            key: StressEdgeKey {
                kind: StressEdgeKind::Internal,
                id: bond.id.0,
            },
            node_a: bond.node_a,
            node_b: Some(bond.node_b),
            normal: bond.normal,
            tangent: bond.tangent,
            base_health: bond.base_health,
            tension_limit: bond.tension_limit,
            shear_limit: bond.shear_limit,
            effective_length: bond.length,
            health: state.health,
            target: StressEdgeTarget::Bond(bond.id),
        });
    }
    for bond in family.external_bonds.values() {
        profile.external_bond_candidates += 1;
        if bond.runtime.is_broken() {
            continue;
        }
        profile.external_bonds_tested += 1;
        edges.push(StressGraphEdge {
            key: StressEdgeKey {
                kind: StressEdgeKind::External,
                id: bond.id.0,
            },
            node_a: bond.node,
            node_b: None,
            normal: bond.normal,
            tangent: bond.tangent,
            base_health: bond.base_health,
            tension_limit: bond.tension_limit,
            shear_limit: bond.shear_limit,
            effective_length: bond.runtime.effective_length,
            health: bond.runtime.health,
            target: StressEdgeTarget::ExternalBond(bond.id),
        });
    }
    for bond in family.dynamic_structural_bonds.values() {
        if bond.policy != DynamicConnectionPolicy::GraphOnly || bond.runtime.is_broken() {
            continue;
        }
        profile.dynamic_structural_bonds_tested += 1;
        edges.push(StressGraphEdge {
            key: StressEdgeKey {
                kind: StressEdgeKind::DynamicStructural,
                id: bond.id.0,
            },
            node_a: bond.node_a,
            node_b: Some(bond.node_b),
            normal: bond.normal,
            tangent: bond.tangent,
            base_health: bond.base_health,
            tension_limit: bond.tension_limit,
            shear_limit: bond.shear_limit,
            effective_length: bond.runtime.effective_length,
            health: bond.runtime.health,
            target: StressEdgeTarget::Connection(bond.id),
        });
    }
    edges.sort_by_key(|edge| edge.key);
    edges
}

fn stress_edge_axes(
    edge: &StressGraphEdge,
    node_context_by_id: &BTreeMap<SupportNodeId, StressNodeContext2D>,
) -> (Vec2, Vec2) {
    if edge.node_b.is_none() {
        return (edge.normal, edge.tangent);
    }
    let position_a = node_context_by_id
        .get(&edge.node_a)
        .map(|node| node.position);
    let position_b = edge
        .node_b
        .and_then(|node| node_context_by_id.get(&node).map(|node| node.position));
    let Some((position_a, position_b)) = position_a.zip(position_b) else {
        return (edge.normal, edge.tangent);
    };
    let normal = (position_b - position_a).normalized_or_zero();
    if normal == Vec2::ZERO {
        (edge.normal, edge.tangent)
    } else {
        (normal, normal.perp())
    }
}

fn stress_command_actor_and_order(
    family: &FxFamily,
    edge: &StressGraphEdge,
    first_key_by_actor: &BTreeMap<FxActorId, DeterministicOrderKey>,
    dynamic_connection_actor: &BTreeMap<ConnectionId, (FxActorId, DeterministicOrderKey)>,
) -> Option<(FxActorId, DeterministicOrderKey)> {
    match edge.target {
        StressEdgeTarget::Bond(_) => {
            let actor = family.node_owner(edge.node_a)?;
            first_key_by_actor
                .get(&actor)
                .copied()
                .map(|order_key| (actor, order_key))
        }
        StressEdgeTarget::ExternalBond(_) => {
            let actor = family.node_owner(edge.node_a)?;
            first_key_by_actor
                .get(&actor)
                .copied()
                .map(|order_key| (actor, order_key))
        }
        StressEdgeTarget::Connection(id) => dynamic_connection_actor.get(&id).copied(),
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct SplitEvent {
    pub event_id: EventId,
    pub family: FxFamilyId,
    pub parent_actor: FxActorId,
    pub kept_actor: FxActorId,
    pub created_children: Vec<FxActorId>,
    pub fragments: Vec<Vec<SupportNodeId>>,
    pub kept_fragment: Vec<SupportNodeId>,
}

pub fn split_dirty_actors(family: &mut FxFamily) -> Vec<SplitEvent> {
    let dirty: Vec<_> = family.dirty_actors.iter().copied().collect();
    let mut events = Vec::new();
    for actor_id in dirty {
        let Some(actor) = family.actors.get(&actor_id).cloned() else {
            family.dirty_actors.remove(&actor_id);
            continue;
        };
        let components = actor_components(&actor, family);
        if components.len() <= 1 {
            family.dirty_actors.remove(&actor_id);
            continue;
        }

        let kept_idx = components
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| compare_fragment_for_keep(a, b, &family.asset))
            .map(|(idx, _)| idx)
            .expect("components is non-empty");
        let kept_nodes = components[kept_idx].clone();
        let mut created_children = Vec::new();
        let mut child_components: Vec<_> = components
            .iter()
            .enumerate()
            .filter_map(|(idx, nodes)| (idx != kept_idx).then_some(nodes.clone()))
            .collect();
        child_components.sort_by_key(|nodes| nodes.first().copied().unwrap_or_default());

        family
            .actors
            .insert(actor_id, build_actor(actor_id, &kept_nodes, &family.asset));
        for node in &kept_nodes {
            family.node_owner.insert(*node, actor_id);
        }

        for nodes in child_components {
            let child_id = FxActorId(family.next_actor_id);
            family.next_actor_id += 1;
            let child = build_actor(child_id, &nodes, &family.asset);
            for node in &nodes {
                family.node_owner.insert(*node, child_id);
            }
            family.actors.insert(child_id, child);
            created_children.push(child_id);
        }

        let mut fragments = components;
        fragments.sort_by_key(|nodes| nodes.first().copied().unwrap_or_default());
        events.push(SplitEvent {
            event_id: family.next_event_id(),
            family: family.id,
            parent_actor: actor_id,
            kept_actor: actor_id,
            created_children,
            fragments,
            kept_fragment: kept_nodes,
        });
        family.dirty_actors.remove(&actor_id);
    }
    events
}

#[derive(Error, Debug, Clone, PartialEq)]
pub enum ValidationError {
    #[error("grid size mismatch: expected {expected}, got {actual}")]
    GridSizeMismatch { expected: usize, actual: usize },
    #[error("occupancy rows must have the same width")]
    RaggedRows,
    #[error("invalid occupancy byte {0}")]
    InvalidOccupancyByte(u8),
    #[error("voxel size must be positive")]
    InvalidVoxelSize,
    #[error("occupied voxel {0:?} is missing support coverage")]
    MissingSupportCoverage(GridCoord),
    #[error("empty voxel {coord:?} is covered by support node {node:?}")]
    EmptyVoxelCovered {
        coord: GridCoord,
        node: SupportNodeId,
    },
    #[error("support coverage overlaps at {coord:?}: {first:?} and {second:?}")]
    OverlappingSupportCoverage {
        coord: GridCoord,
        first: SupportNodeId,
        second: SupportNodeId,
    },
    #[error("support node {0:?} has no voxels")]
    EmptySupportNode(SupportNodeId),
    #[error("support node containing {0:?} is not 4-neighbor connected")]
    SupportNodeNotFourConnected(SupportNodeId),
    #[error("duplicate support node id {0:?}")]
    DuplicateSupportNodeId(SupportNodeId),
    #[error("duplicate chunk id {0:?}")]
    DuplicateChunkId(ChunkId),
    #[error("chunk references missing support node {0:?}")]
    ChunkEndpointMissing(SupportNodeId),
    #[error("chunk references missing parent chunk {0:?}")]
    ChunkParentMissing(ChunkId),
    #[error("chunk parent hierarchy contains a cycle at {0:?}")]
    ChunkParentCycle(ChunkId),
    #[error("chunk hierarchy is not an exact cover over support nodes")]
    ChunkHierarchyNotExact,
    #[error("duplicate internal bond id {0:?}")]
    DuplicateInternalBondId(BondId),
    #[error("internal bond id {actual:?} is not contiguous at index {expected:?}")]
    NonContiguousInternalBondId { expected: BondId, actual: BondId },
    #[error("bond endpoint {0:?} is missing")]
    BondEndpointMissing(SupportNodeId),
    #[error("bond cannot connect support node {0:?} to itself")]
    SelfBond(SupportNodeId),
    #[error("bond {0:?} endpoints must be canonical")]
    NonCanonicalBondEndpoints(BondId),
    #[error("bond {0:?} has invalid scalar values")]
    InvalidBondScalar(BondId),
    #[error("bond {0:?} has invalid normal/tangent directions")]
    InvalidBondDirection(BondId),
    #[error("bond {0:?} length must be positive")]
    NonPositiveBondLength(BondId),
    #[error("bond {0:?} has no interface edges")]
    EmptyBondInterface(BondId),
}

fn dominant_material(
    voxels: &[GridCoord],
    occupancy: &DenseOccupancy,
    material_map: Option<&Vec<u16>>,
) -> u16 {
    let Some(material_map) = material_map else {
        return 0;
    };
    let mut counts: BTreeMap<u16, usize> = BTreeMap::new();
    for &voxel in voxels {
        *counts
            .entry(material_map[occupancy.index(voxel)])
            .or_default() += 1;
    }
    counts
        .into_iter()
        .max_by(|(mat_a, count_a), (mat_b, count_b)| {
            count_a.cmp(count_b).then_with(|| mat_b.cmp(mat_a))
        })
        .map(|(material, _)| material)
        .unwrap_or(0)
}

fn dominant_orientation(
    voxels: &[GridCoord],
    occupancy: &DenseOccupancy,
    orientation_map: Option<&Vec<u16>>,
) -> Option<u16> {
    let orientation_map = orientation_map?;
    let mut counts: BTreeMap<u16, usize> = BTreeMap::new();
    for &voxel in voxels {
        *counts
            .entry(orientation_map[occupancy.index(voxel)])
            .or_default() += 1;
    }
    counts
        .into_iter()
        .max_by(|(angle_a, count_a), (angle_b, count_b)| {
            count_a.cmp(count_b).then_with(|| angle_b.cmp(angle_a))
        })
        .map(|(angle, _)| angle)
}

fn angle16_to_axis(angle: u16) -> Vec2 {
    let radians = (angle as f32 / u16::MAX as f32) * std::f32::consts::TAU;
    Vec2::new(radians.cos(), radians.sin()).normalized_or_zero()
}

fn stable_node_seed(
    node_id: SupportNodeId,
    voxels: &[GridCoord],
    material_id: u16,
    orientation_summary: Option<u16>,
) -> u64 {
    let mut hasher = Fnva64::default();
    hasher.write_u32(node_id.0);
    hasher.write_u32(material_id as u32);
    hasher.write_u32(orientation_summary.map_or(u32::MAX, u32::from));
    for voxel in voxels {
        hasher.write_u32(voxel.x);
        hasher.write_u32(voxel.y);
    }
    hasher.finish()
}

fn validate_chunk_hierarchy(
    chunks: &[Chunk2D],
    node_ids: &BTreeSet<SupportNodeId>,
) -> Result<BTreeMap<SupportNodeId, ChunkId>, ValidationError> {
    let mut chunk_ids = BTreeSet::new();
    let mut parent_ids = BTreeSet::new();
    let mut parent_by_chunk = BTreeMap::new();
    let mut support_chunks = BTreeSet::new();
    for chunk in chunks {
        if !chunk_ids.insert(chunk.id) {
            return Err(ValidationError::DuplicateChunkId(chunk.id));
        }
        parent_by_chunk.insert(chunk.id, chunk.parent);
        if !chunk.support_nodes.is_empty() {
            support_chunks.insert(chunk.id);
        }
        let mut chunk_node_ids = BTreeSet::new();
        for &node in &chunk.support_nodes {
            if !chunk_node_ids.insert(node) {
                return Err(ValidationError::ChunkHierarchyNotExact);
            }
            if !node_ids.contains(&node) {
                return Err(ValidationError::ChunkEndpointMissing(node));
            }
        }
        if let Some(parent) = chunk.parent {
            parent_ids.insert(parent);
        }
    }
    for parent in &parent_ids {
        if !chunk_ids.contains(parent) {
            return Err(ValidationError::ChunkParentMissing(*parent));
        }
    }
    for chunk in chunks {
        let mut seen = BTreeSet::new();
        let mut current = Some(chunk.id);
        while let Some(chunk_id) = current {
            if !seen.insert(chunk_id) {
                return Err(ValidationError::ChunkParentCycle(chunk_id));
            }
            current = parent_by_chunk.get(&chunk_id).copied().flatten();
        }
    }

    for chunk in chunks {
        if chunk.support_nodes.is_empty() {
            continue;
        }
        let mut current = chunk.parent;
        while let Some(parent) = current {
            if support_chunks.contains(&parent) {
                return Err(ValidationError::ChunkHierarchyNotExact);
            }
            current = parent_by_chunk.get(&parent).copied().flatten();
        }
    }

    let mut node_to_support_chunk = BTreeMap::new();
    for chunk in chunks {
        for &node in &chunk.support_nodes {
            if node_to_support_chunk.insert(node, chunk.id).is_some() {
                return Err(ValidationError::ChunkHierarchyNotExact);
            }
        }
    }
    if node_to_support_chunk
        .keys()
        .copied()
        .collect::<BTreeSet<_>>()
        != *node_ids
    {
        return Err(ValidationError::ChunkHierarchyNotExact);
    }
    Ok(node_to_support_chunk)
}

fn normalize_chunks(chunks: &mut [Chunk2D]) {
    for chunk in chunks.iter_mut() {
        chunk.support_nodes.sort_unstable();
    }
    chunks.sort_by_key(|chunk| chunk.id);
}

fn validate_four_neighbor_connected(voxels: &[GridCoord]) -> Result<(), ValidationError> {
    if voxels.is_empty() {
        return Ok(());
    }
    let node_id = SupportNodeId(u32::MAX);
    let set: BTreeSet<_> = voxels.iter().copied().collect();
    let mut seen = BTreeSet::new();
    let mut queue = VecDeque::from([voxels[0]]);
    seen.insert(voxels[0]);
    while let Some(coord) = queue.pop_front() {
        for next in four_neighbors(coord) {
            if set.contains(&next) && seen.insert(next) {
                queue.push_back(next);
            }
        }
    }
    if seen.len() != voxels.len() {
        Err(ValidationError::SupportNodeNotFourConnected(node_id))
    } else {
        Ok(())
    }
}

fn four_neighbors(coord: GridCoord) -> impl Iterator<Item = GridCoord> {
    let mut out = Vec::with_capacity(4);
    if coord.x > 0 {
        out.push(GridCoord::new(coord.x - 1, coord.y));
    }
    if coord.y > 0 {
        out.push(GridCoord::new(coord.x, coord.y - 1));
    }
    out.push(GridCoord::new(coord.x + 1, coord.y));
    out.push(GridCoord::new(coord.x, coord.y + 1));
    out.into_iter()
}

fn split_edge_islands(
    samples: &[impl_edge_sample::EdgeSample],
) -> Vec<Vec<impl_edge_sample::EdgeSample>> {
    let mut seen = vec![false; samples.len()];
    let mut islands = Vec::new();
    for start in 0..samples.len() {
        if seen[start] {
            continue;
        }
        seen[start] = true;
        let mut queue = VecDeque::from([start]);
        let mut island = Vec::new();
        while let Some(idx) = queue.pop_front() {
            island.push(samples[idx]);
            for next in 0..samples.len() {
                if !seen[next] && samples[idx].edge.touches(samples[next].edge) {
                    seen[next] = true;
                    queue.push_back(next);
                }
            }
        }
        island.sort_by_key(|sample| sample.edge);
        islands.push(island);
    }
    islands.sort_by_key(|island| island.first().map(|sample| sample.edge));
    islands
}

fn build_actor(id: FxActorId, nodes: &[SupportNodeId], asset: &FxAsset) -> FxActor {
    let mut owned_nodes = nodes.to_vec();
    owned_nodes.sort_unstable();
    let mut mass = 0.0;
    let mut weighted_center = Vec2::ZERO;
    let mut min: Option<GridCoord> = None;
    let mut max: Option<GridCoord> = None;
    for node_id in &owned_nodes {
        let Some(node) = asset.node(*node_id) else {
            continue;
        };
        for &voxel in &node.voxels {
            mass += 1.0;
            weighted_center += voxel.center(asset.voxel_size);
            min = Some(match min {
                Some(old) => GridCoord::new(old.x.min(voxel.x), old.y.min(voxel.y)),
                None => voxel,
            });
            max = Some(match max {
                Some(old) => GridCoord::new(old.x.max(voxel.x), old.y.max(voxel.y)),
                None => voxel,
            });
        }
    }
    let local_com = if mass > 0.0 {
        weighted_center * (1.0 / mass)
    } else {
        Vec2::ZERO
    };
    let mut inertia = 0.0;
    for node_id in &owned_nodes {
        let Some(node) = asset.node(*node_id) else {
            continue;
        };
        for &voxel in &node.voxels {
            let d = voxel.center(asset.voxel_size) - local_com;
            inertia += d.dot(d) + (asset.voxel_size * asset.voxel_size) / 6.0;
        }
    }
    FxActor {
        id,
        owned_nodes,
        mass,
        local_com,
        inertia,
        bounds: min.zip(max).map(|(min, max)| GridAabb { min, max }),
    }
}

#[cfg(test)]
fn component_without_bond(
    actor: &FxActor,
    family: &FxFamily,
    start: SupportNodeId,
    skipped_bond: Option<BondId>,
) -> Vec<SupportNodeId> {
    component_graph_walk(actor, family, start, skipped_bond)
}

#[cfg(test)]
fn component_graph_walk(
    actor: &FxActor,
    family: &FxFamily,
    start: SupportNodeId,
    skipped_bond: Option<BondId>,
) -> Vec<SupportNodeId> {
    let owned: BTreeSet<_> = actor.owned_nodes.iter().copied().collect();
    let mut seen = BTreeSet::new();
    let mut queue = VecDeque::from([start]);
    seen.insert(start);
    while let Some(node) = queue.pop_front() {
        for bond in &family.asset.internal_bonds {
            if Some(bond.id) == skipped_bond {
                continue;
            }
            if family
                .bond_state(bond.id)
                .is_none_or(BondRuntimeState::is_broken)
            {
                continue;
            }
            let Some(other) = bond.other(node) else {
                continue;
            };
            if owned.contains(&other) && seen.insert(other) {
                queue.push_back(other);
            }
        }
        for bond in family.dynamic_structural_bonds.values() {
            if bond.policy != DynamicConnectionPolicy::GraphOnly || bond.runtime.is_broken() {
                continue;
            }
            let other = if bond.node_a == node {
                Some(bond.node_b)
            } else if bond.node_b == node {
                Some(bond.node_a)
            } else {
                None
            };
            let Some(other) = other else {
                continue;
            };
            if owned.contains(&other) && seen.insert(other) {
                queue.push_back(other);
            }
        }
    }
    seen.into_iter().collect()
}

fn actor_components(actor: &FxActor, family: &FxFamily) -> Vec<Vec<SupportNodeId>> {
    let mut remaining: BTreeSet<_> = actor.owned_nodes.iter().copied().collect();
    let mut components = Vec::new();
    while let Some(start) = remaining.first().copied() {
        let component = split_component_graph_walk(actor, family, start);
        for node in &component {
            remaining.remove(node);
        }
        components.push(component);
    }
    components.sort_by_key(|nodes| nodes.first().copied().unwrap_or_default());
    components
}

fn split_component_graph_walk(
    actor: &FxActor,
    family: &FxFamily,
    start: SupportNodeId,
) -> Vec<SupportNodeId> {
    let owned: BTreeSet<_> = actor.owned_nodes.iter().copied().collect();
    let mut seen = BTreeSet::new();
    let mut queue = VecDeque::from([start]);
    seen.insert(start);
    while let Some(node) = queue.pop_front() {
        for bond in &family.asset.internal_bonds {
            if family
                .bond_state(bond.id)
                .is_none_or(BondRuntimeState::is_broken)
            {
                continue;
            }
            let Some(other) = bond.other(node) else {
                continue;
            };
            if owned.contains(&other) && seen.insert(other) {
                queue.push_back(other);
            }
        }
        for bond in family.dynamic_structural_bonds.values() {
            if bond.policy != DynamicConnectionPolicy::GraphOnly || bond.runtime.is_broken() {
                continue;
            }
            let other = if bond.node_a == node {
                Some(bond.node_b)
            } else if bond.node_b == node {
                Some(bond.node_a)
            } else {
                None
            };
            let Some(other) = other else {
                continue;
            };
            if owned.contains(&other) && seen.insert(other) {
                queue.push_back(other);
            }
        }
        for target in family
            .external_bonds
            .values()
            .filter(|bond| bond.node == node && !bond.runtime.is_broken())
            .map(|bond| bond.target)
        {
            for bond in family.external_bonds.values() {
                if bond.target != target || bond.runtime.is_broken() {
                    continue;
                }
                let other = bond.node;
                if owned.contains(&other) && seen.insert(other) {
                    queue.push_back(other);
                }
            }
        }
    }
    seen.into_iter().collect()
}

fn fragment_stats(nodes: &[SupportNodeId], asset: &FxAsset) -> (usize, f32, SupportNodeId) {
    let voxel_count = nodes
        .iter()
        .filter_map(|node| asset.node(*node))
        .map(|node| node.voxels.len())
        .sum::<usize>();
    let mass = voxel_count as f32;
    let min_node = nodes.iter().copied().min().unwrap_or_default();
    (voxel_count, mass, min_node)
}

fn compare_fragment_for_keep(
    a: &[SupportNodeId],
    b: &[SupportNodeId],
    asset: &FxAsset,
) -> Ordering {
    let (a_voxels, a_mass, a_min) = fragment_stats(a, asset);
    let (b_voxels, b_mass, b_min) = fragment_stats(b, asset);
    a_voxels
        .cmp(&b_voxels)
        .then_with(|| a_mass.total_cmp(&b_mass))
        .then_with(|| b_min.cmp(&a_min))
}

fn valid_runtime_scalar(value: f32) -> bool {
    value.is_finite() && value >= 0.0
}

fn initial_chunk_state() -> ChunkRuntimeState {
    ChunkRuntimeState {
        health: 1.0,
        accumulated_damage: 0.0,
    }
}

fn reconcile_chunk_states(
    old_states: &BTreeMap<ChunkId, ChunkRuntimeState>,
    asset: &FxAsset,
) -> BTreeMap<ChunkId, ChunkRuntimeState> {
    asset
        .chunks()
        .iter()
        .map(|chunk| {
            (
                chunk.id,
                old_states
                    .get(&chunk.id)
                    .cloned()
                    .unwrap_or_else(initial_chunk_state),
            )
        })
        .collect()
}

fn valid_vec2(value: Vec2) -> bool {
    value.x.is_finite() && value.y.is_finite()
}

fn validate_internal_bond_direction(bond: &Bond2D) -> Result<(), ValidationError> {
    if !valid_vec2(bond.normal) || !valid_vec2(bond.tangent) {
        return Err(ValidationError::InvalidBondDirection(bond.id));
    }
    let normal_len = bond.normal.length();
    let tangent_len = bond.tangent.length();
    if (normal_len - 1.0).abs() > 0.0001 || (tangent_len - 1.0).abs() > 0.0001 {
        return Err(ValidationError::InvalidBondDirection(bond.id));
    }
    let expected = bond.normal.perp();
    if (bond.tangent.x - expected.x).abs() > 0.0001 || (bond.tangent.y - expected.y).abs() > 0.0001
    {
        return Err(ValidationError::InvalidBondDirection(bond.id));
    }
    Ok(())
}

fn validate_direction(value: Vec2) -> Option<Vec2> {
    if !valid_vec2(value) {
        return None;
    }
    let normalized = value.normalized_or_zero();
    (normalized.length() > 0.0).then_some(normalized)
}

fn validate_connection_scalars(
    health: f32,
    effective_length: f32,
    tension_limit: f32,
    shear_limit: f32,
) -> Result<(), ()> {
    if valid_runtime_scalar(health)
        && valid_runtime_scalar(effective_length)
        && valid_runtime_scalar(tension_limit)
        && valid_runtime_scalar(shear_limit)
    {
        Ok(())
    } else {
        Err(())
    }
}

#[derive(Clone, Debug)]
struct Fnva64(u64);

impl Default for Fnva64 {
    fn default() -> Self {
        Self(0xcbf2_9ce4_8422_2325)
    }
}

impl Fnva64 {
    fn write_u8(&mut self, value: u8) {
        self.0 ^= value as u64;
        self.0 = self.0.wrapping_mul(0x0000_0100_0000_01b3);
    }

    fn write_u32(&mut self, value: u32) {
        for byte in value.to_le_bytes() {
            self.write_u8(byte);
        }
    }

    fn write_f32(&mut self, value: f32) {
        self.write_u32(value.to_bits());
    }

    fn finish(self) -> u64 {
        self.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn map(width: u32, ids: &[Option<u32>]) -> Vec<Option<SupportNodeId>> {
        assert_eq!(ids.len() % width as usize, 0);
        ids.iter()
            .map(|id| id.map(SupportNodeId))
            .collect::<Vec<_>>()
    }

    fn asset_from_rows(rows: &[&str], node_ids: &[Option<u32>]) -> FxAsset {
        asset_from_rows_with_limits(rows, node_ids, 10.0, 10.0)
    }

    fn asset_from_rows_with_limits(
        rows: &[&str],
        node_ids: &[Option<u32>],
        tension_limit: f32,
        shear_limit: f32,
    ) -> FxAsset {
        let occupancy = DenseOccupancy::from_rows(rows).unwrap();
        let mut desc = FxAssetDesc::new(
            FxAssetId(7),
            1.0,
            occupancy.clone(),
            map(occupancy.width(), node_ids),
        );
        desc.default_bond_health = 10.0;
        desc.default_tension_limit = tension_limit;
        desc.default_shear_limit = shear_limit;
        FxAsset::from_desc(desc).unwrap()
    }

    fn chain_asset() -> FxAsset {
        asset_from_rows(&["###"], &[Some(0), Some(1), Some(2)])
    }

    fn family_for(asset: FxAsset) -> FxFamily {
        FxFamily::instantiate(FxFamilyId(3), asset)
    }

    fn static_anchor_desc(id: u32, node: u32) -> StaticAnchorDesc {
        StaticAnchorDesc {
            id: ExternalBondId(id),
            node: SupportNodeId(node),
            target: ExternalTarget2D {
                kind: ExternalTargetKind::World,
                token: ExternalTargetToken(0),
            },
            anchor: Vec2::new(node as f32 + 0.5, 0.5),
            normal: Vec2::new(1.0, 0.0),
            health: 1.0,
            effective_length: 1.0,
            tension_limit: 0.01,
            shear_limit: 0.01,
        }
    }

    fn dynamic_graph_only_desc(id: u32, node_a: u32, node_b: u32) -> DynamicStructuralBondDesc {
        DynamicStructuralBondDesc {
            id: ConnectionId(id),
            node_a: SupportNodeId(node_a),
            node_b: SupportNodeId(node_b),
            centroid: Vec2::new((node_a + node_b) as f32 * 0.5, 0.5),
            normal: Vec2::new(1.0, 0.0),
            health: 1.0,
            effective_length: 1.0,
            tension_limit: 0.01,
            shear_limit: 0.01,
        }
    }

    fn break_bond_command(
        tick: u64,
        family: FxFamilyId,
        actor: FxActorId,
        bond: BondId,
    ) -> FractureCommand {
        FractureCommand {
            order_key: DeterministicOrderKey::new(tick, 1, family, actor, CommandId(tick as u32)),
            actor,
            target: FractureTarget::Bond(bond),
            health_loss: 20.0,
            effective_length_loss: 20.0,
            source: DamageSource::Script,
        }
    }

    fn break_external_bond_command(
        tick: u64,
        family: FxFamilyId,
        actor: FxActorId,
        bond: ExternalBondId,
    ) -> FractureCommand {
        FractureCommand {
            order_key: DeterministicOrderKey::new(tick, 1, family, actor, CommandId(tick as u32)),
            actor,
            target: FractureTarget::ExternalBond(bond),
            health_loss: 2.0,
            effective_length_loss: 2.0,
            source: DamageSource::Script,
        }
    }

    fn disconnected_three_node_asset() -> FxAsset {
        asset_from_rows(&["#.#.#"], &[Some(0), None, Some(1), None, Some(2)])
    }

    fn repair_plan_from_family(family: &FxFamily) -> RepairPlan {
        RepairPlan {
            asset: family.asset().clone(),
            node_owners: family
                .node_owner
                .iter()
                .map(|(node, actor)| (*node, *actor))
                .collect(),
            node_states: family
                .node_states()
                .map(|(node, state)| (node, state.clone()))
                .collect(),
            bond_states: family.bond_states().to_vec(),
            dirty_actors: vec![],
        }
    }

    #[test]
    fn exact_cover_basic() {
        let asset = asset_from_rows(&["##", "##"], &[Some(0), Some(0), Some(1), Some(1)]);
        assert_eq!(asset.support_nodes().len(), 2);
        assert_eq!(asset.support_nodes()[0].voxels.len(), 2);
        assert_eq!(asset.support_nodes()[1].voxels.len(), 2);
        assert!(asset.validate().is_ok());
    }

    #[test]
    fn four_neighbor_connectivity() {
        let occupancy = DenseOccupancy::from_rows(&["#.", ".#"]).unwrap();
        let desc = FxAssetDesc::new(
            FxAssetId(1),
            1.0,
            occupancy,
            map(2, &[Some(0), None, None, Some(0)]),
        );
        assert!(matches!(
            FxAsset::from_desc(desc),
            Err(ValidationError::SupportNodeNotFourConnected(_))
        ));
    }

    #[test]
    fn bond_generation_edge_scan() {
        let asset = asset_from_rows(&["##", "##"], &[Some(0), Some(1), Some(0), Some(1)]);
        assert_eq!(asset.internal_bonds().len(), 1);
        let bond = &asset.internal_bonds()[0];
        assert_eq!(bond.node_a, SupportNodeId(0));
        assert_eq!(bond.node_b, SupportNodeId(1));
        assert_eq!(bond.interface_edges.len(), 2);
        assert_eq!(bond.length, 2.0);
    }

    #[test]
    fn chunk_hierarchy_allows_non_leaf_support_exact_cover() {
        let occupancy = DenseOccupancy::from_rows(&["##"]).unwrap();
        let mut desc = FxAssetDesc::new(FxAssetId(1), 1.0, occupancy, map(2, &[Some(0), Some(1)]));
        desc.authored_chunks = Some(vec![
            Chunk2D {
                id: ChunkId(10),
                support_nodes: vec![],
                parent: Some(ChunkId(20)),
            },
            Chunk2D {
                id: ChunkId(11),
                support_nodes: vec![],
                parent: Some(ChunkId(20)),
            },
            Chunk2D {
                id: ChunkId(20),
                support_nodes: vec![SupportNodeId(0), SupportNodeId(1)],
                parent: None,
            },
        ]);

        let asset = FxAsset::from_desc(desc).unwrap();

        assert_eq!(asset.support_nodes()[0].chunk_id, ChunkId(20));
        assert_eq!(asset.support_nodes()[1].chunk_id, ChunkId(20));
        assert_eq!(asset.chunks().len(), 3);
        assert!(asset.chunk(ChunkId(20)).unwrap().support_nodes.len() == 2);
        assert!(asset.validate().is_ok());
    }

    #[test]
    fn authored_chunk_hierarchy_requires_support_exact_cover() {
        let occupancy = DenseOccupancy::from_rows(&["##"]).unwrap();
        let mut desc = FxAssetDesc::new(FxAssetId(1), 1.0, occupancy, map(2, &[Some(0), Some(1)]));
        desc.authored_chunks = Some(vec![
            Chunk2D {
                id: ChunkId(10),
                support_nodes: vec![],
                parent: Some(ChunkId(20)),
            },
            Chunk2D {
                id: ChunkId(20),
                support_nodes: vec![SupportNodeId(0)],
                parent: None,
            },
        ]);

        assert_eq!(
            FxAsset::from_desc(desc).unwrap_err(),
            ValidationError::ChunkHierarchyNotExact
        );
    }

    #[test]
    fn chunk_hierarchy_rejects_overlapping_support_ancestors() {
        let occupancy = DenseOccupancy::from_rows(&["##"]).unwrap();
        let mut desc = FxAssetDesc::new(FxAssetId(1), 1.0, occupancy, map(2, &[Some(0), Some(1)]));
        desc.authored_chunks = Some(vec![
            Chunk2D {
                id: ChunkId(10),
                support_nodes: vec![SupportNodeId(1)],
                parent: Some(ChunkId(20)),
            },
            Chunk2D {
                id: ChunkId(20),
                support_nodes: vec![SupportNodeId(0)],
                parent: None,
            },
        ]);

        assert_eq!(
            FxAsset::from_desc(desc).unwrap_err(),
            ValidationError::ChunkHierarchyNotExact
        );
    }

    #[test]
    fn authored_chunk_hierarchy_rejects_missing_parent() {
        let occupancy = DenseOccupancy::from_rows(&["##"]).unwrap();
        let mut desc = FxAssetDesc::new(FxAssetId(1), 1.0, occupancy, map(2, &[Some(0), Some(1)]));
        desc.authored_chunks = Some(vec![
            Chunk2D {
                id: ChunkId(10),
                support_nodes: vec![SupportNodeId(0)],
                parent: Some(ChunkId(99)),
            },
            Chunk2D {
                id: ChunkId(11),
                support_nodes: vec![SupportNodeId(1)],
                parent: None,
            },
        ]);

        assert_eq!(
            FxAsset::from_desc(desc).unwrap_err(),
            ValidationError::ChunkParentMissing(ChunkId(99))
        );
    }

    #[test]
    fn authored_chunk_hierarchy_rejects_self_parent() {
        let occupancy = DenseOccupancy::from_rows(&["##"]).unwrap();
        let mut desc = FxAssetDesc::new(FxAssetId(1), 1.0, occupancy, map(2, &[Some(0), Some(1)]));
        desc.authored_chunks = Some(vec![
            Chunk2D {
                id: ChunkId(10),
                support_nodes: vec![SupportNodeId(0)],
                parent: Some(ChunkId(10)),
            },
            Chunk2D {
                id: ChunkId(11),
                support_nodes: vec![SupportNodeId(1)],
                parent: None,
            },
        ]);

        assert_eq!(
            FxAsset::from_desc(desc).unwrap_err(),
            ValidationError::ChunkParentCycle(ChunkId(10))
        );
    }

    #[test]
    fn authored_chunk_hierarchy_rejects_parent_cycle() {
        let occupancy = DenseOccupancy::from_rows(&["##"]).unwrap();
        let mut desc = FxAssetDesc::new(FxAssetId(1), 1.0, occupancy, map(2, &[Some(0), Some(1)]));
        desc.authored_chunks = Some(vec![
            Chunk2D {
                id: ChunkId(10),
                support_nodes: vec![SupportNodeId(0)],
                parent: Some(ChunkId(11)),
            },
            Chunk2D {
                id: ChunkId(11),
                support_nodes: vec![SupportNodeId(1)],
                parent: Some(ChunkId(10)),
            },
        ]);

        assert_eq!(
            FxAsset::from_desc(desc).unwrap_err(),
            ValidationError::ChunkParentCycle(ChunkId(10))
        );
    }

    #[test]
    fn disconnected_interface_expands_bonds() {
        let asset = asset_from_rows(
            &["####", "##.#", "####"],
            &[
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
            ],
        );
        assert_eq!(asset.internal_bonds().len(), 2);
        assert!(asset.internal_bonds().iter().all(|bond| bond.length == 2.0));
    }

    #[test]
    fn damage_generate_no_mutation() {
        let family = family_for(chain_asset());
        let before = family.deterministic_state_digest();
        let input = DamageInput {
            order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(0), CommandId(0)),
            actor: FxActorId(0),
            target: FractureTarget::Bond(BondId(0)),
            health_loss: 2.0,
            effective_length_loss: 0.0,
            source: DamageSource::Script,
            position: Vec2::ZERO,
            radius: 0.0,
        };
        let commands = generate_damage_commands(&family, &[input]);
        assert_eq!(commands.len(), 1);
        assert_eq!(before, family.deterministic_state_digest());
    }

    #[test]
    fn apply_fracture_mutates_health() {
        let mut family = family_for(chain_asset());
        let command = FractureCommand {
            order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(0), CommandId(0)),
            actor: FxActorId(0),
            target: FractureTarget::Bond(BondId(0)),
            health_loss: 4.0,
            effective_length_loss: 0.5,
            source: DamageSource::Script,
        };
        let events = apply_fracture_commands(&mut family, &[command]);
        assert_eq!(events.len(), 1);
        assert_eq!(events[0].old_health, 10.0);
        assert_eq!(events[0].new_health, 6.0);
        assert_eq!(family.bond_state(BondId(0)).unwrap().health, 6.0);
        assert!(family.is_dirty(FxActorId(0)));
    }

    #[test]
    fn stress_tension_break() {
        let family = family_for(asset_from_rows(&["##"], &[Some(0), Some(1)]));
        let solver = StressSolver2D::new(StressSettings {
            damage_per_overload: 10.0,
            ..Default::default()
        });
        let input = StressInput {
            order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(0), CommandId(0)),
            actor: FxActorId(0),
            node: SupportNodeId(0),
            force: Vec2::new(20.0, 0.0),
            source: DamageSource::Stress,
        };
        let commands = solver.generate(&family, &[input]);
        assert_eq!(commands.len(), 1);
        assert_eq!(commands[0].target, FractureTarget::Bond(BondId(0)));
    }

    #[test]
    fn stress_shear_break() {
        let family = family_for(asset_from_rows(&["##"], &[Some(0), Some(1)]));
        let solver = StressSolver2D::new(StressSettings {
            damage_per_overload: 10.0,
            ..Default::default()
        });
        let input = StressInput {
            order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(0), CommandId(0)),
            actor: FxActorId(0),
            node: SupportNodeId(0),
            force: Vec2::new(0.0, 20.0),
            source: DamageSource::Stress,
        };
        let commands = solver.generate(&family, &[input]);
        assert_eq!(commands.len(), 1);
        assert_eq!(commands[0].target, FractureTarget::Bond(BondId(0)));
    }

    #[test]
    fn stress_tension_break_from_node_b() {
        let family = family_for(asset_from_rows(&["##"], &[Some(0), Some(1)]));
        let solver = StressSolver2D::new(StressSettings {
            damage_per_overload: 10.0,
            ..Default::default()
        });
        let input = StressInput {
            order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(0), CommandId(0)),
            actor: FxActorId(0),
            node: SupportNodeId(1),
            force: Vec2::new(-20.0, 0.0),
            source: DamageSource::Stress,
        };
        let commands = solver.generate(&family, &[input]);
        assert_eq!(commands.len(), 1);
        assert_eq!(commands[0].target, FractureTarget::Bond(BondId(0)));
    }

    #[test]
    fn stress_shear_break_from_node_b() {
        let family = family_for(asset_from_rows(&["##"], &[Some(0), Some(1)]));
        let solver = StressSolver2D::new(StressSettings {
            damage_per_overload: 10.0,
            ..Default::default()
        });
        let input = StressInput {
            order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(0), CommandId(0)),
            actor: FxActorId(0),
            node: SupportNodeId(1),
            force: Vec2::new(0.0, -20.0),
            source: DamageSource::Stress,
        };
        let commands = solver.generate(&family, &[input]);
        assert_eq!(commands.len(), 1);
        assert_eq!(commands[0].target, FractureTarget::Bond(BondId(0)));
    }

    #[test]
    fn stress_compression_does_not_break_from_node_a() {
        let family = family_for(asset_from_rows(&["##"], &[Some(0), Some(1)]));
        let solver = StressSolver2D::new(StressSettings {
            damage_per_overload: 10.0,
            ..Default::default()
        });
        let input = StressInput {
            order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(0), CommandId(0)),
            actor: FxActorId(0),
            node: SupportNodeId(0),
            force: Vec2::new(-20.0, 0.0),
            source: DamageSource::Stress,
        };
        let commands = solver.generate(&family, &[input]);
        assert!(commands.is_empty());
    }

    #[test]
    fn stress_compression_does_not_break_from_node_b() {
        let family = family_for(asset_from_rows(&["##"], &[Some(0), Some(1)]));
        let solver = StressSolver2D::new(StressSettings {
            damage_per_overload: 10.0,
            ..Default::default()
        });
        let input = StressInput {
            order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(0), CommandId(0)),
            actor: FxActorId(0),
            node: SupportNodeId(1),
            force: Vec2::new(20.0, 0.0),
            source: DamageSource::Stress,
        };
        let commands = solver.generate(&family, &[input]);
        assert!(commands.is_empty());
    }

    #[test]
    fn stress_mixed_two_sided_modes_use_mode_wise_limits() {
        let family = family_for(asset_from_rows_with_limits(
            &["##"],
            &[Some(0), Some(1)],
            10.0,
            100.0,
        ));
        let solver = StressSolver2D::new(StressSettings {
            damage_per_overload: 10.0,
            ..Default::default()
        });
        let commands = solver.generate(
            &family,
            &[
                StressInput {
                    order_key: DeterministicOrderKey::new(
                        1,
                        1,
                        family.id,
                        FxActorId(0),
                        CommandId(0),
                    ),
                    actor: FxActorId(0),
                    node: SupportNodeId(0),
                    force: Vec2::new(20.0, 0.0),
                    source: DamageSource::Stress,
                },
                StressInput {
                    order_key: DeterministicOrderKey::new(
                        1,
                        1,
                        family.id,
                        FxActorId(0),
                        CommandId(1),
                    ),
                    actor: FxActorId(0),
                    node: SupportNodeId(1),
                    force: Vec2::new(0.0, 50.0),
                    source: DamageSource::Stress,
                },
            ],
        );
        assert_eq!(commands.len(), 1);
        assert_eq!(commands[0].target, FractureTarget::Bond(BondId(0)));
        assert_eq!(commands[0].effective_length_loss, 1.0);
    }

    #[test]
    fn impact_breaks_weak_place() {
        let mut family = family_for(asset_from_rows_with_limits(
            &["###"],
            &[Some(0), Some(1), Some(2)],
            100.0,
            100.0,
        ));
        let weaken = FractureCommand {
            order_key: DeterministicOrderKey::new(0, 1, family.id, FxActorId(0), CommandId(0)),
            actor: FxActorId(0),
            target: FractureTarget::Bond(BondId(1)),
            health_loss: 9.5,
            effective_length_loss: 0.0,
            source: DamageSource::Script,
        };
        apply_fracture_commands(&mut family, &[weaken]);
        let solver = StressSolver2D::new(StressSettings {
            damage_per_overload: 10.0,
            ..Default::default()
        });
        let input = StressInput {
            order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(0), CommandId(0)),
            actor: FxActorId(0),
            node: SupportNodeId(0),
            force: Vec2::new(10.0, 0.0),
            source: DamageSource::ContactImpulse,
        };
        let commands = solver.generate(&family, &[input]);
        assert_eq!(commands.len(), 1);
        assert_eq!(commands[0].target, FractureTarget::Bond(BondId(1)));
    }

    #[test]
    fn impact_breaks_weak_place_from_other_side() {
        let mut family = family_for(asset_from_rows_with_limits(
            &["###"],
            &[Some(0), Some(1), Some(2)],
            100.0,
            100.0,
        ));
        let weaken = FractureCommand {
            order_key: DeterministicOrderKey::new(0, 1, family.id, FxActorId(0), CommandId(0)),
            actor: FxActorId(0),
            target: FractureTarget::Bond(BondId(1)),
            health_loss: 9.5,
            effective_length_loss: 0.0,
            source: DamageSource::Script,
        };
        apply_fracture_commands(&mut family, &[weaken]);
        let solver = StressSolver2D::new(StressSettings {
            damage_per_overload: 10.0,
            ..Default::default()
        });
        let input = StressInput {
            order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(0), CommandId(0)),
            actor: FxActorId(0),
            node: SupportNodeId(2),
            force: Vec2::new(-10.0, 0.0),
            source: DamageSource::ContactImpulse,
        };
        let commands = solver.generate(&family, &[input]);
        assert_eq!(commands.len(), 1);
        assert_eq!(commands[0].target, FractureTarget::Bond(BondId(1)));
    }

    #[test]
    fn largest_fragment_keeps_parent() {
        let mut family = family_for(chain_asset());
        let command = FractureCommand {
            order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(0), CommandId(0)),
            actor: FxActorId(0),
            target: FractureTarget::Bond(BondId(1)),
            health_loss: 10.0,
            effective_length_loss: 1.0,
            source: DamageSource::Script,
        };
        apply_fracture_commands(&mut family, &[command]);
        let events = split_dirty_actors(&mut family);
        assert_eq!(events.len(), 1);
        assert_eq!(events[0].kept_actor, FxActorId(0));
        assert_eq!(
            family.actor(FxActorId(0)).unwrap().owned_nodes,
            vec![SupportNodeId(0), SupportNodeId(1)]
        );
        assert_eq!(
            family.actor(FxActorId(1)).unwrap().owned_nodes,
            vec![SupportNodeId(2)]
        );
    }

    #[test]
    fn deterministic_sort_commands() {
        let family_id = FxFamilyId(1);
        let actor = FxActorId(0);
        let mut commands = vec![
            FractureCommand {
                order_key: DeterministicOrderKey::new(2, 1, family_id, actor, CommandId(3)),
                actor,
                target: FractureTarget::Bond(BondId(2)),
                health_loss: 1.0,
                effective_length_loss: 0.0,
                source: DamageSource::Script,
            },
            FractureCommand {
                order_key: DeterministicOrderKey::new(1, 1, family_id, actor, CommandId(1)),
                actor,
                target: FractureTarget::Bond(BondId(1)),
                health_loss: 1.0,
                effective_length_loss: 0.0,
                source: DamageSource::Script,
            },
            FractureCommand {
                order_key: DeterministicOrderKey::new(1, 1, family_id, actor, CommandId(0)),
                actor,
                target: FractureTarget::Bond(BondId(0)),
                health_loss: 1.0,
                effective_length_loss: 0.0,
                source: DamageSource::Script,
            },
        ];
        sort_fracture_commands(&mut commands);
        assert_eq!(
            commands.iter().map(|cmd| cmd.target).collect::<Vec<_>>(),
            vec![
                FractureTarget::Bond(BondId(0)),
                FractureTarget::Bond(BondId(1)),
                FractureTarget::Bond(BondId(2)),
            ]
        );
    }

    #[test]
    fn core_has_no_rapier_dependency() {
        let manifest = include_str!("../Cargo.toml");
        assert!(!manifest.contains("rapier2d"));
        assert!(!manifest.contains("rapier3d"));
        assert!(!manifest.contains("parry2d"));
        assert!(!manifest.contains("parry3d"));
    }

    #[test]
    fn deterministic_state_digest_stable() {
        let mut a = family_for(chain_asset());
        let mut b = family_for(chain_asset());
        let command = FractureCommand {
            order_key: DeterministicOrderKey::new(1, 1, a.id, FxActorId(0), CommandId(0)),
            actor: FxActorId(0),
            target: FractureTarget::Bond(BondId(1)),
            health_loss: 10.0,
            effective_length_loss: 1.0,
            source: DamageSource::Script,
        };
        apply_fracture_commands(&mut a, &[command.clone()]);
        apply_fracture_commands(&mut b, &[command]);
        split_dirty_actors(&mut a);
        split_dirty_actors(&mut b);
        assert_eq!(
            a.deterministic_state_digest(),
            b.deterministic_state_digest()
        );
    }

    #[test]
    fn apply_repair_plan_preserves_actor_order() {
        let mut family = family_for(asset_from_rows(&["#.#"], &[Some(0), None, Some(1)]));
        let asset = family.asset().clone();
        let bond_states = asset
            .internal_bonds()
            .iter()
            .map(|bond| BondRuntimeState {
                health: bond.base_health,
                effective_length: bond.length,
                accumulated_damage: 0.0,
            })
            .collect();
        let plan = RepairPlan {
            asset,
            node_owners: vec![
                (SupportNodeId(1), FxActorId(1)),
                (SupportNodeId(0), FxActorId(0)),
            ],
            node_states: vec![
                (
                    SupportNodeId(0),
                    NodeRuntimeState {
                        health: 1.0,
                        accumulated_damage: 0.0,
                    },
                ),
                (
                    SupportNodeId(1),
                    NodeRuntimeState {
                        health: 1.0,
                        accumulated_damage: 0.0,
                    },
                ),
            ],
            bond_states,
            dirty_actors: vec![FxActorId(1), FxActorId(0)],
        };
        let summary = family.apply_repair_plan(plan).unwrap();
        assert_eq!(summary.actor_order, vec![FxActorId(0), FxActorId(1)]);
        assert_eq!(summary.dirty_actors, vec![FxActorId(0), FxActorId(1)]);
    }

    #[test]
    fn apply_repair_plan_rejects_missing_node_owner() {
        let mut family = family_for(asset_from_rows(&["##"], &[Some(0), Some(1)]));
        let asset = family.asset().clone();
        let bond_states = asset
            .internal_bonds()
            .iter()
            .map(|bond| BondRuntimeState {
                health: bond.base_health,
                effective_length: bond.length,
                accumulated_damage: 0.0,
            })
            .collect();
        let err = family
            .apply_repair_plan(RepairPlan {
                asset,
                node_owners: vec![(SupportNodeId(0), FxActorId(0))],
                node_states: vec![
                    (
                        SupportNodeId(0),
                        NodeRuntimeState {
                            health: 1.0,
                            accumulated_damage: 0.0,
                        },
                    ),
                    (
                        SupportNodeId(1),
                        NodeRuntimeState {
                            health: 1.0,
                            accumulated_damage: 0.0,
                        },
                    ),
                ],
                bond_states,
                dirty_actors: vec![],
            })
            .unwrap_err();
        assert_eq!(err, RepairError::MissingNodeOwner(SupportNodeId(1)));
    }

    #[test]
    fn apply_repair_plan_rejects_invalid_node_runtime_state() {
        let mut family = family_for(asset_from_rows(&["#"], &[Some(0)]));
        let mut plan = repair_plan_from_family(&family);
        plan.node_states[0].1.health = f32::NAN;
        let err = family.apply_repair_plan(plan).unwrap_err();
        assert_eq!(err, RepairError::InvalidNodeState(SupportNodeId(0)));
    }

    #[test]
    fn apply_repair_plan_rejects_invalid_bond_runtime_state() {
        let mut family = family_for(asset_from_rows(&["##"], &[Some(0), Some(1)]));
        let mut plan = repair_plan_from_family(&family);
        plan.bond_states[0].effective_length = -1.0;
        let err = family.apply_repair_plan(plan).unwrap_err();
        assert_eq!(err, RepairError::InvalidBondState(BondId(0)));
    }

    #[test]
    fn apply_repair_plan_rejects_unknown_dirty_actor() {
        let mut family = family_for(asset_from_rows(&["#"], &[Some(0)]));
        let mut plan = repair_plan_from_family(&family);
        plan.dirty_actors = vec![FxActorId(99)];
        let err = family.apply_repair_plan(plan).unwrap_err();
        assert_eq!(err, RepairError::UnknownDirtyActor(FxActorId(99)));
    }

    #[test]
    fn apply_repair_plan_rejects_old_only_dirty_actor() {
        let mut family = family_for(asset_from_rows(&["#.#"], &[Some(0), None, Some(1)]));
        let plan = RepairPlan {
            asset: asset_from_rows(&["#"], &[Some(0)]),
            node_owners: vec![(SupportNodeId(0), FxActorId(0))],
            node_states: vec![(
                SupportNodeId(0),
                family.node_state(SupportNodeId(0)).unwrap().clone(),
            )],
            bond_states: vec![],
            dirty_actors: vec![FxActorId(1)],
        };
        let err = family.apply_repair_plan(plan).unwrap_err();
        assert_eq!(err, RepairError::UnknownDirtyActor(FxActorId(1)));
    }

    #[test]
    fn apply_repair_plan_rejects_changed_actor_not_dirty() {
        let mut family = family_for(asset_from_rows(&["#.#"], &[Some(0), None, Some(1)]));
        let mut plan = repair_plan_from_family(&family);
        plan.node_owners = vec![
            (SupportNodeId(0), FxActorId(0)),
            (SupportNodeId(1), FxActorId(0)),
        ];
        let err = family.apply_repair_plan(plan).unwrap_err();
        assert_eq!(err, RepairError::ActorChangedNotDirty(FxActorId(0)));
    }

    #[test]
    fn apply_repair_plan_rejects_disconnected_actor_not_dirty() {
        let mut family = family_for(asset_from_rows(&["###"], &[Some(0), Some(1), Some(2)]));
        let mut plan = repair_plan_from_family(&family);
        plan.bond_states[1].health = 0.0;
        let err = family.apply_repair_plan(plan).unwrap_err();
        assert_eq!(err, RepairError::DisconnectedActorNotDirty(FxActorId(0)));
    }

    #[test]
    fn apply_repair_plan_digest_changes_on_asset_overlay() {
        let mut family = family_for(asset_from_rows(&["##"], &[Some(0), Some(1)]));
        let before = family.deterministic_state_digest();
        let asset = asset_from_rows(&["#"], &[Some(0)]);
        let plan = RepairPlan {
            bond_states: vec![],
            node_owners: vec![(SupportNodeId(0), FxActorId(0))],
            node_states: vec![(
                SupportNodeId(0),
                NodeRuntimeState {
                    health: 1.0,
                    accumulated_damage: 0.0,
                },
            )],
            asset,
            dirty_actors: vec![FxActorId(0)],
        };
        family.apply_repair_plan(plan).unwrap();
        assert_ne!(before, family.deterministic_state_digest());
    }

    #[test]
    fn node_health_damage_and_repair_transfer() {
        let mut family = family_for(asset_from_rows(&["##"], &[Some(0), Some(1)]));
        let command = FractureCommand {
            order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(0), CommandId(0)),
            actor: FxActorId(0),
            target: FractureTarget::Node(SupportNodeId(0)),
            health_loss: 0.25,
            effective_length_loss: 0.0,
            source: DamageSource::Script,
        };
        apply_fracture_commands(&mut family, &[command]);
        assert_eq!(family.node_state(SupportNodeId(0)).unwrap().health, 0.75);
        let asset = family.asset().clone();
        let plan = RepairPlan {
            asset: asset.clone(),
            node_owners: vec![
                (SupportNodeId(0), FxActorId(0)),
                (SupportNodeId(1), FxActorId(0)),
            ],
            node_states: vec![
                (
                    SupportNodeId(0),
                    family.node_state(SupportNodeId(0)).unwrap().clone(),
                ),
                (
                    SupportNodeId(1),
                    family.node_state(SupportNodeId(1)).unwrap().clone(),
                ),
            ],
            bond_states: family.bond_states().to_vec(),
            dirty_actors: vec![FxActorId(0)],
        };
        family.apply_repair_plan(plan).unwrap();
        assert_eq!(family.node_state(SupportNodeId(0)).unwrap().health, 0.75);
    }

    #[test]
    fn repair_plan_marks_dirty_actors_for_split() {
        let mut family = family_for(asset_from_rows(&["##"], &[Some(0), Some(1)]));
        let asset = family.asset().clone();
        let plan = RepairPlan {
            asset,
            node_owners: vec![
                (SupportNodeId(0), FxActorId(0)),
                (SupportNodeId(1), FxActorId(0)),
            ],
            node_states: family
                .node_states()
                .map(|(node, state)| (node, state.clone()))
                .collect(),
            bond_states: family.bond_states().to_vec(),
            dirty_actors: vec![FxActorId(0)],
        };
        family.apply_repair_plan(plan).unwrap();
        assert!(family.is_dirty(FxActorId(0)));
    }

    #[test]
    fn split_single_island_noop() {
        let mut family = family_for(chain_asset());
        family.mark_actor_dirty_for_test(FxActorId(0));
        let events = split_dirty_actors(&mut family);
        assert!(events.is_empty());
        assert_eq!(family.actor_count(), 1);
    }

    #[test]
    fn largest_fragment_tie_breaks_by_min_node_id() {
        let mut family = family_for(asset_from_rows(&["###"], &[Some(0), Some(1), Some(2)]));
        let commands = vec![
            FractureCommand {
                order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(0), CommandId(0)),
                actor: FxActorId(0),
                target: FractureTarget::Bond(BondId(0)),
                health_loss: 10.0,
                effective_length_loss: 1.0,
                source: DamageSource::Script,
            },
            FractureCommand {
                order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(0), CommandId(1)),
                actor: FxActorId(0),
                target: FractureTarget::Bond(BondId(1)),
                health_loss: 10.0,
                effective_length_loss: 1.0,
                source: DamageSource::Script,
            },
        ];
        apply_fracture_commands(&mut family, &commands);
        let events = split_dirty_actors(&mut family);
        assert_eq!(events.len(), 1);
        assert_eq!(
            family.actor(FxActorId(0)).unwrap().owned_nodes,
            vec![SupportNodeId(0)]
        );
    }

    #[test]
    fn split_child_ids_are_stable() {
        let run = || {
            let mut family = family_for(asset_from_rows(&["###"], &[Some(0), Some(1), Some(2)]));
            let commands = vec![
                FractureCommand {
                    order_key: DeterministicOrderKey::new(
                        1,
                        1,
                        family.id,
                        FxActorId(0),
                        CommandId(0),
                    ),
                    actor: FxActorId(0),
                    target: FractureTarget::Bond(BondId(0)),
                    health_loss: 10.0,
                    effective_length_loss: 1.0,
                    source: DamageSource::Script,
                },
                FractureCommand {
                    order_key: DeterministicOrderKey::new(
                        1,
                        1,
                        family.id,
                        FxActorId(0),
                        CommandId(1),
                    ),
                    actor: FxActorId(0),
                    target: FractureTarget::Bond(BondId(1)),
                    health_loss: 10.0,
                    effective_length_loss: 1.0,
                    source: DamageSource::Script,
                },
            ];
            apply_fracture_commands(&mut family, &commands);
            split_dirty_actors(&mut family);
            family
                .actors()
                .map(|(actor_id, _)| *actor_id)
                .collect::<Vec<_>>()
        };
        assert_eq!(run(), run());
    }

    #[test]
    fn stress_generate_commands_no_direct_mutation() {
        let family = family_for(asset_from_rows(&["##"], &[Some(0), Some(1)]));
        let before = family.deterministic_state_digest();
        let solver = StressSolver2D::new(Default::default());
        let input = StressInput {
            order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(0), CommandId(0)),
            actor: FxActorId(0),
            node: SupportNodeId(0),
            force: Vec2::new(20.0, 0.0),
            source: DamageSource::Stress,
        };
        assert_eq!(solver.generate(&family, &[input]).len(), 1);
        assert_eq!(before, family.deterministic_state_digest());
    }

    #[test]
    fn stress_generate_with_profile_reports_deterministic_counters() {
        let family = family_for(asset_from_rows(&["##"], &[Some(0), Some(1)]));
        let solver = StressSolver2D::new(StressSettings {
            max_fractures_per_frame: 1,
            ..Default::default()
        });
        let input = StressInput {
            order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(0), CommandId(0)),
            actor: FxActorId(0),
            node: SupportNodeId(0),
            force: Vec2::new(20.0, 0.0),
            source: DamageSource::Stress,
        };

        let report = solver.generate_with_profile(&family, &[input.clone()]);

        assert_eq!(report.commands.len(), 1);
        assert_eq!(report.profile.input_count, 1);
        assert_eq!(report.profile.actor_count_visited, 1);
        assert_eq!(report.profile.actors_with_input, 1);
        assert_eq!(report.profile.internal_bond_candidates, 1);
        assert_eq!(report.profile.internal_bonds_tested, 1);
        assert_eq!(report.profile.external_bond_candidates, 0);
        assert_eq!(report.profile.external_bonds_tested, 0);
        assert_eq!(report.profile.dynamic_structural_bonds_tested, 0);
        assert_eq!(report.profile.generated_commands_before_cap, 1);
        assert_eq!(report.profile.generated_commands_after_cap, 1);
        assert_eq!(report.profile.frame_cap, 1);
        assert_eq!(
            solver.generate(&family, &[input]).len(),
            report.commands.len()
        );
    }

    #[test]
    fn stress_max_iterations_affects_long_path_propagation() {
        let family = family_for(asset_from_rows(
            &["####"],
            &[Some(0), Some(1), Some(2), Some(3)],
        ));
        let input = StressInput {
            order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(0), CommandId(0)),
            actor: FxActorId(0),
            node: SupportNodeId(0),
            force: Vec2::new(20.0, 0.0),
            source: DamageSource::Stress,
        };

        let one_iteration = StressSolver2D::new(StressSettings {
            max_iterations: 1,
            damage_per_overload: 10.0,
            ..Default::default()
        })
        .generate(&family, &[input.clone()]);
        let three_iterations = StressSolver2D::new(StressSettings {
            max_iterations: 3,
            damage_per_overload: 10.0,
            ..Default::default()
        })
        .generate(&family, &[input]);

        assert_eq!(one_iteration.len(), 1);
        assert_eq!(one_iteration[0].target, FractureTarget::Bond(BondId(0)));
        assert!(three_iterations.len() >= 3);
        assert!(
            three_iterations
                .iter()
                .any(|command| command.target == FractureTarget::Bond(BondId(2)))
        );
    }

    #[test]
    fn stress_cycle_no_iteration_amplification() {
        let family = family_for(asset_from_rows_with_limits(
            &["##", "##"],
            &[Some(0), Some(1), Some(2), Some(3)],
            7.5,
            7.5,
        ));
        let input = StressInput {
            order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(0), CommandId(0)),
            actor: FxActorId(0),
            node: SupportNodeId(0),
            force: Vec2::new(20.0, 0.0),
            source: DamageSource::Stress,
        };
        let shallow = StressSolver2D::new(StressSettings {
            max_iterations: 4,
            damage_per_overload: 10.0,
            ..Default::default()
        })
        .generate(&family, &[input.clone()]);
        let deep = StressSolver2D::new(StressSettings {
            max_iterations: 32,
            damage_per_overload: 10.0,
            ..Default::default()
        })
        .generate(&family, &[input]);

        assert_eq!(deep, shallow);
    }

    #[test]
    fn stress_branch_force_not_duplicated() {
        let family = family_for(asset_from_rows_with_limits(
            &["###"],
            &[Some(0), Some(1), Some(2)],
            15.0,
            15.0,
        ));
        let input = StressInput {
            order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(0), CommandId(0)),
            actor: FxActorId(0),
            node: SupportNodeId(1),
            force: Vec2::new(20.0, 0.0),
            source: DamageSource::Stress,
        };

        let commands = StressSolver2D::new(StressSettings {
            max_iterations: 8,
            damage_per_overload: 10.0,
            ..Default::default()
        })
        .generate(&family, &[input]);

        assert!(
            commands.is_empty(),
            "the 20N load must be split across the two branch bonds, not copied into both"
        );
    }

    #[test]
    fn stress_max_iterations_converges_not_overloads() {
        let mut family = family_for(asset_from_rows_with_limits(
            &["##"],
            &[Some(0), Some(1)],
            100.0,
            100.0,
        ));
        let mut anchor = static_anchor_desc(7, 0);
        anchor.tension_limit = 2.0;
        anchor.shear_limit = 2.0;
        family.connect_static_anchor(anchor).unwrap();
        let input = StressInput {
            order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(0), CommandId(0)),
            actor: FxActorId(0),
            node: SupportNodeId(1),
            force: Vec2::new(1.0, 0.0),
            source: DamageSource::Stress,
        };
        let early = StressSolver2D::new(StressSettings {
            max_iterations: 2,
            damage_per_overload: 10.0,
            ..Default::default()
        })
        .generate(&family, &[input.clone()]);
        let converged = StressSolver2D::new(StressSettings {
            max_iterations: 32,
            damage_per_overload: 10.0,
            ..Default::default()
        })
        .generate(&family, &[input]);

        assert!(early.is_empty());
        assert!(
            converged.is_empty(),
            "extra iterations must converge rather than repeatedly add the same path load"
        );
    }

    #[test]
    fn stress_gravity_remote_mass_loads_static_anchor() {
        let mut family = family_for(asset_from_rows_with_limits(
            &["##"],
            &[Some(0), Some(1)],
            100.0,
            100.0,
        ));
        let anchor = family
            .connect_static_anchor(static_anchor_desc(7, 0))
            .unwrap();
        let mut context = StressContext2D::from_family(&family);
        context.gravity = Vec2::new(0.0, -1.0);
        context.fallback_order_keys = vec![(
            FxActorId(0),
            DeterministicOrderKey::new(1, 30, family.id, FxActorId(0), CommandId(0)),
        )];

        let commands = StressSolver2D::new(StressSettings {
            max_iterations: 32,
            damage_per_overload: 1.0,
            ..Default::default()
        })
        .generate_with_context(&family, &context, &[]);

        assert_eq!(commands.len(), 1);
        assert_eq!(commands[0].target, FractureTarget::ExternalBond(anchor));

        apply_fracture_commands(&mut family, &commands);
        let mut broken_context = StressContext2D::from_family(&family);
        broken_context.gravity = Vec2::new(0.0, -1.0);
        broken_context.fallback_order_keys = vec![(
            FxActorId(0),
            DeterministicOrderKey::new(2, 30, family.id, FxActorId(0), CommandId(0)),
        )];
        assert!(
            StressSolver2D::new(StressSettings {
                max_iterations: 32,
                damage_per_overload: 1.0,
                ..Default::default()
            })
            .generate_with_context(&family, &broken_context, &[])
            .is_empty(),
            "a broken external bond must stop acting as a fixed endpoint"
        );
    }

    #[test]
    fn stress_remote_contact_loads_static_anchor() {
        let mut family = family_for(asset_from_rows_with_limits(
            &["##"],
            &[Some(0), Some(1)],
            100.0,
            100.0,
        ));
        let anchor = family
            .connect_static_anchor(static_anchor_desc(7, 0))
            .unwrap();
        let input = StressInput {
            order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(0), CommandId(0)),
            actor: FxActorId(0),
            node: SupportNodeId(1),
            force: Vec2::new(0.0, -1.0),
            source: DamageSource::ContactImpulse,
        };

        let commands = StressSolver2D::new(StressSettings {
            max_iterations: 32,
            damage_per_overload: 1.0,
            ..Default::default()
        })
        .generate(&family, &[input]);

        assert_eq!(commands.len(), 1);
        assert_eq!(commands[0].target, FractureTarget::ExternalBond(anchor));
        assert_eq!(commands[0].source, DamageSource::ContactImpulse);
    }

    #[test]
    fn stress_convergence_epsilon_can_stop_propagation() {
        let family = family_for(asset_from_rows(&["##"], &[Some(0), Some(1)]));
        let solver = StressSolver2D::new(StressSettings {
            convergence_epsilon: 25.0,
            damage_per_overload: 10.0,
            ..Default::default()
        });
        let input = StressInput {
            order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(0), CommandId(0)),
            actor: FxActorId(0),
            node: SupportNodeId(0),
            force: Vec2::new(20.0, 0.0),
            source: DamageSource::Stress,
        };

        assert!(solver.generate(&family, &[input]).is_empty());
    }

    #[test]
    fn stress_gravity_context_static_anchor_generates_external_command() {
        let mut family = family_for(asset_from_rows(&["#"], &[Some(0)]));
        let anchor = family
            .connect_static_anchor(static_anchor_desc(7, 0))
            .unwrap();
        let mut context = StressContext2D::from_family(&family);
        context.gravity = Vec2::new(0.0, -1.0);
        context.fallback_order_keys = vec![(
            FxActorId(0),
            DeterministicOrderKey::new(1, 30, family.id, FxActorId(0), CommandId(0)),
        )];

        let commands = StressSolver2D::new(StressSettings {
            damage_per_overload: 1.0,
            max_iterations: 1,
            ..Default::default()
        })
        .generate_with_context(&family, &context, &[]);

        assert_eq!(commands.len(), 1);
        assert_eq!(commands[0].target, FractureTarget::ExternalBond(anchor));
    }

    #[test]
    fn stress_broken_fixed_endpoint_no_longer_participates() {
        let mut family = family_for(asset_from_rows(&["#"], &[Some(0)]));
        let anchor = family
            .connect_static_anchor(static_anchor_desc(7, 0))
            .unwrap();
        let family_id = family.id;
        apply_fracture_commands(
            &mut family,
            &[FractureCommand {
                order_key: DeterministicOrderKey::new(0, 1, family_id, FxActorId(0), CommandId(0)),
                actor: FxActorId(0),
                target: FractureTarget::ExternalBond(anchor),
                health_loss: 1.0,
                effective_length_loss: 1.0,
                source: DamageSource::Script,
            }],
        );
        let mut context = StressContext2D::from_family(&family);
        context.gravity = Vec2::new(0.0, -1.0);
        context.fallback_order_keys = vec![(
            FxActorId(0),
            DeterministicOrderKey::new(1, 30, family.id, FxActorId(0), CommandId(0)),
        )];

        let commands = StressSolver2D::new(StressSettings {
            damage_per_overload: 1.0,
            max_iterations: 1,
            ..Default::default()
        })
        .generate_with_context(&family, &context, &[]);

        assert!(commands.is_empty());
    }

    #[test]
    fn deterministic_stress_shuffled_inputs_and_context_order_match() {
        let family = family_for(asset_from_rows(&["###"], &[Some(0), Some(1), Some(2)]));
        let solver = StressSolver2D::new(StressSettings {
            damage_per_overload: 10.0,
            max_iterations: 3,
            ..Default::default()
        });
        let inputs = vec![
            StressInput {
                order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(0), CommandId(1)),
                actor: FxActorId(0),
                node: SupportNodeId(2),
                force: Vec2::new(-20.0, 0.0),
                source: DamageSource::Stress,
            },
            StressInput {
                order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(0), CommandId(0)),
                actor: FxActorId(0),
                node: SupportNodeId(0),
                force: Vec2::new(20.0, 0.0),
                source: DamageSource::Stress,
            },
        ];
        let context = StressContext2D::from_family(&family);
        let mut shuffled_inputs = inputs.clone();
        shuffled_inputs.reverse();
        let mut shuffled_context = context.clone();
        shuffled_context.nodes.reverse();

        assert_eq!(
            solver.generate_with_context(&family, &context, &inputs),
            solver.generate_with_context(&family, &shuffled_context, &shuffled_inputs)
        );
    }

    #[test]
    fn deterministic_component_order_independent_of_adjacency() {
        let mut family = family_for(asset_from_rows(&["###"], &[Some(0), Some(1), Some(2)]));
        let commands = vec![
            FractureCommand {
                order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(0), CommandId(1)),
                actor: FxActorId(0),
                target: FractureTarget::Bond(BondId(1)),
                health_loss: 10.0,
                effective_length_loss: 1.0,
                source: DamageSource::Script,
            },
            FractureCommand {
                order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(0), CommandId(0)),
                actor: FxActorId(0),
                target: FractureTarget::Bond(BondId(0)),
                health_loss: 10.0,
                effective_length_loss: 1.0,
                source: DamageSource::Script,
            },
        ];
        apply_fracture_commands(&mut family, &commands);
        let events = split_dirty_actors(&mut family);
        assert_eq!(events[0].fragments[0], vec![SupportNodeId(0)]);
        assert_eq!(events[0].fragments[1], vec![SupportNodeId(1)]);
        assert_eq!(events[0].fragments[2], vec![SupportNodeId(2)]);
    }

    #[test]
    fn wrong_actor_target_rejected() {
        let mut family = family_for(asset_from_rows(&["##"], &[Some(0), Some(1)]));
        let before = family.deterministic_state_digest();
        let damage = DamageInput {
            order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(99), CommandId(0)),
            actor: FxActorId(99),
            target: FractureTarget::Bond(BondId(0)),
            health_loss: 10.0,
            effective_length_loss: 1.0,
            source: DamageSource::Script,
            position: Vec2::ZERO,
            radius: 0.0,
        };
        assert!(generate_damage_commands(&family, &[damage]).is_empty());
        let command = FractureCommand {
            order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(99), CommandId(0)),
            actor: FxActorId(99),
            target: FractureTarget::Bond(BondId(0)),
            health_loss: 10.0,
            effective_length_loss: 1.0,
            source: DamageSource::Script,
        };
        assert!(apply_fracture_commands(&mut family, &[command]).is_empty());
        assert_eq!(before, family.deterministic_state_digest());
    }

    #[test]
    fn stale_command_after_split_rejected() {
        let mut family = family_for(chain_asset());
        let split_command = FractureCommand {
            order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(0), CommandId(0)),
            actor: FxActorId(0),
            target: FractureTarget::Bond(BondId(1)),
            health_loss: 10.0,
            effective_length_loss: 1.0,
            source: DamageSource::Script,
        };
        apply_fracture_commands(&mut family, &[split_command]);
        split_dirty_actors(&mut family);
        let before = family.deterministic_state_digest();
        let stale = FractureCommand {
            order_key: DeterministicOrderKey::new(2, 1, family.id, FxActorId(1), CommandId(0)),
            actor: FxActorId(1),
            target: FractureTarget::Bond(BondId(0)),
            health_loss: 10.0,
            effective_length_loss: 1.0,
            source: DamageSource::Script,
        };
        assert!(apply_fracture_commands(&mut family, &[stale]).is_empty());
        assert_eq!(before, family.deterministic_state_digest());
    }

    #[test]
    fn negative_loss_does_not_heal() {
        let mut family = family_for(asset_from_rows(&["##"], &[Some(0), Some(1)]));
        let damage = FractureCommand {
            order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(0), CommandId(0)),
            actor: FxActorId(0),
            target: FractureTarget::Bond(BondId(0)),
            health_loss: 2.0,
            effective_length_loss: 0.25,
            source: DamageSource::Script,
        };
        apply_fracture_commands(&mut family, &[damage]);
        let before = family.deterministic_state_digest();
        let health = family.bond_state(BondId(0)).unwrap().health;
        let effective_length = family.bond_state(BondId(0)).unwrap().effective_length;
        let negative = FractureCommand {
            order_key: DeterministicOrderKey::new(2, 1, family.id, FxActorId(0), CommandId(0)),
            actor: FxActorId(0),
            target: FractureTarget::Bond(BondId(0)),
            health_loss: -5.0,
            effective_length_loss: -5.0,
            source: DamageSource::Script,
        };
        assert!(apply_fracture_commands(&mut family, &[negative]).is_empty());
        assert_eq!(health, family.bond_state(BondId(0)).unwrap().health);
        assert_eq!(
            effective_length,
            family.bond_state(BondId(0)).unwrap().effective_length
        );
        assert_eq!(before, family.deterministic_state_digest());
    }

    #[test]
    fn node_target_effective_length_only_is_noop() {
        let mut family = family_for(asset_from_rows(&["#"], &[Some(0)]));
        let before = family.deterministic_state_digest();
        let command = FractureCommand {
            order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(0), CommandId(0)),
            actor: FxActorId(0),
            target: FractureTarget::Node(SupportNodeId(0)),
            health_loss: 0.0,
            effective_length_loss: 1.0,
            source: DamageSource::Script,
        };
        let events = apply_fracture_commands(&mut family, &[command]);
        assert!(events.is_empty());
        assert!(!family.is_dirty(FxActorId(0)));
        assert_eq!(before, family.deterministic_state_digest());
    }

    #[test]
    fn apply_fracture_mutates_visible_chunk_health() {
        let occupancy = DenseOccupancy::from_rows(&["##"]).unwrap();
        let mut desc = FxAssetDesc::new(FxAssetId(1), 1.0, occupancy, map(2, &[Some(0), Some(1)]));
        desc.authored_chunks = Some(vec![
            Chunk2D {
                id: ChunkId(10),
                support_nodes: vec![],
                parent: Some(ChunkId(20)),
            },
            Chunk2D {
                id: ChunkId(20),
                support_nodes: vec![SupportNodeId(0), SupportNodeId(1)],
                parent: None,
            },
        ]);
        let mut family = family_for(FxAsset::from_desc(desc).unwrap());
        let family_id = family.id;

        let events = apply_fracture_commands(
            &mut family,
            &[FractureCommand {
                order_key: DeterministicOrderKey::new(1, 1, family_id, FxActorId(0), CommandId(0)),
                actor: FxActorId(0),
                target: FractureTarget::Chunk(ChunkId(10)),
                health_loss: 0.25,
                effective_length_loss: 1.0,
                source: DamageSource::Script,
            }],
        );

        assert_eq!(events.len(), 1);
        assert_eq!(events[0].target, FractureTarget::Chunk(ChunkId(10)));
        assert_eq!(events[0].old_health, 1.0);
        assert_eq!(events[0].new_health, 0.75);
        assert_eq!(family.chunk_state(ChunkId(10)).unwrap().health, 0.75);
        assert_eq!(family.node_state(SupportNodeId(0)).unwrap().health, 1.0);
    }

    #[test]
    fn deterministic_digest_changes_when_chunk_health_changes() {
        let occupancy = DenseOccupancy::from_rows(&["##"]).unwrap();
        let mut desc = FxAssetDesc::new(FxAssetId(1), 1.0, occupancy, map(2, &[Some(0), Some(1)]));
        desc.authored_chunks = Some(vec![
            Chunk2D {
                id: ChunkId(10),
                support_nodes: vec![],
                parent: Some(ChunkId(20)),
            },
            Chunk2D {
                id: ChunkId(20),
                support_nodes: vec![SupportNodeId(0), SupportNodeId(1)],
                parent: None,
            },
        ]);
        let mut family = family_for(FxAsset::from_desc(desc).unwrap());
        let before = family.deterministic_state_digest();
        let family_id = family.id;

        apply_fracture_commands(
            &mut family,
            &[FractureCommand {
                order_key: DeterministicOrderKey::new(1, 1, family_id, FxActorId(0), CommandId(0)),
                actor: FxActorId(0),
                target: FractureTarget::Chunk(ChunkId(10)),
                health_loss: 0.25,
                effective_length_loss: 0.0,
                source: DamageSource::Script,
            }],
        );

        assert_ne!(before, family.deterministic_state_digest());
    }

    #[test]
    fn deterministic_digest_changes_when_effective_length_changes() {
        let mut family = family_for(asset_from_rows(&["##"], &[Some(0), Some(1)]));
        let before = family.deterministic_state_digest();
        let command = FractureCommand {
            order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(0), CommandId(0)),
            actor: FxActorId(0),
            target: FractureTarget::Bond(BondId(0)),
            health_loss: 0.0,
            effective_length_loss: 0.25,
            source: DamageSource::Script,
        };
        apply_fracture_commands(&mut family, &[command]);
        assert_ne!(before, family.deterministic_state_digest());
    }

    #[test]
    fn deterministic_digest_changes_when_actor_ownership_splits() {
        let mut family = family_for(chain_asset());
        let before = family.deterministic_state_digest();
        let command = FractureCommand {
            order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(0), CommandId(0)),
            actor: FxActorId(0),
            target: FractureTarget::Bond(BondId(1)),
            health_loss: 10.0,
            effective_length_loss: 1.0,
            source: DamageSource::Script,
        };
        apply_fracture_commands(&mut family, &[command]);
        split_dirty_actors(&mut family);
        assert_ne!(before, family.deterministic_state_digest());
        assert_eq!(family.node_owner(SupportNodeId(2)), Some(FxActorId(1)));
    }

    #[test]
    fn deterministic_digest_changes_when_event_allocator_state_changes() {
        let mut family = family_for(asset_from_rows(&["##"], &[Some(0), Some(1)]));
        let before = family.deterministic_state_digest();
        assert_eq!(family.next_event_id_for_test(), 0);
        let command = FractureCommand {
            order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(0), CommandId(0)),
            actor: FxActorId(0),
            target: FractureTarget::Bond(BondId(0)),
            health_loss: 1.0,
            effective_length_loss: 0.0,
            source: DamageSource::Script,
        };
        apply_fracture_commands(&mut family, &[command]);
        assert_eq!(family.next_event_id_for_test(), 1);
        assert_ne!(before, family.deterministic_state_digest());
    }

    #[test]
    fn static_anchor_stress_fixed_endpoint() {
        let mut family = family_for(asset_from_rows(&["#"], &[Some(0)]));
        let anchor = family
            .connect_static_anchor(static_anchor_desc(7, 0))
            .unwrap();
        let solver = StressSolver2D::new(StressSettings {
            damage_per_overload: 1.0,
            ..Default::default()
        });
        let input = StressInput {
            order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(0), CommandId(0)),
            actor: FxActorId(0),
            node: SupportNodeId(0),
            force: Vec2::new(1.0, 0.0),
            source: DamageSource::Stress,
        };

        let commands = solver.generate(&family, &[input.clone()]);
        assert_eq!(commands.len(), 1);
        assert_eq!(commands[0].target, FractureTarget::ExternalBond(anchor));
        apply_fracture_commands(&mut family, &commands);
        assert!(family.external_bond_state(anchor).unwrap().is_broken());
        assert!(
            solver.generate(&family, &[input]).is_empty(),
            "broken external anchors must stop acting as fixed endpoints"
        );
    }

    #[test]
    fn static_anchor_compression_does_not_break_external_bond() {
        let mut family = family_for(asset_from_rows(&["#"], &[Some(0)]));
        let anchor = family
            .connect_static_anchor(static_anchor_desc(7, 0))
            .unwrap();
        let solver = StressSolver2D::new(StressSettings {
            damage_per_overload: 1.0,
            ..Default::default()
        });
        let input = StressInput {
            order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(0), CommandId(0)),
            actor: FxActorId(0),
            node: SupportNodeId(0),
            force: Vec2::new(-1.0, 0.0),
            source: DamageSource::Stress,
        };

        let commands = solver.generate(&family, &[input]);
        assert!(commands.is_empty());
        assert!(!family.external_bond_state(anchor).unwrap().is_broken());
    }

    #[test]
    fn static_external_bonds_keep_world_bound_fragments_grouped() {
        for (idx, kind) in [
            ExternalTargetKind::World,
            ExternalTargetKind::Static,
            ExternalTargetKind::Kinematic,
        ]
        .into_iter()
        .enumerate()
        {
            let mut family = family_for(asset_from_rows(&["##"], &[Some(0), Some(1)]));
            let target = ExternalTarget2D {
                kind,
                token: ExternalTargetToken(0),
            };
            let mut anchor_a = static_anchor_desc(7 + idx as u32 * 2, 0);
            anchor_a.target = target;
            let mut anchor_b = static_anchor_desc(8 + idx as u32 * 2, 1);
            anchor_b.target = target;
            family.connect_static_anchor(anchor_a).unwrap();
            family.connect_static_anchor(anchor_b).unwrap();
            let family_id = family.id;
            apply_fracture_commands(
                &mut family,
                &[break_bond_command(1, family_id, FxActorId(0), BondId(0))],
            );

            assert_eq!(
                component_without_bond(
                    family.actor(FxActorId(0)).unwrap(),
                    &family,
                    SupportNodeId(0),
                    None,
                ),
                vec![SupportNodeId(0)],
                "stress/internal component traversal must not cross external endpoints"
            );
            assert!(
                split_dirty_actors(&mut family).is_empty(),
                "live bonds to the same exact fixed target must keep split grouping connected"
            );
            assert_eq!(family.actor_count(), 1);
            assert_eq!(
                family.actor(FxActorId(0)).unwrap().owned_nodes,
                vec![SupportNodeId(0), SupportNodeId(1)]
            );
        }
    }

    #[test]
    fn broken_external_bond_does_not_group_actor() {
        let mut family = family_for(asset_from_rows(&["##"], &[Some(0), Some(1)]));
        family
            .connect_static_anchor(static_anchor_desc(7, 0))
            .unwrap();
        family
            .connect_static_anchor(static_anchor_desc(8, 1))
            .unwrap();
        let family_id = family.id;
        apply_fracture_commands(
            &mut family,
            &[break_external_bond_command(
                1,
                family_id,
                FxActorId(0),
                ExternalBondId(8),
            )],
        );
        assert!(
            family
                .external_bond_state(ExternalBondId(8))
                .unwrap()
                .is_broken()
        );
        apply_fracture_commands(
            &mut family,
            &[break_bond_command(2, family_id, FxActorId(0), BondId(0))],
        );

        let events = split_dirty_actors(&mut family);
        assert_eq!(events.len(), 1);
        assert_eq!(
            events[0].fragments,
            vec![vec![SupportNodeId(0)], vec![SupportNodeId(1)]]
        );
        assert_eq!(events[0].created_children, vec![FxActorId(1)]);
        assert_eq!(family.actor_count(), 2);
    }

    #[test]
    fn breaking_external_bond_after_grouped_noop_marks_actor_dirty() {
        let mut family = family_for(asset_from_rows(&["##"], &[Some(0), Some(1)]));
        family
            .connect_static_anchor(static_anchor_desc(7, 0))
            .unwrap();
        family
            .connect_static_anchor(static_anchor_desc(8, 1))
            .unwrap();
        let family_id = family.id;
        apply_fracture_commands(
            &mut family,
            &[break_bond_command(1, family_id, FxActorId(0), BondId(0))],
        );

        assert!(
            split_dirty_actors(&mut family).is_empty(),
            "same-target external anchors should make the first split pass a no-op"
        );
        assert!(!family.is_dirty(FxActorId(0)));
        assert_eq!(family.actor_count(), 1);

        apply_fracture_commands(
            &mut family,
            &[break_external_bond_command(
                2,
                family_id,
                FxActorId(0),
                ExternalBondId(8),
            )],
        );
        assert!(
            family.is_dirty(FxActorId(0)),
            "breaking the external topology edge must requeue the owning actor for split"
        );

        let events = split_dirty_actors(&mut family);
        assert_eq!(events.len(), 1);
        assert_eq!(
            events[0].fragments,
            vec![vec![SupportNodeId(0)], vec![SupportNodeId(1)]]
        );
        assert_eq!(events[0].created_children, vec![FxActorId(1)]);
        assert_eq!(family.actor_count(), 2);
    }

    #[test]
    fn different_external_targets_do_not_group() {
        let mut family = family_for(asset_from_rows(&["##"], &[Some(0), Some(1)]));
        family
            .connect_static_anchor(static_anchor_desc(7, 0))
            .unwrap();
        let mut other_target = static_anchor_desc(8, 1);
        other_target.target = ExternalTarget2D {
            kind: ExternalTargetKind::World,
            token: ExternalTargetToken(1),
        };
        family.connect_static_anchor(other_target).unwrap();
        let family_id = family.id;
        apply_fracture_commands(
            &mut family,
            &[break_bond_command(1, family_id, FxActorId(0), BondId(0))],
        );

        let events = split_dirty_actors(&mut family);
        assert_eq!(events.len(), 1);
        assert_eq!(
            events[0].fragments,
            vec![vec![SupportNodeId(0)], vec![SupportNodeId(1)]],
            "external target grouping must use full ExternalTarget2D equality, including token"
        );
        assert_eq!(family.actor_count(), 2);
    }

    #[test]
    fn stress_frame_cap_is_per_actor() {
        let mut family = family_for(asset_from_rows(&["#.#"], &[Some(0), None, Some(1)]));
        let _anchor0 = family
            .connect_static_anchor(static_anchor_desc(7, 0))
            .unwrap();
        let _anchor1 = family
            .connect_static_anchor(static_anchor_desc(8, 1))
            .unwrap();
        let solver = StressSolver2D::new(StressSettings {
            damage_per_overload: 1.0,
            max_fractures_per_frame: 1,
            ..Default::default()
        });
        let report = solver.generate_with_profile(
            &family,
            &[
                StressInput {
                    order_key: DeterministicOrderKey::new(
                        1,
                        1,
                        family.id,
                        FxActorId(0),
                        CommandId(2),
                    ),
                    actor: FxActorId(0),
                    node: SupportNodeId(0),
                    force: Vec2::new(1.0, 0.0),
                    source: DamageSource::Stress,
                },
                StressInput {
                    order_key: DeterministicOrderKey::new(
                        1,
                        1,
                        family.id,
                        FxActorId(1),
                        CommandId(1),
                    ),
                    actor: FxActorId(1),
                    node: SupportNodeId(1),
                    force: Vec2::new(1.0, 0.0),
                    source: DamageSource::Stress,
                },
            ],
        );
        assert_eq!(report.profile.input_count, 2);
        assert_eq!(report.profile.actor_count_visited, 2);
        assert_eq!(report.profile.actors_with_input, 2);
        assert_eq!(report.profile.external_bond_candidates, 2);
        assert_eq!(report.profile.external_bonds_tested, 2);
        assert_eq!(report.profile.generated_commands_before_cap, 2);
        assert_eq!(report.profile.generated_commands_after_cap, 2);
        assert_eq!(report.profile.frame_cap, 1);
        assert_eq!(report.commands.len(), 2);
        assert_eq!(solver.generate(&family, &[]).len(), 0);
        assert_eq!(report.commands[0].actor, FxActorId(0));
        assert_eq!(
            report.commands[0].target,
            FractureTarget::ExternalBond(ExternalBondId(7))
        );
        assert_eq!(report.commands[1].actor, FxActorId(1));
        assert_eq!(
            report.commands[1].target,
            FractureTarget::ExternalBond(ExternalBondId(8))
        );
    }

    #[test]
    fn dynamic_bond_graph_only() {
        let mut family = family_for(asset_from_rows(&["#.#"], &[Some(0), None, Some(1)]));
        assert_eq!(family.actor_count(), 2);
        let connection = family
            .connect_dynamic_structural_bond_graph_only(dynamic_graph_only_desc(11, 0, 1))
            .unwrap();
        assert_eq!(family.actor_count(), 2);
        assert_eq!(
            family.dynamic_structural_bond(connection).unwrap().policy,
            DynamicConnectionPolicy::GraphOnly
        );

        let solver = StressSolver2D::new(StressSettings {
            damage_per_overload: 0.5,
            ..Default::default()
        });
        let commands = solver.generate(
            &family,
            &[StressInput {
                order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(0), CommandId(0)),
                actor: FxActorId(0),
                node: SupportNodeId(0),
                force: Vec2::new(1.0, 0.0),
                source: DamageSource::Stress,
            }],
        );
        assert_eq!(commands.len(), 1);
        assert_eq!(commands[0].target, FractureTarget::Connection(connection));
        apply_fracture_commands(&mut family, &commands);
        assert_eq!(
            family
                .dynamic_structural_bond_state(connection)
                .unwrap()
                .health,
            0.5
        );
        assert!(family.is_dirty(FxActorId(0)));
        assert!(family.is_dirty(FxActorId(1)));
        assert!(split_dirty_actors(&mut family).is_empty());
        assert_eq!(family.actor_count(), 2);
    }

    #[test]
    fn connection_validation_rejects_invalid_inputs() {
        let mut family = family_for(asset_from_rows(&["#.#"], &[Some(0), None, Some(1)]));
        assert_eq!(
            family
                .connect_static_anchor(static_anchor_desc(1, 99))
                .unwrap_err(),
            ConnectionError::UnknownNode(SupportNodeId(99))
        );
        let mut invalid_anchor = static_anchor_desc(1, 0);
        invalid_anchor.health = f32::NAN;
        assert_eq!(
            family.connect_static_anchor(invalid_anchor).unwrap_err(),
            ConnectionError::InvalidExternalBondRuntime(ExternalBondId(1))
        );
        family
            .connect_static_anchor(static_anchor_desc(1, 0))
            .unwrap();
        assert_eq!(
            family
                .connect_static_anchor(static_anchor_desc(1, 1))
                .unwrap_err(),
            ConnectionError::DuplicateExternalBond(ExternalBondId(1))
        );

        assert_eq!(
            family
                .connect_dynamic_structural_bond_graph_only(dynamic_graph_only_desc(2, 0, 0))
                .unwrap_err(),
            ConnectionError::SelfConnection(SupportNodeId(0))
        );
        let mut invalid_dynamic = dynamic_graph_only_desc(2, 0, 1);
        invalid_dynamic.effective_length = -1.0;
        assert_eq!(
            family
                .connect_dynamic_structural_bond_graph_only(invalid_dynamic)
                .unwrap_err(),
            ConnectionError::InvalidConnectionRuntime(ConnectionId(2))
        );
        family
            .connect_dynamic_structural_bond_graph_only(dynamic_graph_only_desc(2, 0, 1))
            .unwrap();
        assert_eq!(
            family
                .connect_dynamic_structural_bond_graph_only(dynamic_graph_only_desc(2, 1, 0))
                .unwrap_err(),
            ConnectionError::DuplicateConnection(ConnectionId(2))
        );
    }

    #[test]
    fn same_family_actor_merge_preserves_connection_state() {
        let mut family = family_for(asset_from_rows(&["#.#"], &[Some(0), None, Some(1)]));
        family
            .connect_static_anchor(static_anchor_desc(3, 1))
            .unwrap();
        let family_id = family.id;
        let connection = family
            .connect_dynamic_structural_bond_graph_only(dynamic_graph_only_desc(4, 0, 1))
            .unwrap();
        apply_fracture_commands(
            &mut family,
            &[FractureCommand {
                order_key: DeterministicOrderKey::new(1, 1, family_id, FxActorId(0), CommandId(0)),
                actor: FxActorId(0),
                target: FractureTarget::Connection(connection),
                health_loss: 0.25,
                effective_length_loss: 0.0,
                source: DamageSource::Script,
            }],
        );

        let result = family.merge_actors(FxActorId(1), FxActorId(0)).unwrap();
        assert_eq!(result.kept_actor, FxActorId(0));
        assert_eq!(result.removed_actor, FxActorId(1));
        assert_eq!(family.actor_count(), 1);
        assert_eq!(family.node_owner(SupportNodeId(0)), Some(FxActorId(0)));
        assert_eq!(family.node_owner(SupportNodeId(1)), Some(FxActorId(0)));
        assert!(!family.is_dirty(FxActorId(0)));
        assert_eq!(
            actor_components(family.actor(FxActorId(0)).unwrap(), &family).len(),
            1
        );
        assert_eq!(
            family
                .external_bond_state(ExternalBondId(3))
                .unwrap()
                .health,
            1.0
        );
        assert_eq!(
            family
                .dynamic_structural_bond_state(connection)
                .unwrap()
                .health,
            0.75
        );
    }

    #[test]
    fn merge_preserves_dirty_split_obligation() {
        let mut family = family_for(asset_from_rows(
            &["##.#"],
            &[Some(0), Some(1), None, Some(2)],
        ));
        let family_id = family.id;
        family
            .connect_dynamic_structural_bond_graph_only(dynamic_graph_only_desc(12, 0, 2))
            .unwrap();
        apply_fracture_commands(
            &mut family,
            &[FractureCommand {
                order_key: DeterministicOrderKey::new(1, 1, family_id, FxActorId(0), CommandId(0)),
                actor: FxActorId(0),
                target: FractureTarget::Bond(BondId(0)),
                health_loss: 10.0,
                effective_length_loss: 1.0,
                source: DamageSource::Script,
            }],
        );
        assert!(family.is_dirty(FxActorId(0)));

        let result = family.merge_actors(FxActorId(0), FxActorId(1)).unwrap();
        assert_eq!(result.kept_actor, FxActorId(0));
        assert!(family.is_dirty(FxActorId(0)));
        assert!(!family.is_dirty(FxActorId(1)));

        let events = split_dirty_actors(&mut family);
        assert_eq!(events.len(), 1);
        assert_eq!(events[0].parent_actor, FxActorId(0));
        assert_eq!(events[0].fragments.len(), 2);
    }

    #[test]
    fn graph_only_connection_stress_generates_once_for_two_endpoint_inputs() {
        let mut family = family_for(asset_from_rows(&["#.#"], &[Some(0), None, Some(1)]));
        let connection = family
            .connect_dynamic_structural_bond_graph_only(dynamic_graph_only_desc(8, 0, 1))
            .unwrap();
        let solver = StressSolver2D::new(StressSettings {
            damage_per_overload: 0.25,
            ..Default::default()
        });
        let commands = solver.generate(
            &family,
            &[
                StressInput {
                    order_key: DeterministicOrderKey::new(
                        1,
                        1,
                        family.id,
                        FxActorId(0),
                        CommandId(0),
                    ),
                    actor: FxActorId(0),
                    node: SupportNodeId(0),
                    force: Vec2::new(1.0, 0.0),
                    source: DamageSource::Stress,
                },
                StressInput {
                    order_key: DeterministicOrderKey::new(
                        1,
                        1,
                        family.id,
                        FxActorId(1),
                        CommandId(1),
                    ),
                    actor: FxActorId(1),
                    node: SupportNodeId(1),
                    force: Vec2::new(-1.0, 0.0),
                    source: DamageSource::Stress,
                },
            ],
        );
        assert_eq!(commands.len(), 1);
        assert_eq!(commands[0].target, FractureTarget::Connection(connection));
        let events = apply_fracture_commands(&mut family, &commands);
        assert_eq!(events.len(), 1);
        assert_eq!(
            family
                .dynamic_structural_bond_state(connection)
                .unwrap()
                .health,
            0.75
        );
    }

    #[test]
    fn graph_only_connection_compression_does_not_break() {
        let mut family = family_for(asset_from_rows(&["#.#"], &[Some(0), None, Some(1)]));
        family
            .connect_dynamic_structural_bond_graph_only(dynamic_graph_only_desc(8, 0, 1))
            .unwrap();
        let solver = StressSolver2D::new(StressSettings {
            damage_per_overload: 0.25,
            ..Default::default()
        });
        let commands = solver.generate(
            &family,
            &[
                StressInput {
                    order_key: DeterministicOrderKey::new(
                        1,
                        1,
                        family.id,
                        FxActorId(0),
                        CommandId(0),
                    ),
                    actor: FxActorId(0),
                    node: SupportNodeId(0),
                    force: Vec2::new(-1.0, 0.0),
                    source: DamageSource::Stress,
                },
                StressInput {
                    order_key: DeterministicOrderKey::new(
                        1,
                        1,
                        family.id,
                        FxActorId(1),
                        CommandId(1),
                    ),
                    actor: FxActorId(1),
                    node: SupportNodeId(1),
                    force: Vec2::new(1.0, 0.0),
                    source: DamageSource::Stress,
                },
            ],
        );
        assert!(commands.is_empty());
    }

    #[test]
    fn graph_only_connection_mixed_two_sided_modes_use_mode_wise_limits() {
        let mut family = family_for(asset_from_rows(&["#.#"], &[Some(0), None, Some(1)]));
        let mut desc = dynamic_graph_only_desc(8, 0, 1);
        desc.tension_limit = 10.0;
        desc.shear_limit = 100.0;
        let connection = family
            .connect_dynamic_structural_bond_graph_only(desc)
            .unwrap();
        let solver = StressSolver2D::new(StressSettings {
            damage_per_overload: 0.25,
            ..Default::default()
        });
        let commands = solver.generate(
            &family,
            &[
                StressInput {
                    order_key: DeterministicOrderKey::new(
                        1,
                        1,
                        family.id,
                        FxActorId(0),
                        CommandId(0),
                    ),
                    actor: FxActorId(0),
                    node: SupportNodeId(0),
                    force: Vec2::new(20.0, 0.0),
                    source: DamageSource::Stress,
                },
                StressInput {
                    order_key: DeterministicOrderKey::new(
                        1,
                        1,
                        family.id,
                        FxActorId(1),
                        CommandId(1),
                    ),
                    actor: FxActorId(1),
                    node: SupportNodeId(1),
                    force: Vec2::new(0.0, 50.0),
                    source: DamageSource::Stress,
                },
            ],
        );
        assert_eq!(commands.len(), 1);
        assert_eq!(commands[0].target, FractureTarget::Connection(connection));
        assert_eq!(commands[0].effective_length_loss, 1.0);
    }

    #[test]
    fn repair_plan_rejects_removed_external_bond_endpoint() {
        let mut family = family_for(asset_from_rows(&["##"], &[Some(0), Some(1)]));
        family
            .connect_static_anchor(static_anchor_desc(3, 1))
            .unwrap();
        let asset = asset_from_rows(&["#"], &[Some(0)]);
        let node_states = vec![(
            SupportNodeId(0),
            family.node_state(SupportNodeId(0)).unwrap().clone(),
        )];
        let err = family
            .apply_repair_plan(RepairPlan {
                asset,
                node_owners: vec![(SupportNodeId(0), FxActorId(0))],
                node_states,
                bond_states: vec![],
                dirty_actors: vec![FxActorId(0)],
            })
            .unwrap_err();
        assert_eq!(
            err,
            RepairError::StaleExternalBondEndpoint {
                bond: ExternalBondId(3),
                node: SupportNodeId(1),
            }
        );
    }

    #[test]
    fn repair_plan_rejects_removed_dynamic_connection_endpoint() {
        let mut family = family_for(asset_from_rows(&["##"], &[Some(0), Some(1)]));
        family
            .connect_dynamic_structural_bond_graph_only(dynamic_graph_only_desc(4, 0, 1))
            .unwrap();
        let asset = asset_from_rows(&["#"], &[Some(0)]);
        let node_states = vec![(
            SupportNodeId(0),
            family.node_state(SupportNodeId(0)).unwrap().clone(),
        )];
        let err = family
            .apply_repair_plan(RepairPlan {
                asset,
                node_owners: vec![(SupportNodeId(0), FxActorId(0))],
                node_states,
                bond_states: vec![],
                dirty_actors: vec![FxActorId(0)],
            })
            .unwrap_err();
        assert_eq!(
            err,
            RepairError::StaleDynamicConnectionEndpoint {
                connection: ConnectionId(4),
                node: SupportNodeId(1),
            }
        );
    }

    #[test]
    fn merge_actors_requires_unbroken_graph_connection() {
        let mut family = family_for(asset_from_rows(&["#.#"], &[Some(0), None, Some(1)]));
        let before = family.deterministic_state_digest();
        let err = family.merge_actors(FxActorId(0), FxActorId(1)).unwrap_err();
        assert_eq!(
            err,
            ConnectionError::MissingMergeConnection {
                actor_a: FxActorId(0),
                actor_b: FxActorId(1),
            }
        );
        assert_eq!(family.deterministic_state_digest(), before);
        assert_eq!(family.actor_count(), 2);

        let connection = family
            .connect_dynamic_structural_bond_graph_only(dynamic_graph_only_desc(9, 0, 1))
            .unwrap();
        apply_fracture_commands(
            &mut family,
            &[FractureCommand {
                order_key: DeterministicOrderKey::new(
                    1,
                    1,
                    FxFamilyId(3),
                    FxActorId(0),
                    CommandId(0),
                ),
                actor: FxActorId(0),
                target: FractureTarget::Connection(connection),
                health_loss: 1.0,
                effective_length_loss: 1.0,
                source: DamageSource::Script,
            }],
        );
        let before_broken_merge = family.deterministic_state_digest();
        let err = family.merge_actors(FxActorId(0), FxActorId(1)).unwrap_err();
        assert_eq!(
            err,
            ConnectionError::MissingMergeConnection {
                actor_a: FxActorId(0),
                actor_b: FxActorId(1),
            }
        );
        assert_eq!(family.deterministic_state_digest(), before_broken_merge);
        assert_eq!(family.actor_count(), 2);
    }

    #[test]
    fn broken_external_and_dynamic_connections_ignore_direct_damage() {
        let mut external_family = family_for(asset_from_rows(&["#"], &[Some(0)]));
        let external = external_family
            .connect_static_anchor(static_anchor_desc(1, 0))
            .unwrap();
        let external_family_id = external_family.id;
        apply_fracture_commands(
            &mut external_family,
            &[FractureCommand {
                order_key: DeterministicOrderKey::new(
                    1,
                    1,
                    external_family_id,
                    FxActorId(0),
                    CommandId(0),
                ),
                actor: FxActorId(0),
                target: FractureTarget::ExternalBond(external),
                health_loss: 1.0,
                effective_length_loss: 1.0,
                source: DamageSource::Script,
            }],
        );
        assert!(
            external_family
                .external_bond_state(external)
                .unwrap()
                .is_broken()
        );
        let before = external_family.deterministic_state_digest();
        let external_damage = DamageInput {
            order_key: DeterministicOrderKey::new(
                2,
                1,
                external_family.id,
                FxActorId(0),
                CommandId(0),
            ),
            actor: FxActorId(0),
            target: FractureTarget::ExternalBond(external),
            health_loss: 1.0,
            effective_length_loss: 1.0,
            source: DamageSource::Script,
            position: Vec2::ZERO,
            radius: 0.0,
        };
        assert!(generate_damage_commands(&external_family, &[external_damage.clone()]).is_empty());
        assert!(
            generate_damage_commands(
                &external_family,
                &[DamageInput {
                    target: FractureTarget::Node(SupportNodeId(0)),
                    ..external_damage.clone()
                }],
            )
            .is_empty()
        );
        assert!(
            apply_fracture_commands(
                &mut external_family,
                &[FractureCommand {
                    order_key: DeterministicOrderKey::new(
                        3,
                        1,
                        FxFamilyId(3),
                        FxActorId(0),
                        CommandId(0),
                    ),
                    actor: FxActorId(0),
                    target: FractureTarget::ExternalBond(external),
                    health_loss: 1.0,
                    effective_length_loss: 1.0,
                    source: DamageSource::Script,
                }],
            )
            .is_empty()
        );
        assert_eq!(external_family.deterministic_state_digest(), before);

        let mut dynamic_family = family_for(asset_from_rows(&["#.#"], &[Some(0), None, Some(1)]));
        let connection = dynamic_family
            .connect_dynamic_structural_bond_graph_only(dynamic_graph_only_desc(2, 0, 1))
            .unwrap();
        let dynamic_family_id = dynamic_family.id;
        apply_fracture_commands(
            &mut dynamic_family,
            &[FractureCommand {
                order_key: DeterministicOrderKey::new(
                    1,
                    1,
                    dynamic_family_id,
                    FxActorId(0),
                    CommandId(0),
                ),
                actor: FxActorId(0),
                target: FractureTarget::Connection(connection),
                health_loss: 1.0,
                effective_length_loss: 1.0,
                source: DamageSource::Script,
            }],
        );
        assert!(
            dynamic_family
                .dynamic_structural_bond_state(connection)
                .unwrap()
                .is_broken()
        );
        let before = dynamic_family.deterministic_state_digest();
        let dynamic_damage = DamageInput {
            order_key: DeterministicOrderKey::new(
                2,
                1,
                dynamic_family.id,
                FxActorId(0),
                CommandId(0),
            ),
            actor: FxActorId(0),
            target: FractureTarget::Connection(connection),
            health_loss: 1.0,
            effective_length_loss: 1.0,
            source: DamageSource::Script,
            position: Vec2::ZERO,
            radius: 0.0,
        };
        assert!(generate_damage_commands(&dynamic_family, &[dynamic_damage.clone()]).is_empty());
        assert!(
            generate_damage_commands(
                &dynamic_family,
                &[DamageInput {
                    target: FractureTarget::Node(SupportNodeId(0)),
                    ..dynamic_damage.clone()
                }],
            )
            .is_empty()
        );
        assert!(
            apply_fracture_commands(
                &mut dynamic_family,
                &[FractureCommand {
                    order_key: DeterministicOrderKey::new(
                        3,
                        1,
                        FxFamilyId(3),
                        FxActorId(0),
                        CommandId(0),
                    ),
                    actor: FxActorId(0),
                    target: FractureTarget::Connection(connection),
                    health_loss: 1.0,
                    effective_length_loss: 1.0,
                    source: DamageSource::Script,
                }],
            )
            .is_empty()
        );
        assert_eq!(dynamic_family.deterministic_state_digest(), before);
    }

    #[test]
    fn phase4_digest_includes_connection_state() {
        fn digest_with_external(desc: StaticAnchorDesc) -> u64 {
            let mut family = family_for(asset_from_rows(&["#"], &[Some(0)]));
            family.connect_static_anchor(desc).unwrap();
            family.deterministic_state_digest()
        }

        fn digest_with_dynamic(desc: DynamicStructuralBondDesc) -> u64 {
            let mut family = family_for(disconnected_three_node_asset());
            family
                .connect_dynamic_structural_bond_graph_only(desc)
                .unwrap();
            family.deterministic_state_digest()
        }

        let external_base = static_anchor_desc(1, 0);
        let external_base_digest = digest_with_external(external_base.clone());

        let mut external_target = external_base.clone();
        external_target.target = ExternalTarget2D {
            kind: ExternalTargetKind::Static,
            token: ExternalTargetToken(42),
        };
        assert_ne!(external_base_digest, digest_with_external(external_target));

        let mut external_anchor = external_base.clone();
        external_anchor.anchor = Vec2::new(9.0, 2.0);
        assert_ne!(external_base_digest, digest_with_external(external_anchor));

        let mut external_normal = external_base.clone();
        external_normal.normal = Vec2::new(0.0, 1.0);
        assert_ne!(external_base_digest, digest_with_external(external_normal));

        let mut external_limits = external_base.clone();
        external_limits.tension_limit = 0.5;
        external_limits.shear_limit = 0.75;
        assert_ne!(external_base_digest, digest_with_external(external_limits));

        let mut external_runtime_family = family_for(asset_from_rows(&["#"], &[Some(0)]));
        external_runtime_family
            .connect_static_anchor(external_base)
            .unwrap();
        let before_external_runtime = external_runtime_family.deterministic_state_digest();
        apply_fracture_commands(
            &mut external_runtime_family,
            &[FractureCommand {
                order_key: DeterministicOrderKey::new(
                    1,
                    1,
                    FxFamilyId(3),
                    FxActorId(0),
                    CommandId(0),
                ),
                actor: FxActorId(0),
                target: FractureTarget::ExternalBond(ExternalBondId(1)),
                health_loss: 0.25,
                effective_length_loss: 0.25,
                source: DamageSource::Script,
            }],
        );
        assert_ne!(
            before_external_runtime,
            external_runtime_family.deterministic_state_digest()
        );

        let dynamic_base = dynamic_graph_only_desc(5, 0, 1);
        let dynamic_base_digest = digest_with_dynamic(dynamic_base.clone());

        let dynamic_endpoint = dynamic_graph_only_desc(5, 0, 2);
        assert_ne!(dynamic_base_digest, digest_with_dynamic(dynamic_endpoint));

        let mut dynamic_centroid = dynamic_base.clone();
        dynamic_centroid.centroid = Vec2::new(4.0, 3.0);
        assert_ne!(dynamic_base_digest, digest_with_dynamic(dynamic_centroid));

        let mut dynamic_normal = dynamic_base.clone();
        dynamic_normal.normal = Vec2::new(0.0, 1.0);
        assert_ne!(dynamic_base_digest, digest_with_dynamic(dynamic_normal));

        let mut dynamic_limits = dynamic_base.clone();
        dynamic_limits.tension_limit = 0.5;
        dynamic_limits.shear_limit = 0.75;
        assert_ne!(dynamic_base_digest, digest_with_dynamic(dynamic_limits));

        let mut dynamic_policy_family = family_for(disconnected_three_node_asset());
        dynamic_policy_family
            .connect_dynamic_structural_bond_graph_only(dynamic_base.clone())
            .unwrap();
        let before_dynamic_policy = dynamic_policy_family.deterministic_state_digest();
        dynamic_policy_family
            .dynamic_structural_bonds
            .get_mut(&ConnectionId(5))
            .unwrap()
            .policy = DynamicConnectionPolicy::CustomHardConstraint;
        assert_ne!(
            before_dynamic_policy,
            dynamic_policy_family.deterministic_state_digest()
        );

        let mut dynamic_runtime_family = family_for(disconnected_three_node_asset());
        dynamic_runtime_family
            .connect_dynamic_structural_bond_graph_only(dynamic_base)
            .unwrap();
        let before_dynamic_runtime = dynamic_runtime_family.deterministic_state_digest();
        apply_fracture_commands(
            &mut dynamic_runtime_family,
            &[FractureCommand {
                order_key: DeterministicOrderKey::new(
                    1,
                    1,
                    FxFamilyId(3),
                    FxActorId(0),
                    CommandId(0),
                ),
                actor: FxActorId(0),
                target: FractureTarget::Connection(ConnectionId(5)),
                health_loss: 0.25,
                effective_length_loss: 0.25,
                source: DamageSource::Script,
            }],
        );
        assert_ne!(
            before_dynamic_runtime,
            dynamic_runtime_family.deterministic_state_digest()
        );

        let build = |reverse: bool| {
            let mut family = family_for(asset_from_rows(&["#.#"], &[Some(0), None, Some(1)]));
            let static_a = static_anchor_desc(2, 0);
            let static_b = static_anchor_desc(1, 1);
            let dynamic_a = dynamic_graph_only_desc(6, 0, 1);
            let dynamic_b = dynamic_graph_only_desc(5, 0, 1);
            if reverse {
                family.connect_static_anchor(static_a).unwrap();
                family
                    .connect_dynamic_structural_bond_graph_only(dynamic_a)
                    .unwrap();
                family.connect_static_anchor(static_b).unwrap();
                family
                    .connect_dynamic_structural_bond_graph_only(dynamic_b)
                    .unwrap();
            } else {
                family
                    .connect_dynamic_structural_bond_graph_only(dynamic_b)
                    .unwrap();
                family.connect_static_anchor(static_b).unwrap();
                family
                    .connect_dynamic_structural_bond_graph_only(dynamic_a)
                    .unwrap();
                family.connect_static_anchor(static_a).unwrap();
            }
            family
        };
        let mut a = build(false);
        let b = build(true);
        assert_eq!(
            a.deterministic_state_digest(),
            b.deterministic_state_digest()
        );

        let before = a.deterministic_state_digest();
        let family_id = a.id;
        apply_fracture_commands(
            &mut a,
            &[FractureCommand {
                order_key: DeterministicOrderKey::new(1, 1, family_id, FxActorId(1), CommandId(0)),
                actor: FxActorId(1),
                target: FractureTarget::ExternalBond(ExternalBondId(1)),
                health_loss: 0.25,
                effective_length_loss: 0.0,
                source: DamageSource::Script,
            }],
        );
        assert_ne!(before, a.deterministic_state_digest());
    }

    #[test]
    fn initial_disconnected_asset_instantiates_stable_actor_islands() {
        let family = family_for(asset_from_rows(&["#.#"], &[Some(0), None, Some(1)]));
        assert_eq!(family.actor_count(), 2);
        assert_eq!(
            family.actor(FxActorId(0)).unwrap().owned_nodes,
            vec![SupportNodeId(0)]
        );
        assert_eq!(
            family.actor(FxActorId(1)).unwrap().owned_nodes,
            vec![SupportNodeId(1)]
        );
        assert_eq!(family.next_actor_id_for_test(), 2);
    }

    #[test]
    fn phase1_thin_slice_generate_apply_stress_split() {
        let asset = asset_from_rows(&["####"], &[Some(0), Some(1), Some(2), Some(3)]);
        let mut family = family_for(asset);
        let start_digest = family.deterministic_state_digest();

        let damage_inputs = vec![
            DamageInput {
                order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(0), CommandId(1)),
                actor: FxActorId(0),
                target: FractureTarget::Bond(BondId(2)),
                health_loss: 9.5,
                effective_length_loss: 0.0,
                source: DamageSource::Script,
                position: Vec2::new(2.5, 0.5),
                radius: 0.0,
            },
            DamageInput {
                order_key: DeterministicOrderKey::new(1, 1, family.id, FxActorId(0), CommandId(0)),
                actor: FxActorId(0),
                target: FractureTarget::Bond(BondId(0)),
                health_loss: 10.0,
                effective_length_loss: 1.0,
                source: DamageSource::Script,
                position: Vec2::new(0.5, 0.5),
                radius: 0.0,
            },
        ];
        let damage_commands = generate_damage_commands(&family, &damage_inputs);
        assert_eq!(damage_commands.len(), 2);
        assert_eq!(start_digest, family.deterministic_state_digest());

        let damage_events = apply_fracture_commands(&mut family, &damage_commands);
        assert_eq!(damage_events.len(), 2);
        assert_eq!(family.bond_state(BondId(0)).unwrap().health, 0.0);
        assert_eq!(family.bond_state(BondId(2)).unwrap().health, 0.5);

        let after_damage_digest = family.deterministic_state_digest();
        assert_ne!(start_digest, after_damage_digest);
        let solver = StressSolver2D::new(StressSettings {
            damage_per_overload: 10.0,
            ..Default::default()
        });
        let stress_inputs = vec![StressInput {
            order_key: DeterministicOrderKey::new(2, 1, family.id, FxActorId(0), CommandId(0)),
            actor: FxActorId(0),
            node: SupportNodeId(3),
            force: Vec2::new(-10.0, 0.0),
            source: DamageSource::ContactImpulse,
        }];
        let stress_commands = solver.generate(&family, &stress_inputs);
        assert_eq!(after_damage_digest, family.deterministic_state_digest());
        assert_eq!(stress_commands.len(), 1);
        assert_eq!(stress_commands[0].target, FractureTarget::Bond(BondId(2)));

        let stress_events = apply_fracture_commands(&mut family, &stress_commands);
        assert_eq!(stress_events.len(), 1);
        assert_eq!(family.bond_state(BondId(2)).unwrap().health, 0.0);

        let split_events = split_dirty_actors(&mut family);
        assert_eq!(split_events.len(), 1);
        assert_eq!(split_events[0].kept_actor, FxActorId(0));
        assert_eq!(
            family.actor(FxActorId(0)).unwrap().owned_nodes,
            vec![SupportNodeId(1), SupportNodeId(2)]
        );
        assert_eq!(
            family.actor(FxActorId(1)).unwrap().owned_nodes,
            vec![SupportNodeId(0)]
        );
        assert_eq!(
            family.actor(FxActorId(2)).unwrap().owned_nodes,
            vec![SupportNodeId(3)]
        );
        assert_ne!(after_damage_digest, family.deterministic_state_digest());
    }
}
