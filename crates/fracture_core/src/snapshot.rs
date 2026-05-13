use std::collections::{BTreeMap, BTreeSet};

use thiserror::Error;

use super::*;

const MAGIC_ASSET: [u8; 8] = *b"RFXSCA\0\0";
const MAGIC_FAMILY: [u8; 8] = *b"RFXSCF\0\0";
const VERSION: u16 = 3;
const DIMENSION_2D: u8 = 2;
const SCALAR_F32: u8 = 4;
const HEADER_LEN: usize = 34;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SnapshotMode {
    Normal = 0,
    Deterministic = 1,
}

impl SnapshotMode {
    pub fn from_u8(value: u8) -> Result<Self, FxCoreSnapshotError> {
        match value {
            0 => Ok(Self::Normal),
            1 => Ok(Self::Deterministic),
            other => Err(FxCoreSnapshotError::UnsupportedMode(other)),
        }
    }
}

#[derive(Error, Debug, Clone, PartialEq)]
pub enum FxCoreSnapshotError {
    #[error("snapshot magic is invalid")]
    InvalidMagic,
    #[error("snapshot version {0} is unsupported")]
    UnsupportedVersion(u16),
    #[error("snapshot dimension {0} is unsupported")]
    UnsupportedDimension(u8),
    #[error("snapshot scalar width {0} is unsupported")]
    UnsupportedScalar(u8),
    #[error("snapshot mode {0} is unsupported")]
    UnsupportedMode(u8),
    #[error("snapshot flags {0:#x} contain unsupported bits")]
    UnsupportedFlags(u32),
    #[error("snapshot payload length mismatch")]
    PayloadLengthMismatch,
    #[error("snapshot payload checksum mismatch")]
    PayloadChecksumMismatch,
    #[error("snapshot ended while reading {0}")]
    UnexpectedEof(&'static str),
    #[error("snapshot has trailing bytes")]
    TrailingBytes,
    #[error("snapshot value {0} is invalid or non-finite")]
    InvalidValue(&'static str),
    #[error("snapshot state is inconsistent: {0}")]
    StateMismatch(&'static str),
    #[error("snapshot contains unsupported custom hard constraint {0:?}")]
    UnsupportedCustomHardConstraint(ConnectionId),
    #[error(transparent)]
    Validation(#[from] ValidationError),
}

impl FxAsset {
    pub fn to_snapshot_bytes(&self, mode: SnapshotMode) -> Result<Vec<u8>, FxCoreSnapshotError> {
        let mut payload = Writer::new();
        write_asset_payload(&mut payload, self)?;
        Ok(wrap_payload(MAGIC_ASSET, mode, 0, payload.into_inner()))
    }

    pub fn from_snapshot_bytes(bytes: &[u8]) -> Result<Self, FxCoreSnapshotError> {
        let (mode, payload) = unwrap_payload(bytes, MAGIC_ASSET)?;
        let mut reader = Reader::new(payload);
        let asset = read_asset_payload(&mut reader)?;
        reader.finish()?;
        if mode == SnapshotMode::Deterministic {
            asset.validate()?;
        }
        Ok(asset)
    }
}

impl FxFamily {
    pub fn to_snapshot_bytes(&self, mode: SnapshotMode) -> Result<Vec<u8>, FxCoreSnapshotError> {
        let mut payload = Writer::new();
        write_family_payload(&mut payload, self)?;
        Ok(wrap_payload(MAGIC_FAMILY, mode, 0, payload.into_inner()))
    }

    pub fn from_snapshot_bytes(bytes: &[u8]) -> Result<Self, FxCoreSnapshotError> {
        let (_mode, payload) = unwrap_payload(bytes, MAGIC_FAMILY)?;
        let mut reader = Reader::new(payload);
        let family = read_family_payload(&mut reader)?;
        reader.finish()?;
        validate_family_snapshot(&family)?;
        Ok(family)
    }
}

pub fn encode_family_snapshot(
    family: &FxFamily,
    mode: SnapshotMode,
) -> Result<Vec<u8>, FxCoreSnapshotError> {
    family.to_snapshot_bytes(mode)
}

pub fn restore_family_snapshot(bytes: &[u8]) -> Result<FxFamily, FxCoreSnapshotError> {
    FxFamily::from_snapshot_bytes(bytes)
}

pub(crate) fn write_asset_payload(
    writer: &mut Writer,
    asset: &FxAsset,
) -> Result<(), FxCoreSnapshotError> {
    validate_asset_finite(asset)?;
    writer.u32(asset.id.0);
    writer.f32(asset.voxel_size, "asset.voxel_size")?;
    writer.u32(asset.occupancy.width);
    writer.u32(asset.occupancy.height);
    writer.vec_len(asset.occupancy.cells.len())?;
    for cell in &asset.occupancy.cells {
        writer.u8(u8::from(*cell));
    }
    writer.vec_len(asset.voxel_to_node.len())?;
    for node in &asset.voxel_to_node {
        write_opt_id(writer, node.map(|id| id.0));
    }
    writer.vec_len(asset.support_nodes.len())?;
    for node in &asset.support_nodes {
        writer.u32(node.id.0);
        writer.u32(node.chunk_id.0);
        writer.u16(node.material_id);
        write_opt_u16(writer, node.orientation_summary);
        write_vec2(writer, node.anisotropy_axis, "support_node.anisotropy_axis")?;
        writer.u64(node.stable_seed);
        writer.vec_len(node.voxels.len())?;
        for voxel in &node.voxels {
            write_grid_coord(writer, *voxel);
        }
    }
    writer.vec_len(asset.chunks.len())?;
    for chunk in &asset.chunks {
        writer.u32(chunk.id.0);
        writer.vec_len(chunk.support_nodes.len())?;
        for node in &chunk.support_nodes {
            writer.u32(node.0);
        }
        write_opt_id(writer, chunk.parent.map(|id| id.0));
    }
    writer.vec_len(asset.internal_bonds.len())?;
    for bond in &asset.internal_bonds {
        writer.u32(bond.id.0);
        writer.u32(bond.node_a.0);
        writer.u32(bond.node_b.0);
        write_vec2(writer, bond.centroid, "bond.centroid")?;
        write_vec2(writer, bond.normal, "bond.normal")?;
        write_vec2(writer, bond.tangent, "bond.tangent")?;
        writer.f32(bond.length, "bond.length")?;
        writer.f32(bond.base_health, "bond.base_health")?;
        writer.f32(bond.tension_limit, "bond.tension_limit")?;
        writer.f32(bond.compression_limit, "bond.compression_limit")?;
        writer.f32(bond.shear_limit, "bond.shear_limit")?;
        writer.u16(bond.material_pair.0);
        writer.u16(bond.material_pair.1);
        writer.vec_len(bond.interface_edges.len())?;
        for edge in &bond.interface_edges {
            write_lattice_point(writer, edge.start);
            write_lattice_point(writer, edge.end);
        }
    }
    Ok(())
}

pub(crate) fn read_asset_payload(reader: &mut Reader<'_>) -> Result<FxAsset, FxCoreSnapshotError> {
    let id = FxAssetId(reader.u32("asset.id")?);
    let voxel_size = reader.f32("asset.voxel_size")?;
    if voxel_size <= 0.0 {
        return Err(FxCoreSnapshotError::InvalidValue("asset.voxel_size"));
    }
    let width = reader.u32("occupancy.width")?;
    let height = reader.u32("occupancy.height")?;
    let cells_len = reader.len("occupancy.cells")?;
    let expected = checked_cell_count(width, height)?;
    if cells_len != expected {
        return Err(FxCoreSnapshotError::StateMismatch(
            "occupancy cell count does not match dimensions",
        ));
    }
    let mut cells = Vec::with_capacity(cells_len);
    for _ in 0..cells_len {
        cells.push(match reader.u8("occupancy.cell")? {
            0 => false,
            1 => true,
            _ => return Err(FxCoreSnapshotError::InvalidValue("occupancy.cell")),
        });
    }
    let occupancy = DenseOccupancy::new(width, height, cells)?;

    let map_len = reader.len("voxel_to_node")?;
    if map_len != expected {
        return Err(FxCoreSnapshotError::StateMismatch(
            "voxel_to_node count does not match dimensions",
        ));
    }
    let mut voxel_to_node = Vec::with_capacity(map_len);
    for _ in 0..map_len {
        voxel_to_node.push(read_opt_id(reader, "voxel_to_node")?.map(SupportNodeId));
    }

    let support_node_count = reader.len("support_nodes")?;
    let mut support_nodes = Vec::with_capacity(support_node_count);
    for _ in 0..support_node_count {
        let id = SupportNodeId(reader.u32("support_node.id")?);
        let chunk_id = ChunkId(reader.u32("support_node.chunk_id")?);
        let material_id = reader.u16("support_node.material_id")?;
        let orientation_summary = read_opt_u16(reader, "support_node.orientation_summary")?;
        let anisotropy_axis = read_vec2(reader, "support_node.anisotropy_axis")?;
        let stable_seed = reader.u64("support_node.stable_seed")?;
        let voxel_count = reader.len("support_node.voxels")?;
        let mut voxels = Vec::with_capacity(voxel_count);
        for _ in 0..voxel_count {
            voxels.push(read_grid_coord(reader)?);
        }
        support_nodes.push(SupportNode2D {
            id,
            chunk_id,
            voxels,
            material_id,
            orientation_summary,
            anisotropy_axis,
            stable_seed,
        });
    }
    support_nodes.sort_by_key(|node| node.id);

    let chunk_count = reader.len("chunks")?;
    let mut chunks = Vec::with_capacity(chunk_count);
    for _ in 0..chunk_count {
        let id = ChunkId(reader.u32("chunk.id")?);
        let support_node_count = reader.len("chunk.support_nodes")?;
        let mut support_nodes = Vec::with_capacity(support_node_count);
        for _ in 0..support_node_count {
            support_nodes.push(SupportNodeId(reader.u32("chunk.support_node")?));
        }
        support_nodes.sort_unstable();
        chunks.push(Chunk2D {
            id,
            support_nodes,
            parent: read_opt_id(reader, "chunk.parent")?.map(ChunkId),
        });
    }
    normalize_chunks(&mut chunks);

    let internal_bond_count = reader.len("internal_bonds")?;
    let mut internal_bonds = Vec::with_capacity(internal_bond_count);
    for _ in 0..internal_bond_count {
        let id = BondId(reader.u32("bond.id")?);
        let node_a = SupportNodeId(reader.u32("bond.node_a")?);
        let node_b = SupportNodeId(reader.u32("bond.node_b")?);
        let centroid = read_vec2(reader, "bond.centroid")?;
        let normal = read_vec2(reader, "bond.normal")?;
        let tangent = read_vec2(reader, "bond.tangent")?;
        let length = reader.f32("bond.length")?;
        let base_health = reader.f32("bond.base_health")?;
        let tension_limit = reader.f32("bond.tension_limit")?;
        let compression_limit = reader.f32("bond.compression_limit")?;
        let shear_limit = reader.f32("bond.shear_limit")?;
        let material_pair = (
            reader.u16("bond.material_pair.0")?,
            reader.u16("bond.material_pair.1")?,
        );
        let interface_edge_count = reader.len("bond.interface_edges")?;
        let mut interface_edges = Vec::with_capacity(interface_edge_count);
        for _ in 0..interface_edge_count {
            interface_edges.push(InterfaceEdge {
                start: read_lattice_point(reader)?,
                end: read_lattice_point(reader)?,
            });
        }
        internal_bonds.push(Bond2D {
            id,
            node_a,
            node_b,
            centroid,
            normal,
            tangent,
            length,
            base_health,
            tension_limit,
            compression_limit,
            shear_limit,
            material_pair,
            interface_edges,
        });
    }
    internal_bonds.sort_by_key(|bond| bond.id);

    let asset = FxAsset {
        id,
        voxel_size,
        occupancy,
        support_nodes,
        chunks,
        internal_bonds,
        voxel_to_node,
    };
    asset.validate()?;
    validate_asset_finite(&asset)?;
    Ok(asset)
}

pub(crate) fn write_family_payload(
    writer: &mut Writer,
    family: &FxFamily,
) -> Result<(), FxCoreSnapshotError> {
    validate_family_snapshot(family)?;
    writer.u32(family.id.0);
    writer.u32(family.next_actor_id);
    writer.u32(family.next_event_id);
    write_asset_payload(writer, &family.asset)?;
    writer.vec_len(family.actors.len())?;
    for (id, actor) in &family.actors {
        writer.u32(id.0);
        write_actor(writer, actor)?;
    }
    writer.vec_len(family.node_owner.len())?;
    for (node, actor) in &family.node_owner {
        writer.u32(node.0);
        writer.u32(actor.0);
    }
    writer.vec_len(family.node_states.len())?;
    for (node, state) in &family.node_states {
        writer.u32(node.0);
        write_node_state(writer, state)?;
    }
    writer.vec_len(family.chunk_states.len())?;
    for (chunk, state) in &family.chunk_states {
        writer.u32(chunk.0);
        write_chunk_state(writer, state)?;
    }
    writer.vec_len(family.bond_states.len())?;
    for state in &family.bond_states {
        write_bond_state(writer, state)?;
    }
    writer.vec_len(family.external_bonds.len())?;
    for (id, bond) in &family.external_bonds {
        writer.u32(id.0);
        writer.u32(bond.id.0);
        writer.u32(bond.node.0);
        writer.u8(match bond.target.kind {
            ExternalTargetKind::World => 0,
            ExternalTargetKind::Static => 1,
            ExternalTargetKind::Kinematic => 2,
        });
        writer.u32(bond.target.token.0);
        write_vec2(writer, bond.anchor, "external_bond.anchor")?;
        write_vec2(writer, bond.normal, "external_bond.normal")?;
        write_vec2(writer, bond.tangent, "external_bond.tangent")?;
        writer.f32(bond.base_health, "external_bond.base_health")?;
        writer.f32(bond.tension_limit, "external_bond.tension_limit")?;
        writer.f32(bond.compression_limit, "external_bond.compression_limit")?;
        writer.f32(bond.shear_limit, "external_bond.shear_limit")?;
        write_bond_state(writer, &bond.runtime)?;
    }
    writer.vec_len(family.dynamic_structural_bonds.len())?;
    for (id, bond) in &family.dynamic_structural_bonds {
        if bond.policy == DynamicConnectionPolicy::CustomHardConstraint {
            return Err(FxCoreSnapshotError::UnsupportedCustomHardConstraint(*id));
        }
        writer.u32(id.0);
        writer.u32(bond.id.0);
        writer.u32(bond.node_a.0);
        writer.u32(bond.node_b.0);
        writer.u8(0);
        write_vec2(writer, bond.centroid, "dynamic_bond.centroid")?;
        write_vec2(writer, bond.normal, "dynamic_bond.normal")?;
        write_vec2(writer, bond.tangent, "dynamic_bond.tangent")?;
        writer.f32(bond.base_health, "dynamic_bond.base_health")?;
        writer.f32(bond.tension_limit, "dynamic_bond.tension_limit")?;
        writer.f32(bond.compression_limit, "dynamic_bond.compression_limit")?;
        writer.f32(bond.shear_limit, "dynamic_bond.shear_limit")?;
        write_bond_state(writer, &bond.runtime)?;
    }
    writer.vec_len(family.dirty_actors.len())?;
    for actor in &family.dirty_actors {
        writer.u32(actor.0);
    }
    Ok(())
}

pub(crate) fn read_family_payload(
    reader: &mut Reader<'_>,
) -> Result<FxFamily, FxCoreSnapshotError> {
    let id = FxFamilyId(reader.u32("family.id")?);
    let next_actor_id = reader.u32("family.next_actor_id")?;
    let next_event_id = reader.u32("family.next_event_id")?;
    let asset = read_asset_payload(reader)?;

    let mut actors = BTreeMap::new();
    for _ in 0..reader.len("family.actors")? {
        let key = FxActorId(reader.u32("actor.key")?);
        let actor = read_actor(reader)?;
        if key != actor.id || actors.insert(key, actor).is_some() {
            return Err(FxCoreSnapshotError::StateMismatch(
                "actor key/id mismatch or duplicate",
            ));
        }
    }

    let mut node_owner = BTreeMap::new();
    for _ in 0..reader.len("family.node_owner")? {
        let node = SupportNodeId(reader.u32("node_owner.node")?);
        let actor = FxActorId(reader.u32("node_owner.actor")?);
        if node_owner.insert(node, actor).is_some() {
            return Err(FxCoreSnapshotError::StateMismatch("duplicate node owner"));
        }
    }

    let mut node_states = BTreeMap::new();
    for _ in 0..reader.len("family.node_states")? {
        let node = SupportNodeId(reader.u32("node_state.node")?);
        let state = read_node_state(reader)?;
        if node_states.insert(node, state).is_some() {
            return Err(FxCoreSnapshotError::StateMismatch("duplicate node state"));
        }
    }

    let mut chunk_states = BTreeMap::new();
    for _ in 0..reader.len("family.chunk_states")? {
        let chunk = ChunkId(reader.u32("chunk_state.chunk")?);
        let state = read_chunk_state(reader)?;
        if chunk_states.insert(chunk, state).is_some() {
            return Err(FxCoreSnapshotError::StateMismatch("duplicate chunk state"));
        }
    }

    let bond_state_count = reader.len("family.bond_states")?;
    let mut bond_states = Vec::with_capacity(bond_state_count);
    for _ in 0..bond_state_count {
        bond_states.push(read_bond_state(reader)?);
    }

    let mut external_bonds = BTreeMap::new();
    for _ in 0..reader.len("family.external_bonds")? {
        let key = ExternalBondId(reader.u32("external_bond.key")?);
        let id = ExternalBondId(reader.u32("external_bond.id")?);
        let node = SupportNodeId(reader.u32("external_bond.node")?);
        let kind = match reader.u8("external_bond.kind")? {
            0 => ExternalTargetKind::World,
            1 => ExternalTargetKind::Static,
            2 => ExternalTargetKind::Kinematic,
            _ => return Err(FxCoreSnapshotError::InvalidValue("external_bond.kind")),
        };
        let token = ExternalTargetToken(reader.u32("external_bond.token")?);
        let bond = ExternalBond2D {
            id,
            node,
            target: ExternalTarget2D { kind, token },
            anchor: read_vec2(reader, "external_bond.anchor")?,
            normal: read_vec2(reader, "external_bond.normal")?,
            tangent: read_vec2(reader, "external_bond.tangent")?,
            base_health: reader.f32("external_bond.base_health")?,
            tension_limit: reader.f32("external_bond.tension_limit")?,
            compression_limit: reader.f32("external_bond.compression_limit")?,
            shear_limit: reader.f32("external_bond.shear_limit")?,
            runtime: read_bond_state(reader)?,
        };
        if key != id || external_bonds.insert(key, bond).is_some() {
            return Err(FxCoreSnapshotError::StateMismatch(
                "external bond key/id mismatch or duplicate",
            ));
        }
    }

    let mut dynamic_structural_bonds = BTreeMap::new();
    for _ in 0..reader.len("family.dynamic_structural_bonds")? {
        let key = ConnectionId(reader.u32("dynamic_bond.key")?);
        let id = ConnectionId(reader.u32("dynamic_bond.id")?);
        let node_a = SupportNodeId(reader.u32("dynamic_bond.node_a")?);
        let node_b = SupportNodeId(reader.u32("dynamic_bond.node_b")?);
        let policy = match reader.u8("dynamic_bond.policy")? {
            0 => DynamicConnectionPolicy::GraphOnly,
            1 => return Err(FxCoreSnapshotError::UnsupportedCustomHardConstraint(id)),
            _ => return Err(FxCoreSnapshotError::InvalidValue("dynamic_bond.policy")),
        };
        let bond = DynamicStructuralBond2D {
            id,
            node_a,
            node_b,
            policy,
            centroid: read_vec2(reader, "dynamic_bond.centroid")?,
            normal: read_vec2(reader, "dynamic_bond.normal")?,
            tangent: read_vec2(reader, "dynamic_bond.tangent")?,
            base_health: reader.f32("dynamic_bond.base_health")?,
            tension_limit: reader.f32("dynamic_bond.tension_limit")?,
            compression_limit: reader.f32("dynamic_bond.compression_limit")?,
            shear_limit: reader.f32("dynamic_bond.shear_limit")?,
            runtime: read_bond_state(reader)?,
        };
        if key != id || dynamic_structural_bonds.insert(key, bond).is_some() {
            return Err(FxCoreSnapshotError::StateMismatch(
                "dynamic bond key/id mismatch or duplicate",
            ));
        }
    }

    let mut dirty_actors = BTreeSet::new();
    for _ in 0..reader.len("family.dirty_actors")? {
        if !dirty_actors.insert(FxActorId(reader.u32("family.dirty_actor")?)) {
            return Err(FxCoreSnapshotError::StateMismatch("duplicate dirty actor"));
        }
    }

    let family = FxFamily {
        id,
        asset,
        actors,
        node_owner,
        node_states,
        chunk_states,
        bond_states,
        external_bonds,
        dynamic_structural_bonds,
        dirty_actors,
        next_actor_id,
        next_event_id,
    };
    validate_family_snapshot(&family)?;
    Ok(family)
}

fn validate_family_snapshot(family: &FxFamily) -> Result<(), FxCoreSnapshotError> {
    family.asset.validate()?;
    validate_asset_finite(&family.asset)?;
    if let Some(max_actor) = family.actors.keys().map(|actor| actor.0).max() {
        if family.next_actor_id <= max_actor {
            return Err(FxCoreSnapshotError::StateMismatch(
                "next_actor_id would overwrite an existing actor",
            ));
        }
    }
    if family.bond_states.len() != family.asset.internal_bonds.len() {
        return Err(FxCoreSnapshotError::StateMismatch(
            "bond state count mismatch",
        ));
    }
    let node_ids = family
        .asset
        .support_nodes
        .iter()
        .map(|node| node.id)
        .collect::<BTreeSet<_>>();
    if family.node_states.keys().copied().collect::<BTreeSet<_>>() != node_ids {
        return Err(FxCoreSnapshotError::StateMismatch(
            "node state keys mismatch",
        ));
    }
    if family.node_owner.keys().copied().collect::<BTreeSet<_>>() != node_ids {
        return Err(FxCoreSnapshotError::StateMismatch(
            "node owner keys mismatch",
        ));
    }
    let chunk_ids = family
        .asset
        .chunks
        .iter()
        .map(|chunk| chunk.id)
        .collect::<BTreeSet<_>>();
    if family.chunk_states.keys().copied().collect::<BTreeSet<_>>() != chunk_ids {
        return Err(FxCoreSnapshotError::StateMismatch(
            "chunk state keys mismatch",
        ));
    }
    let mut built_node_owner = BTreeMap::new();
    for (actor_id, actor) in &family.actors {
        if *actor_id != actor.id || actor.owned_nodes.is_empty() {
            return Err(FxCoreSnapshotError::StateMismatch("invalid actor"));
        }
        validate_actor_finite(actor)?;
        for node in &actor.owned_nodes {
            if !node_ids.contains(node) {
                return Err(FxCoreSnapshotError::StateMismatch(
                    "actor owns unknown node",
                ));
            }
            if built_node_owner.insert(*node, *actor_id).is_some() {
                return Err(FxCoreSnapshotError::StateMismatch(
                    "duplicate actor-owned node",
                ));
            }
            if family.node_owner.get(node) != Some(actor_id) {
                return Err(FxCoreSnapshotError::StateMismatch(
                    "actor ownership does not match node owner map",
                ));
            }
        }
    }
    if built_node_owner != family.node_owner {
        return Err(FxCoreSnapshotError::StateMismatch(
            "built actor ownership map does not match node owner map",
        ));
    }
    for actor in family.node_owner.values() {
        if !family.actors.contains_key(actor) {
            return Err(FxCoreSnapshotError::StateMismatch(
                "node owner references missing actor",
            ));
        }
    }
    for actor in &family.dirty_actors {
        if !family.actors.contains_key(actor) {
            return Err(FxCoreSnapshotError::StateMismatch(
                "dirty actor references missing actor",
            ));
        }
    }
    for (node, state) in &family.node_states {
        if !valid_runtime_scalar(state.health) || !valid_runtime_scalar(state.accumulated_damage) {
            return Err(FxCoreSnapshotError::InvalidValue("node_state"));
        }
        if !node_ids.contains(node) {
            return Err(FxCoreSnapshotError::StateMismatch(
                "node state references missing node",
            ));
        }
    }
    for (chunk, state) in &family.chunk_states {
        if !valid_runtime_scalar(state.health) || !valid_runtime_scalar(state.accumulated_damage) {
            return Err(FxCoreSnapshotError::InvalidValue("chunk_state"));
        }
        if !chunk_ids.contains(chunk) {
            return Err(FxCoreSnapshotError::StateMismatch(
                "chunk state references missing chunk",
            ));
        }
    }
    for state in &family.bond_states {
        validate_bond_state(state)?;
    }
    for bond in family.external_bonds.values() {
        if !node_ids.contains(&bond.node) || !family.node_owner.contains_key(&bond.node) {
            return Err(FxCoreSnapshotError::StateMismatch(
                "external bond references missing node",
            ));
        }
        validate_external_bond(bond)?;
    }
    for (id, bond) in &family.dynamic_structural_bonds {
        if bond.policy == DynamicConnectionPolicy::CustomHardConstraint {
            return Err(FxCoreSnapshotError::UnsupportedCustomHardConstraint(*id));
        }
        if !node_ids.contains(&bond.node_a) || !node_ids.contains(&bond.node_b) {
            return Err(FxCoreSnapshotError::StateMismatch(
                "dynamic bond references missing node",
            ));
        }
        validate_dynamic_bond(bond)?;
    }
    Ok(())
}

fn write_actor(writer: &mut Writer, actor: &FxActor) -> Result<(), FxCoreSnapshotError> {
    validate_actor_finite(actor)?;
    writer.u32(actor.id.0);
    writer.vec_len(actor.owned_nodes.len())?;
    for node in &actor.owned_nodes {
        writer.u32(node.0);
    }
    writer.f32(actor.mass, "actor.mass")?;
    write_vec2(writer, actor.local_com, "actor.local_com")?;
    writer.f32(actor.inertia, "actor.inertia")?;
    match actor.bounds {
        Some(bounds) => {
            writer.u8(1);
            write_grid_coord(writer, bounds.min);
            write_grid_coord(writer, bounds.max);
        }
        None => writer.u8(0),
    }
    Ok(())
}

fn read_actor(reader: &mut Reader<'_>) -> Result<FxActor, FxCoreSnapshotError> {
    let id = FxActorId(reader.u32("actor.id")?);
    let owned_node_count = reader.len("actor.owned_nodes")?;
    let mut owned_nodes = Vec::with_capacity(owned_node_count);
    for _ in 0..owned_node_count {
        owned_nodes.push(SupportNodeId(reader.u32("actor.owned_node")?));
    }
    let mass = reader.f32("actor.mass")?;
    let local_com = read_vec2(reader, "actor.local_com")?;
    let inertia = reader.f32("actor.inertia")?;
    let bounds = match reader.u8("actor.bounds.tag")? {
        0 => None,
        1 => Some(GridAabb {
            min: read_grid_coord(reader)?,
            max: read_grid_coord(reader)?,
        }),
        _ => return Err(FxCoreSnapshotError::InvalidValue("actor.bounds.tag")),
    };
    Ok(FxActor {
        id,
        owned_nodes,
        mass,
        local_com,
        inertia,
        bounds,
    })
}

fn write_node_state(
    writer: &mut Writer,
    state: &NodeRuntimeState,
) -> Result<(), FxCoreSnapshotError> {
    writer.f32(state.health, "node_state.health")?;
    writer.f32(state.accumulated_damage, "node_state.accumulated_damage")?;
    Ok(())
}

fn read_node_state(reader: &mut Reader<'_>) -> Result<NodeRuntimeState, FxCoreSnapshotError> {
    Ok(NodeRuntimeState {
        health: reader.f32("node_state.health")?,
        accumulated_damage: reader.f32("node_state.accumulated_damage")?,
    })
}

fn write_chunk_state(
    writer: &mut Writer,
    state: &ChunkRuntimeState,
) -> Result<(), FxCoreSnapshotError> {
    writer.f32(state.health, "chunk_state.health")?;
    writer.f32(state.accumulated_damage, "chunk_state.accumulated_damage")?;
    Ok(())
}

fn read_chunk_state(reader: &mut Reader<'_>) -> Result<ChunkRuntimeState, FxCoreSnapshotError> {
    Ok(ChunkRuntimeState {
        health: reader.f32("chunk_state.health")?,
        accumulated_damage: reader.f32("chunk_state.accumulated_damage")?,
    })
}

fn write_bond_state(
    writer: &mut Writer,
    state: &BondRuntimeState,
) -> Result<(), FxCoreSnapshotError> {
    validate_bond_state(state)?;
    writer.f32(state.health, "bond_state.health")?;
    writer.f32(state.effective_length, "bond_state.effective_length")?;
    writer.f32(state.accumulated_damage, "bond_state.accumulated_damage")?;
    Ok(())
}

fn read_bond_state(reader: &mut Reader<'_>) -> Result<BondRuntimeState, FxCoreSnapshotError> {
    let state = BondRuntimeState {
        health: reader.f32("bond_state.health")?,
        effective_length: reader.f32("bond_state.effective_length")?,
        accumulated_damage: reader.f32("bond_state.accumulated_damage")?,
    };
    validate_bond_state(&state)?;
    Ok(state)
}

fn validate_asset_finite(asset: &FxAsset) -> Result<(), FxCoreSnapshotError> {
    if !asset.voxel_size.is_finite() || asset.voxel_size <= 0.0 {
        return Err(FxCoreSnapshotError::InvalidValue("asset.voxel_size"));
    }
    for node in &asset.support_nodes {
        if !valid_vec2(node.anisotropy_axis) {
            return Err(FxCoreSnapshotError::InvalidValue(
                "support_node.anisotropy_axis",
            ));
        }
    }
    for bond in &asset.internal_bonds {
        if !valid_vec2(bond.centroid)
            || !valid_vec2(bond.normal)
            || !valid_vec2(bond.tangent)
            || !valid_runtime_scalar(bond.length)
            || !valid_runtime_scalar(bond.base_health)
            || !valid_runtime_scalar(bond.tension_limit)
            || !valid_runtime_scalar(bond.compression_limit)
            || !valid_runtime_scalar(bond.shear_limit)
        {
            return Err(FxCoreSnapshotError::InvalidValue("bond"));
        }
    }
    Ok(())
}

fn validate_actor_finite(actor: &FxActor) -> Result<(), FxCoreSnapshotError> {
    if !valid_runtime_scalar(actor.mass)
        || !valid_vec2(actor.local_com)
        || !valid_runtime_scalar(actor.inertia)
    {
        return Err(FxCoreSnapshotError::InvalidValue("actor"));
    }
    Ok(())
}

fn validate_external_bond(bond: &ExternalBond2D) -> Result<(), FxCoreSnapshotError> {
    if !valid_vec2(bond.anchor)
        || !valid_vec2(bond.normal)
        || !valid_vec2(bond.tangent)
        || !valid_runtime_scalar(bond.base_health)
        || !valid_runtime_scalar(bond.tension_limit)
        || !valid_runtime_scalar(bond.compression_limit)
        || !valid_runtime_scalar(bond.shear_limit)
    {
        return Err(FxCoreSnapshotError::InvalidValue("external_bond"));
    }
    validate_direction_pair(bond.normal, bond.tangent, "external_bond.direction")?;
    validate_bond_state(&bond.runtime)
}

fn validate_dynamic_bond(bond: &DynamicStructuralBond2D) -> Result<(), FxCoreSnapshotError> {
    if bond.node_a >= bond.node_b {
        return Err(FxCoreSnapshotError::StateMismatch(
            "dynamic bond endpoints must be canonical and non-self",
        ));
    }
    if !valid_vec2(bond.centroid)
        || !valid_vec2(bond.normal)
        || !valid_vec2(bond.tangent)
        || !valid_runtime_scalar(bond.base_health)
        || !valid_runtime_scalar(bond.tension_limit)
        || !valid_runtime_scalar(bond.compression_limit)
        || !valid_runtime_scalar(bond.shear_limit)
    {
        return Err(FxCoreSnapshotError::InvalidValue("dynamic_bond"));
    }
    validate_direction_pair(bond.normal, bond.tangent, "dynamic_bond.direction")?;
    validate_bond_state(&bond.runtime)
}

fn validate_direction_pair(
    normal: Vec2,
    tangent: Vec2,
    field: &'static str,
) -> Result<(), FxCoreSnapshotError> {
    let normal_len = normal.length();
    let tangent_len = tangent.length();
    if (normal_len - 1.0).abs() > 0.0001 || (tangent_len - 1.0).abs() > 0.0001 {
        return Err(FxCoreSnapshotError::InvalidValue(field));
    }
    let expected = normal.perp();
    if (tangent.x - expected.x).abs() > 0.0001 || (tangent.y - expected.y).abs() > 0.0001 {
        return Err(FxCoreSnapshotError::InvalidValue(field));
    }
    Ok(())
}

fn validate_bond_state(state: &BondRuntimeState) -> Result<(), FxCoreSnapshotError> {
    if !valid_runtime_scalar(state.health)
        || !valid_runtime_scalar(state.effective_length)
        || !valid_runtime_scalar(state.accumulated_damage)
    {
        return Err(FxCoreSnapshotError::InvalidValue("bond_state"));
    }
    Ok(())
}

pub(crate) fn wrap_payload(
    magic: [u8; 8],
    mode: SnapshotMode,
    flags: u32,
    payload: Vec<u8>,
) -> Vec<u8> {
    let mut out = Vec::with_capacity(HEADER_LEN + payload.len());
    out.extend_from_slice(&magic);
    out.extend_from_slice(&VERSION.to_le_bytes());
    out.push(DIMENSION_2D);
    out.push(SCALAR_F32);
    out.push(mode as u8);
    out.push(0);
    out.extend_from_slice(&flags.to_le_bytes());
    out.extend_from_slice(&(payload.len() as u64).to_le_bytes());
    out.extend_from_slice(&checksum(&payload).to_le_bytes());
    out.extend_from_slice(&payload);
    out
}

pub(crate) fn unwrap_payload<'a>(
    bytes: &'a [u8],
    magic: [u8; 8],
) -> Result<(SnapshotMode, &'a [u8]), FxCoreSnapshotError> {
    if bytes.len() < HEADER_LEN {
        return Err(FxCoreSnapshotError::UnexpectedEof("header"));
    }
    if bytes[0..8] != magic {
        return Err(FxCoreSnapshotError::InvalidMagic);
    }
    let version = u16::from_le_bytes([bytes[8], bytes[9]]);
    if version != VERSION {
        return Err(FxCoreSnapshotError::UnsupportedVersion(version));
    }
    if bytes[10] != DIMENSION_2D {
        return Err(FxCoreSnapshotError::UnsupportedDimension(bytes[10]));
    }
    if bytes[11] != SCALAR_F32 {
        return Err(FxCoreSnapshotError::UnsupportedScalar(bytes[11]));
    }
    let mode = SnapshotMode::from_u8(bytes[12])?;
    if bytes[13] != 0 {
        return Err(FxCoreSnapshotError::UnsupportedFlags(bytes[13] as u32));
    }
    let flags = u32::from_le_bytes([bytes[14], bytes[15], bytes[16], bytes[17]]);
    if flags != 0 {
        return Err(FxCoreSnapshotError::UnsupportedFlags(flags));
    }
    let payload_len = u64::from_le_bytes([
        bytes[18], bytes[19], bytes[20], bytes[21], bytes[22], bytes[23], bytes[24], bytes[25],
    ]) as usize;
    let expected = HEADER_LEN
        .checked_add(payload_len)
        .ok_or(FxCoreSnapshotError::PayloadLengthMismatch)?;
    if bytes.len() != expected {
        return Err(FxCoreSnapshotError::PayloadLengthMismatch);
    }
    let expected_checksum = u64::from_le_bytes([
        bytes[26], bytes[27], bytes[28], bytes[29], bytes[30], bytes[31], bytes[32], bytes[33],
    ]);
    let payload = &bytes[HEADER_LEN..];
    if checksum(payload) != expected_checksum {
        return Err(FxCoreSnapshotError::PayloadChecksumMismatch);
    }
    Ok((mode, payload))
}

fn checked_cell_count(width: u32, height: u32) -> Result<usize, FxCoreSnapshotError> {
    (width as usize)
        .checked_mul(height as usize)
        .ok_or(FxCoreSnapshotError::StateMismatch(
            "grid dimensions overflow",
        ))
}

fn write_vec2(
    writer: &mut Writer,
    value: Vec2,
    field: &'static str,
) -> Result<(), FxCoreSnapshotError> {
    if !valid_vec2(value) {
        return Err(FxCoreSnapshotError::InvalidValue(field));
    }
    writer.f32(value.x, field)?;
    writer.f32(value.y, field)?;
    Ok(())
}

fn read_vec2(reader: &mut Reader<'_>, field: &'static str) -> Result<Vec2, FxCoreSnapshotError> {
    Ok(Vec2::new(reader.f32(field)?, reader.f32(field)?))
}

fn write_grid_coord(writer: &mut Writer, value: GridCoord) {
    writer.u32(value.x);
    writer.u32(value.y);
}

fn read_grid_coord(reader: &mut Reader<'_>) -> Result<GridCoord, FxCoreSnapshotError> {
    Ok(GridCoord::new(
        reader.u32("grid_coord.x")?,
        reader.u32("grid_coord.y")?,
    ))
}

fn write_lattice_point(writer: &mut Writer, value: LatticePoint) {
    writer.u32(value.x);
    writer.u32(value.y);
}

fn read_lattice_point(reader: &mut Reader<'_>) -> Result<LatticePoint, FxCoreSnapshotError> {
    Ok(LatticePoint::new(
        reader.u32("lattice_point.x")?,
        reader.u32("lattice_point.y")?,
    ))
}

fn write_opt_id(writer: &mut Writer, value: Option<u32>) {
    match value {
        Some(value) => {
            writer.u8(1);
            writer.u32(value);
        }
        None => writer.u8(0),
    }
}

fn read_opt_id(
    reader: &mut Reader<'_>,
    field: &'static str,
) -> Result<Option<u32>, FxCoreSnapshotError> {
    match reader.u8(field)? {
        0 => Ok(None),
        1 => Ok(Some(reader.u32(field)?)),
        _ => Err(FxCoreSnapshotError::InvalidValue(field)),
    }
}

fn write_opt_u16(writer: &mut Writer, value: Option<u16>) {
    match value {
        Some(value) => {
            writer.u8(1);
            writer.u16(value);
        }
        None => writer.u8(0),
    }
}

fn read_opt_u16(
    reader: &mut Reader<'_>,
    field: &'static str,
) -> Result<Option<u16>, FxCoreSnapshotError> {
    match reader.u8(field)? {
        0 => Ok(None),
        1 => Ok(Some(reader.u16(field)?)),
        _ => Err(FxCoreSnapshotError::InvalidValue(field)),
    }
}

pub(crate) struct Writer {
    bytes: Vec<u8>,
}

impl Writer {
    pub(crate) fn new() -> Self {
        Self { bytes: Vec::new() }
    }

    pub(crate) fn into_inner(self) -> Vec<u8> {
        self.bytes
    }

    pub(crate) fn u8(&mut self, value: u8) {
        self.bytes.push(value);
    }

    pub(crate) fn u16(&mut self, value: u16) {
        self.bytes.extend_from_slice(&value.to_le_bytes());
    }

    pub(crate) fn u32(&mut self, value: u32) {
        self.bytes.extend_from_slice(&value.to_le_bytes());
    }

    pub(crate) fn u64(&mut self, value: u64) {
        self.bytes.extend_from_slice(&value.to_le_bytes());
    }

    pub(crate) fn f32(
        &mut self,
        value: f32,
        field: &'static str,
    ) -> Result<(), FxCoreSnapshotError> {
        if !value.is_finite() {
            return Err(FxCoreSnapshotError::InvalidValue(field));
        }
        self.u32(value.to_bits());
        Ok(())
    }

    pub(crate) fn vec_len(&mut self, len: usize) -> Result<(), FxCoreSnapshotError> {
        let len = u32::try_from(len).map_err(|_| FxCoreSnapshotError::InvalidValue("length"))?;
        self.u32(len);
        Ok(())
    }
}

pub(crate) struct Reader<'a> {
    bytes: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    pub(crate) fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, pos: 0 }
    }

    pub(crate) fn finish(&self) -> Result<(), FxCoreSnapshotError> {
        if self.pos == self.bytes.len() {
            Ok(())
        } else {
            Err(FxCoreSnapshotError::TrailingBytes)
        }
    }

    pub(crate) fn u8(&mut self, field: &'static str) -> Result<u8, FxCoreSnapshotError> {
        let bytes = self.take(1, field)?;
        Ok(bytes[0])
    }

    pub(crate) fn u16(&mut self, field: &'static str) -> Result<u16, FxCoreSnapshotError> {
        let bytes = self.take(2, field)?;
        Ok(u16::from_le_bytes([bytes[0], bytes[1]]))
    }

    pub(crate) fn u32(&mut self, field: &'static str) -> Result<u32, FxCoreSnapshotError> {
        let bytes = self.take(4, field)?;
        Ok(u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
    }

    pub(crate) fn u64(&mut self, field: &'static str) -> Result<u64, FxCoreSnapshotError> {
        let bytes = self.take(8, field)?;
        Ok(u64::from_le_bytes([
            bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7],
        ]))
    }

    pub(crate) fn f32(&mut self, field: &'static str) -> Result<f32, FxCoreSnapshotError> {
        let value = f32::from_bits(self.u32(field)?);
        if value.is_finite() {
            Ok(value)
        } else {
            Err(FxCoreSnapshotError::InvalidValue(field))
        }
    }

    pub(crate) fn len(&mut self, field: &'static str) -> Result<usize, FxCoreSnapshotError> {
        Ok(self.u32(field)? as usize)
    }

    fn take(&mut self, len: usize, field: &'static str) -> Result<&'a [u8], FxCoreSnapshotError> {
        let end = self
            .pos
            .checked_add(len)
            .ok_or(FxCoreSnapshotError::UnexpectedEof(field))?;
        if end > self.bytes.len() {
            return Err(FxCoreSnapshotError::UnexpectedEof(field));
        }
        let out = &self.bytes[self.pos..end];
        self.pos = end;
        Ok(out)
    }
}

fn checksum(bytes: &[u8]) -> u64 {
    let mut hash = 0xcbf2_9ce4_8422_2325u64;
    for byte in bytes {
        hash ^= u64::from(*byte);
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    }
    hash
}

#[cfg(test)]
mod tests {
    use super::*;

    fn asset_from_rows(rows: &[&str], map: &[Option<u32>]) -> FxAsset {
        asset_from_rows_with_limits(rows, map, 10.0, 10.0, 10.0)
    }

    fn asset_from_rows_with_limits(
        rows: &[&str],
        map: &[Option<u32>],
        tension_limit: f32,
        compression_limit: f32,
        shear_limit: f32,
    ) -> FxAsset {
        let occupancy = DenseOccupancy::from_rows(rows).unwrap();
        let mut desc = FxAssetDesc::new(
            FxAssetId(7),
            1.0,
            occupancy,
            map.iter().map(|id| id.map(SupportNodeId)).collect(),
        );
        desc.default_tension_limit = tension_limit;
        desc.default_compression_limit = compression_limit;
        desc.default_shear_limit = shear_limit;
        FxAsset::from_desc(desc).unwrap()
    }

    fn non_leaf_support_asset() -> FxAsset {
        let occupancy = DenseOccupancy::from_rows(&["##"]).unwrap();
        let mut desc = FxAssetDesc::new(
            FxAssetId(7),
            1.0,
            occupancy,
            vec![Some(SupportNodeId(0)), Some(SupportNodeId(1))],
        );
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
        FxAsset::from_desc(desc).unwrap()
    }

    fn family_snapshot_payload(family: &FxFamily) -> Vec<u8> {
        let bytes = family
            .to_snapshot_bytes(SnapshotMode::Deterministic)
            .unwrap();
        unwrap_payload(&bytes, MAGIC_FAMILY).unwrap().1.to_vec()
    }

    fn asset_snapshot_payload(asset: &FxAsset) -> Vec<u8> {
        let mut payload = Writer::new();
        write_asset_payload(&mut payload, asset).unwrap();
        payload.into_inner()
    }

    fn wrapped_asset_payload(payload: Vec<u8>) -> Vec<u8> {
        wrap_payload(MAGIC_ASSET, SnapshotMode::Deterministic, 0, payload)
    }

    fn wrapped_family_payload(payload: Vec<u8>) -> Vec<u8> {
        wrap_payload(MAGIC_FAMILY, SnapshotMode::Deterministic, 0, payload)
    }

    fn patch_first(bytes: &mut [u8], needle: &[u8], replacement: &[u8]) {
        assert_eq!(needle.len(), replacement.len());
        let offset = bytes
            .windows(needle.len())
            .position(|window| window == needle)
            .expect("needle not found");
        bytes[offset..offset + replacement.len()].copy_from_slice(replacement);
    }

    fn patch_last(bytes: &mut [u8], needle: &[u8], replacement: &[u8]) {
        assert_eq!(needle.len(), replacement.len());
        let offset = bytes
            .windows(needle.len())
            .rposition(|window| window == needle)
            .expect("needle not found");
        bytes[offset..offset + replacement.len()].copy_from_slice(replacement);
    }

    fn dynamic_bond_offset(bytes: &[u8], id: ConnectionId) -> usize {
        let needle = [id.0.to_le_bytes(), id.0.to_le_bytes()].concat();
        bytes
            .windows(needle.len())
            .position(|window| window == needle)
            .expect("dynamic bond id pair not found")
    }

    #[test]
    fn core_snapshot_roundtrip_preserves_digest_and_allocator_state() {
        let asset =
            asset_from_rows_with_limits(&["###"], &[Some(0), Some(1), Some(2)], 11.0, 17.0, 23.0);
        let mut family = FxFamily::instantiate(FxFamilyId(3), asset);
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
                target: FractureTarget::Bond(BondId(0)),
                health_loss: 1.0,
                effective_length_loss: 1.0,
                source: DamageSource::Script,
            }],
        );
        let split_events = split_dirty_actors(&mut family);
        assert_eq!(split_events.len(), 1);
        let external_id = family
            .connect_static_anchor(StaticAnchorDesc {
                id: ExternalBondId(5),
                node: SupportNodeId(2),
                target: ExternalTarget2D {
                    kind: ExternalTargetKind::World,
                    token: ExternalTargetToken(0),
                },
                anchor: Vec2::new(2.5, 0.5),
                normal: Vec2::new(1.0, 0.0),
                health: 1.0,
                effective_length: 1.0,
                tension_limit: 31.0,
                compression_limit: 37.0,
                shear_limit: 41.0,
            })
            .unwrap();
        let dynamic_id = family
            .connect_dynamic_structural_bond_graph_only(DynamicStructuralBondDesc {
                id: ConnectionId(9),
                node_a: SupportNodeId(0),
                node_b: SupportNodeId(2),
                centroid: Vec2::new(1.5, 0.5),
                normal: Vec2::new(1.0, 0.0),
                health: 1.0,
                effective_length: 1.0,
                tension_limit: 43.0,
                compression_limit: 47.0,
                shear_limit: 53.0,
            })
            .unwrap();
        let digest = family.deterministic_state_digest();
        let mut digest_probe = family.clone();
        digest_probe.asset.internal_bonds[0].compression_limit = 19.0;
        assert_ne!(digest_probe.deterministic_state_digest(), digest);
        let mut digest_probe = family.clone();
        digest_probe
            .external_bonds
            .get_mut(&external_id)
            .unwrap()
            .compression_limit = 39.0;
        assert_ne!(digest_probe.deterministic_state_digest(), digest);
        let mut digest_probe = family.clone();
        digest_probe
            .dynamic_structural_bonds
            .get_mut(&dynamic_id)
            .unwrap()
            .compression_limit = 49.0;
        assert_ne!(digest_probe.deterministic_state_digest(), digest);
        let bytes = family
            .to_snapshot_bytes(SnapshotMode::Deterministic)
            .unwrap();
        let mut restored = FxFamily::from_snapshot_bytes(&bytes).unwrap();
        assert_eq!(restored.asset.internal_bonds[0].tension_limit, 11.0);
        assert_eq!(restored.asset.internal_bonds[0].compression_limit, 17.0);
        assert_eq!(restored.asset.internal_bonds[0].shear_limit, 23.0);
        assert_eq!(
            restored
                .external_bonds
                .get(&external_id)
                .unwrap()
                .tension_limit,
            31.0
        );
        assert_eq!(
            restored
                .external_bonds
                .get(&external_id)
                .unwrap()
                .compression_limit,
            37.0
        );
        assert_eq!(
            restored
                .external_bonds
                .get(&external_id)
                .unwrap()
                .shear_limit,
            41.0
        );
        assert_eq!(
            restored
                .dynamic_structural_bonds
                .get(&dynamic_id)
                .unwrap()
                .tension_limit,
            43.0
        );
        assert_eq!(
            restored
                .dynamic_structural_bonds
                .get(&dynamic_id)
                .unwrap()
                .compression_limit,
            47.0
        );
        assert_eq!(
            restored
                .dynamic_structural_bonds
                .get(&dynamic_id)
                .unwrap()
                .shear_limit,
            53.0
        );
        assert_eq!(restored.deterministic_state_digest(), digest);

        let events = apply_fracture_commands(
            &mut restored,
            &[FractureCommand {
                order_key: DeterministicOrderKey::new(
                    2,
                    1,
                    FxFamilyId(3),
                    FxActorId(0),
                    CommandId(1),
                ),
                actor: FxActorId(0),
                target: FractureTarget::Bond(BondId(1)),
                health_loss: 1.0,
                effective_length_loss: 1.0,
                source: DamageSource::Script,
            }],
        );
        assert_eq!(events[0].event_id, EventId(2));
    }

    #[test]
    fn snapshot_restore() {
        let asset = asset_from_rows(&["###"], &[Some(0), Some(1), Some(2)]);
        let mut family = FxFamily::instantiate(FxFamilyId(3), asset);
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
                target: FractureTarget::Bond(BondId(0)),
                health_loss: 1.0,
                effective_length_loss: 1.0,
                source: DamageSource::Script,
            }],
        );
        split_dirty_actors(&mut family);
        let bytes = encode_family_snapshot(&family, SnapshotMode::Deterministic).unwrap();
        assert_eq!(&bytes[0..4], b"RFXS");
        let restored = restore_family_snapshot(&bytes).unwrap();
        assert_eq!(restored, family);
        assert_eq!(
            restored.deterministic_state_digest(),
            family.deterministic_state_digest()
        );
    }

    #[test]
    fn core_snapshot_roundtrip_preserves_non_leaf_support_chunks() {
        let asset = non_leaf_support_asset();
        let bytes = asset
            .to_snapshot_bytes(SnapshotMode::Deterministic)
            .unwrap();
        let restored_asset = FxAsset::from_snapshot_bytes(&bytes).unwrap();
        assert_eq!(restored_asset, asset);
        assert_eq!(restored_asset.support_nodes()[0].chunk_id, ChunkId(20));
        assert!(
            restored_asset
                .chunk(ChunkId(10))
                .unwrap()
                .support_nodes
                .is_empty()
        );

        let mut family = FxFamily::instantiate(FxFamilyId(3), asset);
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
                target: FractureTarget::Chunk(ChunkId(10)),
                health_loss: 0.25,
                effective_length_loss: 0.0,
                source: DamageSource::Script,
            }],
        );
        let digest = family.deterministic_state_digest();
        let bytes = family
            .to_snapshot_bytes(SnapshotMode::Deterministic)
            .unwrap();
        let restored = FxFamily::from_snapshot_bytes(&bytes).unwrap();
        assert_eq!(restored, family);
        assert_eq!(restored.deterministic_state_digest(), digest);
        assert_eq!(restored.chunk_state(ChunkId(10)).unwrap().health, 0.75);
    }

    #[test]
    fn core_snapshot_rejects_corrupt_header() {
        let asset = asset_from_rows(&["#"], &[Some(0)]);
        let mut bytes = asset
            .to_snapshot_bytes(SnapshotMode::Deterministic)
            .unwrap();
        bytes[12] = 99;
        assert_eq!(
            FxAsset::from_snapshot_bytes(&bytes).unwrap_err(),
            FxCoreSnapshotError::UnsupportedMode(99)
        );
    }

    #[test]
    fn asset_snapshot_rejects_duplicate_support_node_id() {
        let mut asset = asset_from_rows(&["##"], &[Some(0), Some(1)]);
        asset.support_nodes[1].id = SupportNodeId(0);

        assert_eq!(
            FxAsset::from_snapshot_bytes(&wrapped_asset_payload(asset_snapshot_payload(&asset)))
                .unwrap_err(),
            FxCoreSnapshotError::Validation(ValidationError::DuplicateSupportNodeId(
                SupportNodeId(0)
            ))
        );
    }

    #[test]
    fn asset_snapshot_rejects_duplicate_chunk_id() {
        let mut asset = asset_from_rows(&["##"], &[Some(0), Some(1)]);
        asset.chunks[1].id = ChunkId(0);

        assert_eq!(
            FxAsset::from_snapshot_bytes(&wrapped_asset_payload(asset_snapshot_payload(&asset)))
                .unwrap_err(),
            FxCoreSnapshotError::Validation(ValidationError::DuplicateChunkId(ChunkId(0)))
        );
    }

    #[test]
    fn asset_snapshot_rejects_overlapping_support_ancestors() {
        let mut asset = asset_from_rows(&["##"], &[Some(0), Some(1)]);
        asset.chunks[1].parent = Some(ChunkId(0));

        assert_eq!(
            FxAsset::from_snapshot_bytes(&wrapped_asset_payload(asset_snapshot_payload(&asset)))
                .unwrap_err(),
            FxCoreSnapshotError::Validation(ValidationError::ChunkHierarchyNotExact)
        );
    }

    #[test]
    fn asset_snapshot_rejects_non_contiguous_internal_bond_id() {
        let mut asset = asset_from_rows(&["###"], &[Some(0), Some(1), Some(2)]);
        asset.internal_bonds[1].id = BondId(3);

        assert_eq!(
            FxAsset::from_snapshot_bytes(&wrapped_asset_payload(asset_snapshot_payload(&asset)))
                .unwrap_err(),
            FxCoreSnapshotError::Validation(ValidationError::NonContiguousInternalBondId {
                expected: BondId(1),
                actual: BondId(3),
            })
        );
    }

    #[test]
    fn asset_snapshot_rejects_duplicate_internal_bond_id() {
        let mut asset = asset_from_rows(&["###"], &[Some(0), Some(1), Some(2)]);
        asset.internal_bonds[1].id = BondId(0);

        assert_eq!(
            FxAsset::from_snapshot_bytes(&wrapped_asset_payload(asset_snapshot_payload(&asset)))
                .unwrap_err(),
            FxCoreSnapshotError::Validation(ValidationError::DuplicateInternalBondId(BondId(0)))
        );
    }

    #[test]
    fn asset_snapshot_rejects_non_canonical_internal_bond_endpoints() {
        let mut asset = asset_from_rows(&["##"], &[Some(0), Some(1)]);
        asset.internal_bonds[0].node_a = SupportNodeId(1);
        asset.internal_bonds[0].node_b = SupportNodeId(0);

        assert_eq!(
            FxAsset::from_snapshot_bytes(&wrapped_asset_payload(asset_snapshot_payload(&asset)))
                .unwrap_err(),
            FxCoreSnapshotError::Validation(ValidationError::NonCanonicalBondEndpoints(BondId(0)))
        );
    }

    #[test]
    fn asset_snapshot_rejects_self_internal_bond() {
        let mut asset = asset_from_rows(&["##"], &[Some(0), Some(1)]);
        asset.internal_bonds[0].node_b = SupportNodeId(0);

        assert_eq!(
            FxAsset::from_snapshot_bytes(&wrapped_asset_payload(asset_snapshot_payload(&asset)))
                .unwrap_err(),
            FxCoreSnapshotError::Validation(ValidationError::SelfBond(SupportNodeId(0)))
        );
    }

    #[test]
    fn asset_snapshot_rejects_invalid_internal_bond_scalar() {
        let asset = asset_from_rows(&["##"], &[Some(0), Some(1)]);
        let mut payload = asset_snapshot_payload(&asset);
        patch_first(
            &mut payload,
            &10.0f32.to_bits().to_le_bytes(),
            &(-1.0f32).to_bits().to_le_bytes(),
        );

        assert_eq!(
            FxAsset::from_snapshot_bytes(&wrapped_asset_payload(payload)).unwrap_err(),
            FxCoreSnapshotError::Validation(ValidationError::InvalidBondScalar(BondId(0)))
        );
    }

    #[test]
    fn asset_snapshot_rejects_invalid_internal_bond_direction() {
        let mut asset = asset_from_rows(&["##"], &[Some(0), Some(1)]);
        asset.internal_bonds[0].normal = Vec2::new(2.0, 0.0);

        assert_eq!(
            FxAsset::from_snapshot_bytes(&wrapped_asset_payload(asset_snapshot_payload(&asset)))
                .unwrap_err(),
            FxCoreSnapshotError::Validation(ValidationError::InvalidBondDirection(BondId(0)))
        );
    }

    #[test]
    fn family_snapshot_rejects_stale_next_actor_id() {
        let family = FxFamily::instantiate(
            FxFamilyId(3),
            asset_from_rows(&["#.#"], &[Some(0), None, Some(1)]),
        );
        let mut payload = family_snapshot_payload(&family);
        payload[4..8].copy_from_slice(&1u32.to_le_bytes());
        assert!(matches!(
            FxFamily::from_snapshot_bytes(&wrapped_family_payload(payload)),
            Err(FxCoreSnapshotError::StateMismatch(
                "next_actor_id would overwrite an existing actor"
            ))
        ));
    }

    #[test]
    fn family_snapshot_rejects_actor_owner_bijection_mismatch() {
        let family = FxFamily::instantiate(
            FxFamilyId(3),
            asset_from_rows(&["###"], &[Some(0), Some(1), Some(2)]),
        );
        let mut duplicate_payload = family_snapshot_payload(&family);
        let needle = [
            3u32.to_le_bytes(),
            0u32.to_le_bytes(),
            1u32.to_le_bytes(),
            2u32.to_le_bytes(),
        ]
        .concat();
        let replacement = [
            3u32.to_le_bytes(),
            0u32.to_le_bytes(),
            1u32.to_le_bytes(),
            1u32.to_le_bytes(),
        ]
        .concat();
        patch_first(&mut duplicate_payload, &needle, &replacement);
        assert!(matches!(
            FxFamily::from_snapshot_bytes(&wrapped_family_payload(duplicate_payload)),
            Err(FxCoreSnapshotError::StateMismatch(
                "duplicate actor-owned node"
            ))
        ));

        let unknown_family = FxFamily::instantiate(
            FxFamilyId(3),
            asset_from_rows(&["#.#"], &[Some(0), None, Some(1)]),
        );
        let mut unknown_owner_payload = family_snapshot_payload(&unknown_family);
        let node_owner = [1u32.to_le_bytes(), 1u32.to_le_bytes()].concat();
        let unknown_owner = [1u32.to_le_bytes(), 99u32.to_le_bytes()].concat();
        patch_last(&mut unknown_owner_payload, &node_owner, &unknown_owner);
        assert!(matches!(
            FxFamily::from_snapshot_bytes(&wrapped_family_payload(unknown_owner_payload)),
            Err(FxCoreSnapshotError::StateMismatch(_))
        ));
    }

    #[test]
    fn family_snapshot_rejects_invalid_connection_decode() {
        let mut dynamic = FxFamily::instantiate(
            FxFamilyId(3),
            asset_from_rows(&["#.#"], &[Some(0), None, Some(1)]),
        );
        dynamic
            .connect_dynamic_structural_bond_graph_only(DynamicStructuralBondDesc {
                id: ConnectionId(9),
                node_a: SupportNodeId(0),
                node_b: SupportNodeId(1),
                centroid: Vec2::new(1.0, 0.5),
                normal: Vec2::new(1.0, 0.0),
                health: 1.0,
                effective_length: 1.0,
                tension_limit: 1.0,
                compression_limit: 1.0,
                shear_limit: 1.0,
            })
            .unwrap();
        let mut custom_policy = family_snapshot_payload(&dynamic);
        let custom_offset = dynamic_bond_offset(&custom_policy, ConnectionId(9));
        custom_policy[custom_offset + 16] = 1;
        assert!(matches!(
            FxFamily::from_snapshot_bytes(&wrapped_family_payload(custom_policy)),
            Err(FxCoreSnapshotError::UnsupportedCustomHardConstraint(
                ConnectionId(9)
            ))
        ));

        let mut self_connection = family_snapshot_payload(&dynamic);
        let self_offset = dynamic_bond_offset(&self_connection, ConnectionId(9));
        self_connection[self_offset + 12..self_offset + 16].copy_from_slice(&0u32.to_le_bytes());
        assert!(matches!(
            FxFamily::from_snapshot_bytes(&wrapped_family_payload(self_connection)),
            Err(FxCoreSnapshotError::StateMismatch(
                "dynamic bond endpoints must be canonical and non-self"
            ))
        ));

        let mut external =
            FxFamily::instantiate(FxFamilyId(4), asset_from_rows(&["#"], &[Some(0)]));
        external
            .connect_static_anchor(StaticAnchorDesc {
                id: ExternalBondId(5),
                node: SupportNodeId(0),
                target: ExternalTarget2D {
                    kind: ExternalTargetKind::World,
                    token: ExternalTargetToken(0),
                },
                anchor: Vec2::new(0.5, 0.5),
                normal: Vec2::new(1.0, 0.0),
                health: 1.0,
                effective_length: 1.0,
                tension_limit: 1.0,
                compression_limit: 1.0,
                shear_limit: 1.0,
            })
            .unwrap();
        external
            .external_bonds
            .get_mut(&ExternalBondId(5))
            .unwrap()
            .tangent = Vec2::new(1.0, 0.0);
        assert!(matches!(
            validate_family_snapshot(&external),
            Err(FxCoreSnapshotError::InvalidValue("external_bond.direction"))
        ));
    }

    #[test]
    fn snapshot_decode_rejects_checksum_trailing_and_invalid_option() {
        let asset = asset_from_rows(&["#"], &[Some(0)]);
        let mut checksum = asset
            .to_snapshot_bytes(SnapshotMode::Deterministic)
            .unwrap();
        let last = checksum.last_mut().unwrap();
        *last ^= 0x80;
        assert_eq!(
            FxAsset::from_snapshot_bytes(&checksum).unwrap_err(),
            FxCoreSnapshotError::PayloadChecksumMismatch
        );

        let mut payload = {
            let bytes = asset
                .to_snapshot_bytes(SnapshotMode::Deterministic)
                .unwrap();
            unwrap_payload(&bytes, MAGIC_ASSET).unwrap().1.to_vec()
        };
        payload.push(0);
        assert_eq!(
            FxAsset::from_snapshot_bytes(&wrap_payload(
                MAGIC_ASSET,
                SnapshotMode::Deterministic,
                0,
                payload
            ))
            .unwrap_err(),
            FxCoreSnapshotError::TrailingBytes
        );

        let mut invalid_option_payload = {
            let bytes = asset
                .to_snapshot_bytes(SnapshotMode::Deterministic)
                .unwrap();
            unwrap_payload(&bytes, MAGIC_ASSET).unwrap().1.to_vec()
        };
        let valid_option = [1u8, 0, 0, 0, 0];
        let invalid_option = [2u8, 0, 0, 0, 0];
        patch_first(&mut invalid_option_payload, &valid_option, &invalid_option);
        assert_eq!(
            FxAsset::from_snapshot_bytes(&wrap_payload(
                MAGIC_ASSET,
                SnapshotMode::Deterministic,
                0,
                invalid_option_payload
            ))
            .unwrap_err(),
            FxCoreSnapshotError::InvalidValue("voxel_to_node")
        );
    }
}
