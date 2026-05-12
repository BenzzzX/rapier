use thiserror::Error;

use super::*;

const MAGIC: [u8; 8] = *b"RFXSVX\0\0";
const VERSION: u16 = 1;
const HEADER_LEN: usize = 34;

#[derive(Clone, Debug, PartialEq)]
pub struct AuthoredVoxelAssetSnapshot {
    pub bytes: Vec<u8>,
}

#[derive(Error, Debug, Clone, PartialEq)]
pub enum VoxelSnapshotError {
    #[error("snapshot magic is invalid")]
    InvalidMagic,
    #[error("snapshot version {0} is unsupported")]
    UnsupportedVersion(u16),
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
    #[error("snapshot restored authored asset does not match encoded state")]
    StateMismatch,
    #[error(transparent)]
    Voxel(#[from] VoxelError),
}

impl AuthoredVoxelAsset {
    pub fn to_snapshot_bytes(&self) -> Result<Vec<u8>, VoxelSnapshotError> {
        validate_finite(self)?;
        let mut writer = Writer::new();
        writer.u32(self.width);
        writer.u32(self.height);
        writer.f32(self.core.voxel_size(), "voxel_size")?;
        write_bool_vec(&mut writer, &self.occupancy())?;
        write_u16_vec(&mut writer, &self.fracture_material)?;
        write_u16_vec(&mut writer, &self.contact_material)?;
        write_u32_vec(&mut writer, &self.external_id)?;
        match &self.orientation {
            Some(map) => {
                writer.u8(1);
                write_u16_vec(&mut writer, map)?;
            }
            None => writer.u8(0),
        }
        writer.len(self.core.voxel_to_node_map().len())?;
        for node in self.core.voxel_to_node_map() {
            match node {
                Some(node) => {
                    writer.u8(1);
                    writer.u32(node.0);
                }
                None => writer.u8(0),
            }
        }
        writer.f32(self.default_bond_health, "default_bond_health")?;
        writer.f32(self.default_tension_limit, "default_tension_limit")?;
        writer.f32(self.default_shear_limit, "default_shear_limit")?;
        Ok(wrap(writer.bytes))
    }

    pub fn from_snapshot_bytes(bytes: &[u8]) -> Result<Self, VoxelSnapshotError> {
        let payload = unwrap(bytes)?;
        let mut reader = Reader::new(payload);
        let width = reader.u32("width")?;
        let height = reader.u32("height")?;
        let voxel_size = reader.f32("voxel_size")?;
        let cell_count = checked_cell_count(width, height)?;
        let occupancy = reader.bool_vec_exact("occupancy", cell_count)?;
        let fracture_material = reader.u16_vec_exact("fracture_material", cell_count)?;
        let contact_material = reader.u16_vec_exact("contact_material", cell_count)?;
        let external_id = reader.u32_vec_exact("external_id", cell_count)?;
        let orientation = match reader.u8("orientation.tag")? {
            0 => None,
            1 => Some(reader.u16_vec_exact("orientation", cell_count)?),
            _ => return Err(VoxelSnapshotError::InvalidValue("orientation.tag")),
        };
        let support_node_hint =
            reader.support_node_hint_vec_exact("support_node_hint", cell_count)?;
        let default_bond_health = reader.f32("default_bond_health")?;
        let default_tension_limit = reader.f32("default_tension_limit")?;
        let default_shear_limit = reader.f32("default_shear_limit")?;
        reader.finish()?;

        let encoded_occupancy = occupancy.clone();
        let encoded_fracture_material = fracture_material.clone();
        let encoded_contact_material = contact_material.clone();
        let encoded_external_id = external_id.clone();
        let encoded_orientation = orientation.clone();
        let encoded_support_node_hint = support_node_hint.clone();
        let mut input = VoxelAuthoringInput::new(
            width,
            height,
            voxel_size,
            occupancy,
            fracture_material,
            contact_material,
            external_id,
        );
        input.orientation = orientation;
        input.support_node_hint = Some(support_node_hint);
        input.default_bond_health = default_bond_health;
        input.default_tension_limit = default_tension_limit;
        input.default_shear_limit = default_shear_limit;
        let asset = author_voxel_asset(input)?;
        validate_finite(&asset)?;
        validate_restored_matches_encoded(
            &asset,
            width,
            height,
            voxel_size,
            &encoded_occupancy,
            &encoded_fracture_material,
            &encoded_contact_material,
            &encoded_external_id,
            encoded_orientation.as_deref(),
            &encoded_support_node_hint,
            default_bond_health,
            default_tension_limit,
            default_shear_limit,
        )?;
        Ok(asset)
    }
}

fn validate_restored_matches_encoded(
    asset: &AuthoredVoxelAsset,
    width: u32,
    height: u32,
    voxel_size: f32,
    occupancy: &[bool],
    fracture_material: &[u16],
    contact_material: &[u16],
    external_id: &[u32],
    orientation: Option<&[u16]>,
    support_node_hint: &[Option<u32>],
    default_bond_health: f32,
    default_tension_limit: f32,
    default_shear_limit: f32,
) -> Result<(), VoxelSnapshotError> {
    let encoded_node_map = support_node_hint
        .iter()
        .map(|hint| hint.map(SupportNodeId))
        .collect::<Vec<_>>();
    if asset.width != width
        || asset.height != height
        || asset.core.voxel_size().to_bits() != voxel_size.to_bits()
        || asset.occupancy() != occupancy
        || asset.fracture_material_map() != fracture_material
        || asset.contact_material_map() != contact_material
        || asset.external_id_map() != external_id
        || asset.orientation_map() != orientation
        || asset.core.voxel_to_node_map() != encoded_node_map
        || asset.default_bond_health.to_bits() != default_bond_health.to_bits()
        || asset.default_tension_limit.to_bits() != default_tension_limit.to_bits()
        || asset.default_shear_limit.to_bits() != default_shear_limit.to_bits()
    {
        return Err(VoxelSnapshotError::StateMismatch);
    }

    let expected = AuthoredVoxelAsset {
        core: asset.core.clone(),
        width,
        height,
        contact_material: contact_material.to_vec(),
        fracture_material: fracture_material.to_vec(),
        external_id: external_id.to_vec(),
        orientation: orientation.map(|map| map.to_vec()),
        summaries: node_summaries(&asset.core, contact_material, external_id, width),
        bond_summaries: bond_summaries(&asset.core, contact_material, external_id, width, height),
        default_bond_health,
        default_tension_limit,
        default_shear_limit,
    };
    if asset.core.support_nodes() != expected.core.support_nodes()
        || asset.core.internal_bonds() != expected.core.internal_bonds()
        || asset.node_summaries() != expected.node_summaries()
        || asset.bond_summaries() != expected.bond_summaries()
    {
        return Err(VoxelSnapshotError::StateMismatch);
    }
    Ok(())
}

fn validate_finite(asset: &AuthoredVoxelAsset) -> Result<(), VoxelSnapshotError> {
    if !asset.core.voxel_size().is_finite()
        || asset.core.voxel_size() <= 0.0
        || !asset.default_bond_health.is_finite()
        || !asset.default_tension_limit.is_finite()
        || !asset.default_shear_limit.is_finite()
    {
        return Err(VoxelSnapshotError::InvalidValue("asset"));
    }
    Ok(())
}

fn write_bool_vec(writer: &mut Writer, values: &[bool]) -> Result<(), VoxelSnapshotError> {
    writer.len(values.len())?;
    for value in values {
        writer.u8(u8::from(*value));
    }
    Ok(())
}

fn write_u16_vec(writer: &mut Writer, values: &[u16]) -> Result<(), VoxelSnapshotError> {
    writer.len(values.len())?;
    for value in values {
        writer.u16(*value);
    }
    Ok(())
}

fn write_u32_vec(writer: &mut Writer, values: &[u32]) -> Result<(), VoxelSnapshotError> {
    writer.len(values.len())?;
    for value in values {
        writer.u32(*value);
    }
    Ok(())
}

fn checked_cell_count(width: u32, height: u32) -> Result<usize, VoxelSnapshotError> {
    let count = u64::from(width)
        .checked_mul(u64::from(height))
        .ok_or(VoxelSnapshotError::InvalidValue("grid"))?;
    usize::try_from(count).map_err(|_| VoxelSnapshotError::InvalidValue("grid"))
}

fn wrap(payload: Vec<u8>) -> Vec<u8> {
    let mut out = Vec::with_capacity(HEADER_LEN + payload.len());
    out.extend_from_slice(&MAGIC);
    out.extend_from_slice(&VERSION.to_le_bytes());
    out.push(2);
    out.push(4);
    out.push(1);
    out.push(0);
    out.extend_from_slice(&0u32.to_le_bytes());
    out.extend_from_slice(&(payload.len() as u64).to_le_bytes());
    out.extend_from_slice(&checksum(&payload).to_le_bytes());
    out.extend_from_slice(&payload);
    out
}

fn unwrap(bytes: &[u8]) -> Result<&[u8], VoxelSnapshotError> {
    if bytes.len() < HEADER_LEN {
        return Err(VoxelSnapshotError::UnexpectedEof("header"));
    }
    if bytes[0..8] != MAGIC {
        return Err(VoxelSnapshotError::InvalidMagic);
    }
    let version = u16::from_le_bytes([bytes[8], bytes[9]]);
    if version != VERSION {
        return Err(VoxelSnapshotError::UnsupportedVersion(version));
    }
    if bytes[10] != 2 || bytes[11] != 4 {
        return Err(VoxelSnapshotError::InvalidValue("header"));
    }
    if bytes[12] > 1 {
        return Err(VoxelSnapshotError::UnsupportedMode(bytes[12]));
    }
    if bytes[13] != 0 {
        return Err(VoxelSnapshotError::UnsupportedFlags(bytes[13] as u32));
    }
    let flags = u32::from_le_bytes([bytes[14], bytes[15], bytes[16], bytes[17]]);
    if flags != 0 {
        return Err(VoxelSnapshotError::UnsupportedFlags(flags));
    }
    let len_u64 = u64::from_le_bytes([
        bytes[18], bytes[19], bytes[20], bytes[21], bytes[22], bytes[23], bytes[24], bytes[25],
    ]);
    let len = usize::try_from(len_u64).map_err(|_| VoxelSnapshotError::PayloadLengthMismatch)?;
    let expected_len = HEADER_LEN
        .checked_add(len)
        .ok_or(VoxelSnapshotError::PayloadLengthMismatch)?;
    if bytes.len() != expected_len {
        return Err(VoxelSnapshotError::PayloadLengthMismatch);
    }
    let expected = u64::from_le_bytes([
        bytes[26], bytes[27], bytes[28], bytes[29], bytes[30], bytes[31], bytes[32], bytes[33],
    ]);
    let payload = &bytes[HEADER_LEN..];
    if checksum(payload) != expected {
        return Err(VoxelSnapshotError::PayloadChecksumMismatch);
    }
    Ok(payload)
}

struct Writer {
    bytes: Vec<u8>,
}

impl Writer {
    fn new() -> Self {
        Self { bytes: Vec::new() }
    }

    fn u8(&mut self, value: u8) {
        self.bytes.push(value);
    }

    fn u16(&mut self, value: u16) {
        self.bytes.extend_from_slice(&value.to_le_bytes());
    }

    fn u32(&mut self, value: u32) {
        self.bytes.extend_from_slice(&value.to_le_bytes());
    }

    fn f32(&mut self, value: f32, field: &'static str) -> Result<(), VoxelSnapshotError> {
        if !value.is_finite() {
            return Err(VoxelSnapshotError::InvalidValue(field));
        }
        self.u32(value.to_bits());
        Ok(())
    }

    fn len(&mut self, len: usize) -> Result<(), VoxelSnapshotError> {
        let len = u32::try_from(len).map_err(|_| VoxelSnapshotError::InvalidValue("length"))?;
        self.u32(len);
        Ok(())
    }
}

struct Reader<'a> {
    bytes: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, pos: 0 }
    }

    fn finish(&self) -> Result<(), VoxelSnapshotError> {
        if self.pos == self.bytes.len() {
            Ok(())
        } else {
            Err(VoxelSnapshotError::TrailingBytes)
        }
    }

    fn u8(&mut self, field: &'static str) -> Result<u8, VoxelSnapshotError> {
        Ok(self.take(1, field)?[0])
    }

    fn u16(&mut self, field: &'static str) -> Result<u16, VoxelSnapshotError> {
        let bytes = self.take(2, field)?;
        Ok(u16::from_le_bytes([bytes[0], bytes[1]]))
    }

    fn u32(&mut self, field: &'static str) -> Result<u32, VoxelSnapshotError> {
        let bytes = self.take(4, field)?;
        Ok(u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
    }

    fn f32(&mut self, field: &'static str) -> Result<f32, VoxelSnapshotError> {
        let value = f32::from_bits(self.u32(field)?);
        if value.is_finite() {
            Ok(value)
        } else {
            Err(VoxelSnapshotError::InvalidValue(field))
        }
    }

    fn len(&mut self, field: &'static str) -> Result<usize, VoxelSnapshotError> {
        Ok(self.u32(field)? as usize)
    }

    fn bool_vec_exact(
        &mut self,
        field: &'static str,
        expected: usize,
    ) -> Result<Vec<bool>, VoxelSnapshotError> {
        let len = self.exact_len(field, expected)?;
        self.ensure_elements_available(len, 1, field)?;
        let mut out = Vec::with_capacity(len);
        for _ in 0..len {
            out.push(match self.u8(field)? {
                0 => false,
                1 => true,
                _ => return Err(VoxelSnapshotError::InvalidValue(field)),
            });
        }
        Ok(out)
    }

    fn u16_vec_exact(
        &mut self,
        field: &'static str,
        expected: usize,
    ) -> Result<Vec<u16>, VoxelSnapshotError> {
        let len = self.exact_len(field, expected)?;
        self.ensure_elements_available(len, 2, field)?;
        let mut out = Vec::with_capacity(len);
        for _ in 0..len {
            out.push(self.u16(field)?);
        }
        Ok(out)
    }

    fn u32_vec_exact(
        &mut self,
        field: &'static str,
        expected: usize,
    ) -> Result<Vec<u32>, VoxelSnapshotError> {
        let len = self.exact_len(field, expected)?;
        self.ensure_elements_available(len, 4, field)?;
        let mut out = Vec::with_capacity(len);
        for _ in 0..len {
            out.push(self.u32(field)?);
        }
        Ok(out)
    }

    fn support_node_hint_vec_exact(
        &mut self,
        field: &'static str,
        expected: usize,
    ) -> Result<Vec<Option<u32>>, VoxelSnapshotError> {
        let len = self.exact_len(field, expected)?;
        self.ensure_support_node_hints_available(len, field)?;
        let mut out = Vec::with_capacity(len);
        for _ in 0..len {
            out.push(match self.u8("support_node_hint.tag")? {
                0 => None,
                1 => Some(self.u32(field)?),
                _ => return Err(VoxelSnapshotError::InvalidValue("support_node_hint.tag")),
            });
        }
        Ok(out)
    }

    fn exact_len(
        &mut self,
        field: &'static str,
        expected: usize,
    ) -> Result<usize, VoxelSnapshotError> {
        let len = self.len(field)?;
        if len != expected {
            return Err(VoxelSnapshotError::InvalidValue(field));
        }
        Ok(len)
    }

    fn remaining(&self) -> usize {
        self.bytes.len() - self.pos
    }

    fn ensure_elements_available(
        &self,
        len: usize,
        element_width: usize,
        field: &'static str,
    ) -> Result<(), VoxelSnapshotError> {
        let byte_count = len
            .checked_mul(element_width)
            .ok_or(VoxelSnapshotError::UnexpectedEof(field))?;
        if byte_count > self.remaining() {
            return Err(VoxelSnapshotError::UnexpectedEof(field));
        }
        Ok(())
    }

    fn ensure_support_node_hints_available(
        &self,
        len: usize,
        field: &'static str,
    ) -> Result<(), VoxelSnapshotError> {
        let mut cursor = self.pos;
        for _ in 0..len {
            if cursor >= self.bytes.len() {
                return Err(VoxelSnapshotError::UnexpectedEof("support_node_hint.tag"));
            }
            match self.bytes[cursor] {
                0 => cursor += 1,
                1 => {
                    cursor = cursor
                        .checked_add(5)
                        .ok_or(VoxelSnapshotError::UnexpectedEof(field))?;
                    if cursor > self.bytes.len() {
                        return Err(VoxelSnapshotError::UnexpectedEof(field));
                    }
                }
                _ => return Err(VoxelSnapshotError::InvalidValue("support_node_hint.tag")),
            }
        }
        Ok(())
    }

    fn take(&mut self, len: usize, field: &'static str) -> Result<&'a [u8], VoxelSnapshotError> {
        let end = self
            .pos
            .checked_add(len)
            .ok_or(VoxelSnapshotError::UnexpectedEof(field))?;
        if end > self.bytes.len() {
            return Err(VoxelSnapshotError::UnexpectedEof(field));
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

    fn encoded_asset_bytes(
        fracture_material: Vec<u16>,
        support_node_hint: Vec<Option<u32>>,
    ) -> Vec<u8> {
        let mut writer = Writer::new();
        writer.u32(2);
        writer.u32(1);
        writer.f32(1.0, "voxel_size").unwrap();
        write_bool_vec(&mut writer, &[true, true]).unwrap();
        write_u16_vec(&mut writer, &fracture_material).unwrap();
        write_u16_vec(&mut writer, &[7, 8]).unwrap();
        write_u32_vec(&mut writer, &[42, 43]).unwrap();
        writer.u8(0);
        writer.len(support_node_hint.len()).unwrap();
        for hint in support_node_hint {
            match hint {
                Some(id) => {
                    writer.u8(1);
                    writer.u32(id);
                }
                None => writer.u8(0),
            }
        }
        writer.f32(1.0, "default_bond_health").unwrap();
        writer.f32(10.0, "default_tension_limit").unwrap();
        writer.f32(10.0, "default_shear_limit").unwrap();
        wrap(writer.bytes)
    }

    #[test]
    fn voxel_asset_snapshot_roundtrip_preserves_exact_cover_and_metadata() {
        let mut input = VoxelAuthoringInput::new(
            3,
            1,
            0.5,
            vec![true, true, true],
            vec![1, 1, 2],
            vec![7, 8, 9],
            vec![42, 43, 44],
        );
        input.orientation = Some(vec![10, 11, 12]);
        input.support_node_hint = Some(vec![Some(4), Some(4), Some(8)]);
        input.default_bond_health = 3.0;
        input.default_tension_limit = 4.0;
        input.default_shear_limit = 5.0;
        let asset = author_voxel_asset(input).unwrap();
        let bytes = asset.to_snapshot_bytes().unwrap();
        let restored = AuthoredVoxelAsset::from_snapshot_bytes(&bytes).unwrap();
        restored.validate_exact_cover().unwrap();
        assert_eq!(restored.occupancy(), asset.occupancy());
        assert_eq!(
            restored.contact_material_map(),
            asset.contact_material_map()
        );
        assert_eq!(restored.external_id_map(), asset.external_id_map());
        assert_eq!(restored.orientation_map(), asset.orientation_map());
        assert_eq!(restored.node_summaries(), asset.node_summaries());
        assert_eq!(restored.bond_summaries(), asset.bond_summaries());
        assert_eq!(restored.core(), asset.core());
    }

    #[test]
    fn voxel_snapshot_rejects_support_hint_normalization() {
        let bytes = encoded_asset_bytes(vec![1, 2], vec![Some(7), Some(7)]);
        assert_eq!(
            AuthoredVoxelAsset::from_snapshot_bytes(&bytes).unwrap_err(),
            VoxelSnapshotError::StateMismatch
        );
    }

    #[test]
    fn voxel_snapshot_rejects_huge_length_checksum_and_invalid_bool() {
        let valid = encoded_asset_bytes(vec![1, 1], vec![Some(7), Some(7)]);
        let mut huge = valid.clone();
        huge[18..26].copy_from_slice(&u64::MAX.to_le_bytes());
        assert_eq!(
            AuthoredVoxelAsset::from_snapshot_bytes(&huge).unwrap_err(),
            VoxelSnapshotError::PayloadLengthMismatch
        );

        let mut checksum = valid.clone();
        *checksum.last_mut().unwrap() ^= 1;
        assert_eq!(
            AuthoredVoxelAsset::from_snapshot_bytes(&checksum).unwrap_err(),
            VoxelSnapshotError::PayloadChecksumMismatch
        );

        let mut invalid_bool_payload = unwrap(&valid).unwrap().to_vec();
        // Payload layout begins with width, height, voxel_size, occupancy len, then bool cells.
        invalid_bool_payload[16] = 2;
        assert_eq!(
            AuthoredVoxelAsset::from_snapshot_bytes(&wrap(invalid_bool_payload)).unwrap_err(),
            VoxelSnapshotError::InvalidValue("occupancy")
        );
    }

    #[test]
    fn voxel_snapshot_rejects_inflated_internal_vector_len_before_allocation() {
        let valid = encoded_asset_bytes(vec![1, 1], vec![Some(7), Some(7)]);
        let mut payload = unwrap(&valid).unwrap().to_vec();
        let fracture_material_len_offset = 4 + 4 + 4 + 4 + 2;
        payload[fracture_material_len_offset..fracture_material_len_offset + 4]
            .copy_from_slice(&u32::MAX.to_le_bytes());

        assert_eq!(
            AuthoredVoxelAsset::from_snapshot_bytes(&wrap(payload)).unwrap_err(),
            VoxelSnapshotError::InvalidValue("fracture_material")
        );
    }
}
