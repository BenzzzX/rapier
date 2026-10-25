//! Stateless bridge to the existing voxel authoring, bond breaking and actor
//! splitting code. Labels preserve every source voxel exactly once; physics,
//! terrain ownership and sleeping remain the caller's responsibility.
use super::{AlchemyRapierStatus, actor_cells};
use fracture_core::{
    CommandId, DamageSource, DeterministicOrderKey, FractureCommand, FractureTarget, FxFamilyId,
};
use fracture_voxel::{
    VoxelAuthoringInput, VoxelAuthoringOptions, VoxelClusterPolicy, VoxelRuntime,
    author_voxel_asset_with_options,
};
use std::{
    panic::{AssertUnwindSafe, catch_unwind},
    slice,
};

#[repr(C)]
#[derive(Clone, Copy)]
pub struct AlchemyRockPartitionResult {
    pub status: AlchemyRapierStatus,
    pub fragment_count: u32,
}

#[unsafe(no_mangle)]
pub extern "C" fn alchemy_rapier_partition_brittle_rock(
    width: u32,
    height: u32,
    material_ids: *const u16,
    cell_count: usize,
    seed: u64,
    labels: *mut u32,
    label_capacity: usize,
) -> AlchemyRockPartitionResult {
    let fail = |status| AlchemyRockPartitionResult {
        status,
        fragment_count: 0,
    };
    match catch_unwind(AssertUnwindSafe(|| {
        let Some(n) = (width as usize).checked_mul(height as usize) else {
            return fail(AlchemyRapierStatus::InvalidArgument);
        };
        if width == 0
            || height == 0
            || n > 65536
            || cell_count != n
            || label_capacity < n
            || material_ids.is_null()
            || labels.is_null()
        {
            return fail(AlchemyRapierStatus::InvalidArgument);
        }
        let materials = unsafe { slice::from_raw_parts(material_ids, n) };
        let occupancy: Vec<bool> = materials.iter().map(|m| *m != 0).collect();
        let occupied = occupancy.iter().filter(|v| **v).count();
        if occupied == 0 {
            return fail(AlchemyRapierStatus::InvalidArgument);
        }
        // Reuse the library's grain-aware natural fracture clusters, with a
        // finite body budget. No gameplay-side Voronoi or connectivity solver.
        let seeds = occupied.div_ceil(180).clamp(2, 16).min(occupied);
        let mut options = VoxelAuthoringOptions::default();
        for material in materials.iter().copied().filter(|m| *m != 0) {
            options
                .material_cluster_rules
                .entry(material)
                .or_insert_with(|| VoxelClusterPolicy::natural_voronoi_generated(seeds, seed));
        }
        let input = VoxelAuthoringInput::new(
            width,
            height,
            1.0,
            occupancy,
            materials.to_vec(),
            materials.to_vec(),
            vec![0; n],
        );
        let Ok(asset) = author_voxel_asset_with_options(input, options) else {
            return fail(AlchemyRapierStatus::InvalidArgument);
        };
        let mut runtime = VoxelRuntime::instantiate(FxFamilyId(1), asset);
        let commands: Vec<FractureCommand> = runtime
            .asset()
            .core()
            .internal_bonds()
            .iter()
            .filter_map(|bond| {
                let actor = runtime.family().node_owner(bond.node_a)?;
                Some(FractureCommand {
                    order_key: DeterministicOrderKey::new(
                        0,
                        0,
                        FxFamilyId(1),
                        actor,
                        CommandId(bond.id.0),
                    ),
                    actor,
                    target: FractureTarget::Bond(bond.id),
                    health_loss: f32::MAX,
                    effective_length_loss: f32::MAX,
                    source: DamageSource::Script,
                })
            })
            .collect();
        runtime.apply_fracture_commands(&commands);
        runtime.split_dirty_actors();
        let mut result = vec![0u32; n];
        let mut fragment_count = 0;
        for (actor, _) in runtime.family().actors() {
            fragment_count += 1;
            let Some(cells) = actor_cells(&runtime, *actor) else {
                return fail(AlchemyRapierStatus::Panic);
            };
            for (_, index, _) in cells {
                if index >= n || result[index] != 0 {
                    return fail(AlchemyRapierStatus::Panic);
                }
                result[index] = fragment_count;
            }
        }
        if result.iter().filter(|label| **label != 0).count() != occupied {
            return fail(AlchemyRapierStatus::Panic);
        }
        unsafe { slice::from_raw_parts_mut(labels, n) }.copy_from_slice(&result);
        AlchemyRockPartitionResult {
            status: AlchemyRapierStatus::Ok,
            fragment_count,
        }
    })) {
        Ok(result) => result,
        Err(_) => fail(AlchemyRapierStatus::Panic),
    }
}
