use std::cmp::min;

use marten::Real;
use marten::level::OCTREE_CHUNK_SHIFT;
use rapier3d::glamx::{DVec3, IVec3};
use rapier3d::math::{Pose3, Vec3};
use rapier3d::na::SimdComplexField;
use rayon::iter::ParallelIterator;
use rayon::prelude::{IntoParallelRefIterator, ParallelExtend};

use crate::ActiveLevelColliderInfo;
use crate::scene::{SableSceneData, pack_section_pos};

pub const DEFAULT_COLLISION_PARALLEL_CUTOFF: usize = 256;

/// Detects the collision pairs of a sable body
pub fn find_collision_pairs(
    sable_body: &ActiveLevelColliderInfo,
    other_sable_body: Option<&ActiveLevelColliderInfo>,
    isometry: &Pose3,
    prediction: Real,
    cutoff: usize,
    liquid: bool,
    sable_data: &SableSceneData,
) -> Vec<(IVec3, IVec3)> {
    struct StackObject {
        index: u32,
        depth: u32,
        min: IVec3,
    }

    let Some(octree) = &sable_body.octree else {
        panic!("No octree!")
    };
    crate::step_stats::note_octree_log_size(octree.log_size);

    let local_bounds_min = sable_body.local_bounds_min.unwrap();

    let center_of_mass = sable_body.center_of_mass.unwrap();

    let offset = DVec3::new(
        local_bounds_min.x as f64 - center_of_mass.x,
        local_bounds_min.y as f64 - center_of_mass.y,
        local_bounds_min.z as f64 - center_of_mass.z,
    );
    let offset = Vec3::new(offset.x as Real, offset.y as Real, offset.z as Real);

    let offset = isometry.rotation.mul_vec3(offset.into());
    let translation = isometry.translation + offset;

    // start with the root node
    let mut current_level = Vec::with_capacity(128);

    let com_offset: DVec3 = if let Some(other_handle) = other_sable_body {
        let com = other_handle.center_of_mass.unwrap();
        DVec3::new(com.x, com.y, com.z)
    } else {
        DVec3::ZERO
    };

    current_level.push(StackObject {
        index: 0,
        depth: 0,
        min: IVec3::ZERO,
    });

    let mut pairs = Vec::with_capacity(16);
    // process nodes level by level to maintain some structure while parallelizing
    while !current_level.is_empty() {
        type LevelData = (Option<Vec<StackObject>>, Option<Vec<(IVec3, IVec3)>>);
        let mut next_level_data = Vec::<LevelData>::with_capacity(8);

        let do_level_parallel = current_level.len() >= cutoff;

        crate::step_stats::PAIR_SEARCH_NODES.count(current_level.len() as u64);
        let process_stack_object = |entry: &StackObject| -> LevelData {
            let node = *unsafe { octree.buffer.get_unchecked(entry.index as usize) };
            if node == 0 {
                // Empty node: neither a leaf (pairs) nor a branch (children), so the overlap
                // query below would be thrown away. Most children of a sparse body are empty.
                return (None, None);
            }
            let node_size = 1 << (octree.log_size as u32 - entry.depth);

            // Calculate the center and radius for this node
            let node_center = entry.min.as_dvec3() + node_size as Real / 2.0;
            let transformed_center = isometry.rotation.mul_vec3(node_center) + translation;
            let radius = node_size as Real / 2.0 * 1.7321 + prediction;

            let (has_any_intersections, blocks_opt) = get_overlapping_nodes(
                other_sable_body,
                com_offset,
                transformed_center.into(),
                radius,
                sable_data,
                node >= 0,
                liquid,
            );

            if !has_any_intersections {
                return (None, None);
            }

            // leaf node - add collision pairs
            if node < 0 {
                let mut local_pairs = Vec::new();
                for static_block in blocks_opt.unwrap().iter() {
                    local_pairs.push((*static_block, entry.min + local_bounds_min));
                }

                return (None, Some(local_pairs));
            }

            if node > 0 {
                let mut local_next_level = Vec::with_capacity(8);

                for i in 0..8 {
                    local_next_level.push(StackObject {
                        index: (node + i) as u32,
                        depth: entry.depth + 1,
                        min: entry.min
                            + IVec3::new(
                                (i & 1) * node_size / 2,
                                ((i >> 1) & 1) * node_size / 2,
                                ((i >> 2) & 1) * node_size / 2,
                            ),
                    });
                }

                (Some(local_next_level), None)
            } else {
                (None, None)
            }
        };

        if do_level_parallel {
            next_level_data.par_extend(current_level.par_iter().map(process_stack_object))
        } else {
            next_level_data.extend(current_level.iter().map(process_stack_object))
        }

        let (a_parts, b_parts): (Vec<_>, Vec<_>) = next_level_data.into_iter().unzip();

        // filter out none's and add them
        for local_pairs in b_parts.into_iter().flatten() {
            pairs.extend(local_pairs);
        }

        current_level = a_parts.into_iter().flatten().flatten().collect();
    }

    pairs
}

fn get_overlapping_nodes(
    other_handle: Option<&ActiveLevelColliderInfo>,
    com_offset: DVec3,
    pos: Vec3,
    dist: Real,
    sable_data: &SableSceneData,
    cancel_early: bool,
    liquid: bool,
) -> (bool, Option<Vec<IVec3>>) {
    // biggest power of two that doesn't go over radius
    let log2 = ((dist * 2.0).simd_ln() / 2.0f64.simd_ln()).floor() as i32;

    let log2 = if let Some(other_handle) = other_handle {
        let Some(oct) = &other_handle.octree else {
            panic!("No octree!")
        };
        min(log2, oct.log_size)
    } else {
        min(log2, OCTREE_CHUNK_SHIFT)
    };

    let origin = if other_handle.is_some() {
        DVec3::ZERO
    } else {
        sable_data.origin
    };
    let min_block_pos = (pos - dist + com_offset + origin)
        .floor()
        .as_ivec3();
    let max_block_pos = (pos + dist + com_offset + origin)
        .floor()
        .as_ivec3();

    if let Some(other_handle) = other_handle {
        let other_min = other_handle.local_bounds_min.unwrap();

        let min_pos = ((min_block_pos - other_min) >> log2).map(|x| x.max(0));
        let max_pos = (max_block_pos - other_min) >> log2;

        let Some(oct) = &other_handle.octree else {
            panic!("No octree!")
        };

        let mut blocks = if cancel_early {
            None
        } else {
            Some(Vec::with_capacity(16))
        };

        for x in min_pos.x..=max_pos.x {
            for y in min_pos.y..=max_pos.y {
                for z in min_pos.z..=max_pos.z {
                    if oct.query(x << log2, y << log2, z << log2, log2) > -2 {
                        if cancel_early {
                            return (true, None);
                        } else {
                            blocks.as_mut().unwrap().push(IVec3::new(
                                (x << log2) + other_min.x,
                                (y << log2) + other_min.y,
                                (z << log2) + other_min.z,
                            ));
                        }
                    }
                }
            }
        }

        if cancel_early {
            return (false, None);
        } else {
            return (!blocks.as_ref().unwrap().is_empty(), blocks);
        }
    }

    // find all the octrees
    let min_octree_pos = min_block_pos >> OCTREE_CHUNK_SHIFT;
    let max_octree_pos = max_block_pos >> OCTREE_CHUNK_SHIFT;

    let mut blocks = if cancel_early {
        None
    } else {
        Some(Vec::with_capacity(8))
    };
    for ox in min_octree_pos.x..=max_octree_pos.x {
        for oy in min_octree_pos.y..=max_octree_pos.y {
            for oz in min_octree_pos.z..=max_octree_pos.z {
                crate::step_stats::PAIR_SEARCH_CHUNK_LOOKUPS.count(1);
                let chunk = sable_data.octree_chunks.get(&pack_section_pos(ox, oy, oz));
                let Some(chunk) = chunk else {
                    continue;
                };

                // Only the cells of the query range that lie inside this octree chunk: cells
                // outside it always miss (the chunk octree answers -2 out of bounds), so walking
                // the whole range for every chunk only multiplied the work by the chunk count
                // (millions of queries per step for a large body's top nodes). log2 <= the
                // chunk shift, so chunk borders fall on cell borders.
                let cells_per_chunk_shift = OCTREE_CHUNK_SHIFT - log2;
                let min_x = (min_block_pos.x >> log2).max(ox << cells_per_chunk_shift);
                let min_y = (min_block_pos.y >> log2).max(oy << cells_per_chunk_shift);
                let min_z = (min_block_pos.z >> log2).max(oz << cells_per_chunk_shift);
                let max_x = (max_block_pos.x >> log2).min(((ox + 1) << cells_per_chunk_shift) - 1);
                let max_y = (max_block_pos.y >> log2).min(((oy + 1) << cells_per_chunk_shift) - 1);
                let max_z = (max_block_pos.z >> log2).min(((oz + 1) << cells_per_chunk_shift) - 1);
                let chunk_octree = if liquid {
                    &chunk.liquid_octree
                } else {
                    &chunk.octree
                };

                if chunk_octree.is_empty() {
                    continue;
                }

                crate::step_stats::PAIR_SEARCH_QUERIES.count(
                    ((max_x - min_x + 1).max(0) as u64)
                        * ((max_y - min_y + 1).max(0) as u64)
                        * ((max_z - min_z + 1).max(0) as u64),
                );
                for x in min_x..=max_x {
                    for y in min_y..=max_y {
                        for z in min_z..=max_z {
                            if chunk_octree.query(
                                (x << log2) - (ox << OCTREE_CHUNK_SHIFT),
                                (y << log2) - (oy << OCTREE_CHUNK_SHIFT),
                                (z << log2) - (oz << OCTREE_CHUNK_SHIFT),
                                log2,
                            ) > -2
                            {
                                if cancel_early {
                                    return (true, None);
                                } else {
                                    blocks.as_mut().unwrap().push(IVec3::new(x, y, z) << log2);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    if cancel_early {
        (false, None)
    } else {
        (!blocks.as_ref().unwrap().is_empty(), blocks)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::joints::SableJointSet;
    use crate::rope::RopeMap;
    use marten::level::OctreeChunkSection;
    use std::collections::HashMap;

    /// Sparse terrain spanning octree chunks on both sides of zero.
    fn terrain() -> SableSceneData {
        let mut octree_chunks: HashMap<i64, OctreeChunkSection> = HashMap::new();
        for x in -70i32..70 {
            for y in -8..8 {
                for z in -70..70 {
                    if (x * 7 + y * 13 + z * 3).rem_euclid(11) != 0 {
                        continue;
                    }
                    let key = pack_section_pos(
                        x >> OCTREE_CHUNK_SHIFT,
                        y >> OCTREE_CHUNK_SHIFT,
                        z >> OCTREE_CHUNK_SHIFT,
                    );
                    let mask = (1 << OCTREE_CHUNK_SHIFT) - 1;
                    octree_chunks
                        .entry(key)
                        .or_insert_with(OctreeChunkSection::new)
                        .octree
                        .insert(x & mask, y & mask, z & mask, 1);
                }
            }
        }
        SableSceneData {
            main_level_chunks: HashMap::new(),
            octree_chunks,
            joint_set: SableJointSet::new(),
            rope_map: RopeMap::default(),
            level_colliders: HashMap::new(),
            rigid_bodies: HashMap::new(),
            origin: DVec3::ZERO,
        }
    }

    /// The terrain query before clipping: every chunk walks the whole cell range.
    fn unclipped(pos: Vec3, dist: Real, sable_data: &SableSceneData, cancel_early: bool) -> (bool, Option<Vec<IVec3>>) {
        let log2 = ((dist * 2.0).simd_ln() / 2.0f64.simd_ln()).floor() as i32;
        let log2 = min(log2, OCTREE_CHUNK_SHIFT);
        let min_block_pos = (pos - dist + sable_data.origin).floor().as_ivec3();
        let max_block_pos = (pos + dist + sable_data.origin).floor().as_ivec3();
        let min_octree_pos = min_block_pos >> OCTREE_CHUNK_SHIFT;
        let max_octree_pos = max_block_pos >> OCTREE_CHUNK_SHIFT;
        let mut blocks = Vec::new();
        for ox in min_octree_pos.x..=max_octree_pos.x {
            for oy in min_octree_pos.y..=max_octree_pos.y {
                for oz in min_octree_pos.z..=max_octree_pos.z {
                    let Some(chunk) = sable_data.octree_chunks.get(&pack_section_pos(ox, oy, oz)) else {
                        continue;
                    };
                    for x in (min_block_pos.x >> log2)..=(max_block_pos.x >> log2) {
                        for y in (min_block_pos.y >> log2)..=(max_block_pos.y >> log2) {
                            for z in (min_block_pos.z >> log2)..=(max_block_pos.z >> log2) {
                                if chunk.octree.query(
                                    (x << log2) - (ox << OCTREE_CHUNK_SHIFT),
                                    (y << log2) - (oy << OCTREE_CHUNK_SHIFT),
                                    (z << log2) - (oz << OCTREE_CHUNK_SHIFT),
                                    log2,
                                ) > -2
                                {
                                    if cancel_early {
                                        return (true, None);
                                    }
                                    blocks.push(IVec3::new(x, y, z) << log2);
                                }
                            }
                        }
                    }
                }
            }
        }
        if cancel_early {
            (false, None)
        } else {
            (!blocks.is_empty(), Some(blocks))
        }
    }

    #[test]
    fn clipped_terrain_query_matches_the_unclipped_walk() {
        let sable_data = terrain();
        let mut hits = 0;
        let mut misses = 0;
        for dist in [0.87, 1.8, 3.5, 7.0, 15.0, 31.0, 62.0, 120.0] {
            for x in (-75..75).step_by(11) {
                for y in (-12..12).step_by(5) {
                    for z in (-75..75).step_by(13) {
                        let pos = Vec3::new(x as Real + 0.37, y as Real + 0.61, z as Real + 0.19);
                        for cancel_early in [true, false] {
                            let expected = unclipped(pos, dist, &sable_data, cancel_early);
                            let actual = get_overlapping_nodes(None, DVec3::ZERO, pos, dist, &sable_data, cancel_early, false);
                            assert_eq!(actual, expected, "pos {pos:?} dist {dist} cancel_early {cancel_early}");
                            if expected.0 {
                                hits += 1;
                            } else {
                                misses += 1;
                            }
                        }
                    }
                }
            }
        }
        // The fixture must exercise both outcomes.
        assert!(hits > 0 && misses > 0, "hits {hits} misses {misses}");
    }
}
