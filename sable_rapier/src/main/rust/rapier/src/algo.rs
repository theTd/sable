use std::cmp::min;

use marten::Real;
use marten::level::OCTREE_CHUNK_SHIFT;
use rapier3d::glamx::{DVec3, IVec3};
use rapier3d::math::{Pose3, Vec3};
use rapier3d::na::SimdComplexField;

use crate::ActiveLevelColliderInfo;
use crate::scene::{SableSceneData, pack_section_pos};

/// Detects the collision pairs of a sable body.
///
/// Walks the body's octree breadth-first on the calling thread. It used to hand levels of 256 or
/// more nodes to rayon, but the work per node is tiny (a few octree reads) while the hand-off
/// parks the server thread until normal-priority rayon workers get CPU time. With world
/// generation saturating the cores that cost a scheduler quantum or more per call (measured
/// 2026-10-14 for a train car on rails with 16 busy threads: median 9 ms, p99 176 ms, max 323 ms,
/// against 0.3 ms / 0.9 ms / 1.5 ms serial). Pairs come out in the same order as before: level by
/// level, node by node.
pub fn find_collision_pairs(
    sable_body: &ActiveLevelColliderInfo,
    other_sable_body: Option<&ActiveLevelColliderInfo>,
    isometry: &Pose3,
    prediction: Real,
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

    let com_offset: DVec3 = if let Some(other_handle) = other_sable_body {
        let com = other_handle.center_of_mass.unwrap();
        DVec3::new(com.x, com.y, com.z)
    } else {
        DVec3::ZERO
    };

    // start with the root node; the two level buffers and the cell scratch are reused throughout
    let mut current_level = Vec::with_capacity(128);
    let mut next_level = Vec::with_capacity(128);
    let mut cells = Vec::with_capacity(16);

    current_level.push(StackObject {
        index: 0,
        depth: 0,
        min: IVec3::ZERO,
    });

    let mut pairs = Vec::with_capacity(16);
    while !current_level.is_empty() {
        crate::step_stats::PAIR_SEARCH_NODES.count(current_level.len() as u64);
        for entry in &current_level {
            let node = *unsafe { octree.buffer.get_unchecked(entry.index as usize) };
            if node == 0 {
                // Empty node: neither a leaf (pairs) nor a branch (children), so the overlap
                // query below would be thrown away. Most children of a sparse body are empty.
                continue;
            }
            let node_size = 1 << (octree.log_size as u32 - entry.depth);

            // Calculate the center and radius for this node
            let node_center = entry.min.as_dvec3() + node_size as Real / 2.0;
            let transformed_center = isometry.rotation.mul_vec3(node_center) + translation;
            let radius = node_size as Real / 2.0 * 1.7321 + prediction;

            if node > 0 {
                // branch: descend if anything overlaps (first hit is enough)
                if !get_overlapping_nodes(
                    other_sable_body,
                    com_offset,
                    transformed_center,
                    radius,
                    sable_data,
                    None,
                    liquid,
                ) {
                    continue;
                }
                for i in 0..8 {
                    next_level.push(StackObject {
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
            } else {
                // leaf node - add collision pairs
                cells.clear();
                if get_overlapping_nodes(
                    other_sable_body,
                    com_offset,
                    transformed_center,
                    radius,
                    sable_data,
                    Some(&mut cells),
                    liquid,
                ) {
                    let dynamic_block = entry.min + local_bounds_min;
                    pairs.extend(cells.iter().map(|static_block| (*static_block, dynamic_block)));
                }
            }
        }

        std::mem::swap(&mut current_level, &mut next_level);
        next_level.clear();
    }

    pairs
}

/// Whether any block of the other body (or of the terrain when `other_handle` is `None`) lies in
/// the cells overlapping the sphere at `pos` with radius `dist`. With `blocks` it appends every
/// such cell to it; without, it stops at the first one.
fn get_overlapping_nodes(
    other_handle: Option<&ActiveLevelColliderInfo>,
    com_offset: DVec3,
    pos: Vec3,
    dist: Real,
    sable_data: &SableSceneData,
    mut blocks: Option<&mut Vec<IVec3>>,
    liquid: bool,
) -> bool {
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

        let mut found = false;
        for x in min_pos.x..=max_pos.x {
            for y in min_pos.y..=max_pos.y {
                for z in min_pos.z..=max_pos.z {
                    if oct.query(x << log2, y << log2, z << log2, log2) > -2 {
                        let Some(blocks) = blocks.as_deref_mut() else {
                            return true;
                        };
                        found = true;
                        blocks.push(IVec3::new(
                            (x << log2) + other_min.x,
                            (y << log2) + other_min.y,
                            (z << log2) + other_min.z,
                        ));
                    }
                }
            }
        }

        return found;
    }

    // find all the octrees
    let min_octree_pos = min_block_pos >> OCTREE_CHUNK_SHIFT;
    let max_octree_pos = max_block_pos >> OCTREE_CHUNK_SHIFT;

    let mut found = false;
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
                                let Some(blocks) = blocks.as_deref_mut() else {
                                    return true;
                                };
                                found = true;
                                blocks.push(IVec3::new(x, y, z) << log2);
                            }
                        }
                    }
                }
            }
        }
    }

    found
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
                    let solid = (x * 7 + y * 13 + z * 3).rem_euclid(11) == 0;
                    // A different sparse pattern for liquid cells, so a mix-up of the two shows.
                    let liquid = (x * 5 + y * 3 + z * 11).rem_euclid(13) == 0;
                    if !solid && !liquid {
                        continue;
                    }
                    let key = pack_section_pos(
                        x >> OCTREE_CHUNK_SHIFT,
                        y >> OCTREE_CHUNK_SHIFT,
                        z >> OCTREE_CHUNK_SHIFT,
                    );
                    let mask = (1 << OCTREE_CHUNK_SHIFT) - 1;
                    let section = octree_chunks.entry(key).or_insert_with(OctreeChunkSection::new);
                    if solid {
                        section.octree.insert(x & mask, y & mask, z & mask, 1);
                    }
                    if liquid {
                        section.liquid_octree.insert(x & mask, y & mask, z & mask, 1);
                    }
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
    fn unclipped(
        pos: Vec3,
        dist: Real,
        sable_data: &SableSceneData,
        cancel_early: bool,
        liquid: bool,
    ) -> (bool, Option<Vec<IVec3>>) {
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
                    let octree = if liquid { &chunk.liquid_octree } else { &chunk.octree };
                    for x in (min_block_pos.x >> log2)..=(max_block_pos.x >> log2) {
                        for y in (min_block_pos.y >> log2)..=(max_block_pos.y >> log2) {
                            for z in (min_block_pos.z >> log2)..=(max_block_pos.z >> log2) {
                                if octree.query(
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
                        for (cancel_early, liquid) in [(true, false), (false, false), (true, true), (false, true)] {
                            let expected = unclipped(pos, dist, &sable_data, cancel_early, liquid);
                            let actual = if cancel_early {
                                let any = get_overlapping_nodes(None, DVec3::ZERO, pos, dist, &sable_data, None, liquid);
                                (any, None)
                            } else {
                                let mut blocks = Vec::new();
                                let any = get_overlapping_nodes(
                                    None,
                                    DVec3::ZERO,
                                    pos,
                                    dist,
                                    &sable_data,
                                    Some(&mut blocks),
                                    liquid,
                                );
                                (any, Some(blocks))
                            };
                            assert_eq!(
                                actual, expected,
                                "pos {pos:?} dist {dist} cancel_early {cancel_early} liquid {liquid}"
                            );
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

    /// A 39 x 10 x 7 car shell with bogie blocks under it, placed at `bounds_min`.
    fn car(bounds_min: IVec3) -> ActiveLevelColliderInfo {
        let mut octree = marten::octree::SubLevelOctree::new(6);
        for x in 0..39 {
            for z in 0..7 {
                for y in 1..10 {
                    if y == 1 || y == 9 || x == 0 || x == 38 || z == 0 || z == 6 {
                        octree.insert(x, y, z, 1);
                    }
                }
            }
        }
        for x in [4, 5, 33, 34] {
            for z in 2..5 {
                octree.insert(x, 0, z, 1);
            }
        }
        ActiveLevelColliderInfo {
            static_mount: None,
            collider: rapier3d::prelude::ColliderHandle::default(),
            local_bounds_min: Some(bounds_min),
            local_bounds_max: Some(bounds_min + IVec3::new(39, 10, 7)),
            center_of_mass: Some(bounds_min.as_dvec3() + DVec3::new(19.5, 3.0, 3.5)),
            octree: Some(octree),
            chunk_map: None,
            fake_velocities: None,
        }
    }

    /// The pair search as it was before it went serial (its serial branch, which the parallel
    /// one matched pair for pair): terrain cells through the pre-clipping walk, other bodies
    /// through the original loop.
    fn reference_pairs(
        sable_body: &ActiveLevelColliderInfo,
        other: Option<&ActiveLevelColliderInfo>,
        isometry: &Pose3,
        prediction: Real,
        liquid: bool,
        sable_data: &SableSceneData,
    ) -> Vec<(IVec3, IVec3)> {
        let octree = sable_body.octree.as_ref().unwrap();
        let local_bounds_min = sable_body.local_bounds_min.unwrap();
        let com = sable_body.center_of_mass.unwrap();
        let offset = DVec3::new(
            local_bounds_min.x as f64 - com.x,
            local_bounds_min.y as f64 - com.y,
            local_bounds_min.z as f64 - com.z,
        );
        let translation = isometry.translation + isometry.rotation.mul_vec3(offset);
        let com_offset = other.map_or(DVec3::ZERO, |o| o.center_of_mass.unwrap());

        let overlap = |pos: Vec3, dist: Real, cancel_early: bool| -> (bool, Option<Vec<IVec3>>) {
            let Some(other) = other else {
                return unclipped(pos + com_offset, dist, sable_data, cancel_early, liquid);
            };
            let oct = other.octree.as_ref().unwrap();
            let log2 = min(((dist * 2.0).simd_ln() / 2.0f64.simd_ln()).floor() as i32, oct.log_size);
            let min_block_pos = (pos - dist + com_offset).floor().as_ivec3();
            let max_block_pos = (pos + dist + com_offset).floor().as_ivec3();
            let other_min = other.local_bounds_min.unwrap();
            let min_pos = ((min_block_pos - other_min) >> log2).map(|x| x.max(0));
            let max_pos = (max_block_pos - other_min) >> log2;
            let mut blocks = Vec::new();
            for x in min_pos.x..=max_pos.x {
                for y in min_pos.y..=max_pos.y {
                    for z in min_pos.z..=max_pos.z {
                        if oct.query(x << log2, y << log2, z << log2, log2) > -2 {
                            if cancel_early {
                                return (true, None);
                            }
                            blocks.push(IVec3::new(
                                (x << log2) + other_min.x,
                                (y << log2) + other_min.y,
                                (z << log2) + other_min.z,
                            ));
                        }
                    }
                }
            }
            if cancel_early { (false, None) } else { (!blocks.is_empty(), Some(blocks)) }
        };

        let mut current_level = vec![(0u32, 0u32, IVec3::ZERO)];
        let mut pairs = Vec::new();
        while !current_level.is_empty() {
            let mut next_level = Vec::new();
            for &(index, depth, node_min) in &current_level {
                let node = octree.buffer[index as usize];
                if node == 0 {
                    continue;
                }
                let node_size = 1 << (octree.log_size as u32 - depth);
                let node_center = node_min.as_dvec3() + node_size as Real / 2.0;
                let center = isometry.rotation.mul_vec3(node_center) + translation;
                let radius = node_size as Real / 2.0 * 1.7321 + prediction;
                let (any, blocks) = overlap(center, radius, node >= 0);
                if !any {
                    continue;
                }
                if node < 0 {
                    for block in blocks.unwrap() {
                        pairs.push((block, node_min + local_bounds_min));
                    }
                } else {
                    for i in 0..8 {
                        next_level.push((
                            (node + i) as u32,
                            depth + 1,
                            node_min
                                + IVec3::new(
                                    (i & 1) * node_size / 2,
                                    ((i >> 1) & 1) * node_size / 2,
                                    ((i >> 2) & 1) * node_size / 2,
                                ),
                        ));
                    }
                }
            }
            current_level = next_level;
        }
        pairs
    }

    fn poses() -> Vec<Pose3> {
        let mut poses = Vec::new();
        for (x, y, z) in [(0.3, 3.2, 0.4), (-20.6, -1.7, 12.9), (31.1, 6.4, -40.2), (5.5, 9.9, 66.7)] {
            for axisangle in [
                DVec3::ZERO,
                DVec3::new(0.0, 0.7, 0.0),
                DVec3::new(0.2, -1.3, 0.05),
                DVec3::new(-0.9, 0.4, 2.1),
            ] {
                poses.push(Pose3::new(DVec3::new(x, y, z), axisangle));
            }
        }
        poses
    }

    #[test]
    fn serial_terrain_pairs_match_the_former_search_in_order() {
        let sable_data = terrain();
        let body = car(IVec3::new(100, 0, 100));
        let mut total = 0;
        for pose in poses() {
            for prediction in [0.0, 0.01, 0.4] {
                let expected = reference_pairs(&body, None, &pose, prediction, false, &sable_data);
                let actual = find_collision_pairs(&body, None, &pose, prediction, false, &sable_data);
                assert_eq!(actual, expected, "pose {pose:?} prediction {prediction}");
                total += expected.len();
            }
        }
        assert!(total > 0, "the fixture must produce pairs");
    }

    #[test]
    fn serial_body_pairs_match_the_former_search_in_order() {
        let sable_data = terrain();
        let body = car(IVec3::new(0, 0, 0));
        let other = car(IVec3::new(200, 40, -30));
        let other_com = other.center_of_mass.unwrap();
        let mut total = 0;
        for pose in poses() {
            // Body poses relative to the other body's centre of mass, overlapping it.
            let pose = Pose3::from_parts(pose.translation * 0.15, pose.rotation);
            for prediction in [0.0, 0.01, 0.4] {
                let expected = reference_pairs(&body, Some(&other), &pose, prediction, false, &sable_data);
                let actual = find_collision_pairs(&body, Some(&other), &pose, prediction, false, &sable_data);
                assert_eq!(actual, expected, "pose {pose:?} prediction {prediction} (other com {other_com:?})");
                total += expected.len();
            }
        }
        assert!(total > 0, "the fixture must produce pairs");
    }

    #[test]
    fn serial_liquid_pairs_match_the_former_search_off_origin() {
        // Buoyancy's liquid search against a floating origin away from zero.
        let mut sable_data = terrain();
        sable_data.origin = DVec3::new(3.0, -2.0, 5.0);
        let body = car(IVec3::new(100, 0, 100));
        let mut total = 0;
        let mut differs_from_solid = false;
        for pose in poses() {
            let expected = reference_pairs(&body, None, &pose, 0.0, true, &sable_data);
            let actual = find_collision_pairs(&body, None, &pose, 0.0, true, &sable_data);
            assert_eq!(actual, expected, "pose {pose:?}");
            differs_from_solid |= actual != find_collision_pairs(&body, None, &pose, 0.0, false, &sable_data);
            total += expected.len();
        }
        assert!(total > 0, "the fixture must produce liquid pairs");
        assert!(differs_from_solid, "the liquid search must not read the solid cells");
    }
}
