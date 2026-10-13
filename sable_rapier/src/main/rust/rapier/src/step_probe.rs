//! Opt-in, observation-only dump of a scene's first jointed physics steps.
//!
//! Enabled only when the environment variable `SABLE_STEP_PROBE_FILE` names an output file
//! (JSON lines, appended). `SABLE_STEP_PROBE_STEPS` (default 6) bounds how many steps are
//! recorded per scene; recording for a scene starts at its first executed step that has at
//! least one impulse joint. For each recorded step two records are written: `pre` (after
//! rebase and the joint frame refresh, immediately before `PhysicsPipeline::step`) and `post`
//! (immediately after it). Nothing here mutates simulation state.

use crate::collider::LevelCollider;
use crate::scene::{ChunkAccess, LevelColliderID, PhysicsScene, SableSceneData, SimulationSceneData};
use marten::level::{CHUNK_SHIFT, VoxelPhysicsState};
use rapier3d::dynamics::RigidBodyHandle;
use rapier3d::glamx::{DQuat, DVec3};
use std::collections::HashMap;
use std::fmt::Write as _;
use std::fs::{File, OpenOptions};
use std::io::Write as _;
use std::sync::{Mutex, OnceLock};

struct ProbeOutput {
    file: Mutex<File>,
    steps: u32,
}

/// Per scene handle: (recorded steps, executed steps).
static SCENE_COUNTERS: OnceLock<Mutex<HashMap<i64, (u32, u64)>>> = OnceLock::new();
static OUTPUT: OnceLock<Option<ProbeOutput>> = OnceLock::new();

fn output() -> Option<&'static ProbeOutput> {
    OUTPUT
        .get_or_init(|| {
            let path = std::env::var("SABLE_STEP_PROBE_FILE").ok()?;
            let steps = std::env::var("SABLE_STEP_PROBE_STEPS")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(6);
            let file = match OpenOptions::new().create(true).append(true).open(&path) {
                Ok(file) => file,
                Err(err) => {
                    log::warn!("Sable step probe disabled: cannot open {}: {}", path, err);
                    return None;
                }
            };
            log::info!("Sable step probe enabled: {} ({} steps per scene)", path, steps);
            Some(ProbeOutput {
                file: Mutex::new(file),
                steps,
            })
        })
        .as_ref()
}

/// Forgets a disposed scene so a later scene reusing its handle is recorded afresh.
pub fn forget(scene_handle: i64) {
    if OUTPUT.get().map_or(true, |out| out.is_none()) {
        return;
    }
    if let Some(counters) = SCENE_COUNTERS.get() {
        if let Ok(mut counters) = counters.lock() {
            counters.remove(&scene_handle);
        }
    }
}

/// Returns `(recorded_index, executed_step)` when this step of the scene must be recorded.
/// Call once per executed step, right before the pipeline step.
pub fn begin(scene_handle: i64, sim: &SimulationSceneData) -> Option<(u32, u64)> {
    let out = output()?;
    let mut counters = SCENE_COUNTERS
        .get_or_init(|| Mutex::new(HashMap::new()))
        .lock()
        .ok()?;
    let entry = counters.entry(scene_handle).or_insert((0, 0));
    entry.1 += 1;
    if entry.0 >= out.steps || sim.impulse_joint_set.len() == 0 {
        return None;
    }
    entry.0 += 1;
    Some(*entry)
}

fn num(v: f64) -> String {
    if v.is_finite() {
        format!("{:.9}", v)
    } else {
        format!("\"{}\"", v)
    }
}

fn v3(v: DVec3) -> String {
    format!("[{},{},{}]", num(v.x), num(v.y), num(v.z))
}

fn q4(q: DQuat) -> String {
    format!("[{},{},{},{}]", num(q.x), num(q.y), num(q.z), num(q.w))
}

fn id_json(id: Option<LevelColliderID>) -> String {
    id.map_or("null".to_string(), |i| i.to_string())
}

struct BodyNames {
    by_handle: HashMap<RigidBodyHandle, LevelColliderID>,
    ground: Option<RigidBodyHandle>,
}

impl BodyNames {
    fn new(scene: &PhysicsScene, sable: &SableSceneData) -> Self {
        Self {
            by_handle: sable.rigid_bodies.iter().map(|(id, h)| (*h, *id)).collect(),
            ground: scene.ground_handle,
        }
    }

    fn name(&self, handle: Option<RigidBodyHandle>) -> String {
        match handle {
            None => "\"none\"".to_string(),
            Some(h) if Some(h) == self.ground => "\"ground\"".to_string(),
            Some(h) => self
                .by_handle
                .get(&h)
                .map_or_else(|| format!("\"unmapped:{:?}\"", h.into_raw_parts()), |id| id.to_string()),
        }
    }
}

/// Writes one record. `phase` is `pre` or `post`.
pub fn record(
    scene_handle: i64,
    index: (u32, u64),
    phase: &str,
    dt: f64,
    scene: &PhysicsScene,
    sim: &SimulationSceneData,
) {
    let Some(out) = output() else {
        return;
    };
    let Ok(sable) = scene.sable_data.read() else {
        return;
    };
    let origin = sable.origin;
    let names = BodyNames::new(scene, &sable);
    let mut s = String::with_capacity(64 * 1024);
    let _ = write!(
        s,
        "{{\"scene\":{},\"recorded\":{},\"executed_step\":{},\"phase\":\"{}\",\"dt\":{},\"origin\":{},\"gravity\":{}",
        scene_handle,
        index.0,
        index.1,
        phase,
        num(dt),
        v3(origin),
        v3(DVec3::new(scene.gravity.x as f64, scene.gravity.y as f64, scene.gravity.z as f64)),
    );

    // Bodies registered with Sable (sub-levels and boxes).
    s.push_str(",\"bodies\":[");
    let mut first = true;
    let mut ids: Vec<_> = sable.rigid_bodies.iter().collect();
    ids.sort_by_key(|(id, _)| **id);
    for (id, handle) in ids {
        let Some(rb) = sim.rigid_body_set.get(*handle) else {
            continue;
        };
        let info = sable.level_colliders.get(id);
        if !first {
            s.push(',');
        }
        first = false;
        let _ = write!(
            s,
            "{{\"id\":{},\"kind\":\"{}\",\"pos\":{},\"rot\":{},\"linvel\":{},\"angvel\":{},\"mass\":{},\"native_com\":{},\"rapier_local_com\":{},\"dynamic\":{},\"sleeping\":{},\"colliders\":{}}}",
            id,
            if info.is_some() { "sublevel" } else { "box" },
            v3(rb.translation() + origin),
            q4(*rb.rotation()),
            v3(rb.linvel()),
            v3(rb.angvel()),
            num(rb.mass()),
            info.and_then(|i| i.center_of_mass).map_or("null".to_string(), v3),
            v3(rb.local_center_of_mass()),
            rb.is_dynamic(),
            rb.is_sleeping(),
            rb.colliders().len(),
        );
    }
    s.push(']');

    // Every Sable joint with its native frames, world-space error and solver state.
    s.push_str(",\"joints\":[");
    let mut first = true;
    let mut views: Vec<_> = sable.joint_set.views().collect();
    views.sort_by_key(|v| v.java_handle);
    for view in views {
        let Some(joint) = sim.impulse_joint_set.get(view.rapier_handle) else {
            continue;
        };
        let (Some(rb1), Some(rb2)) = (
            sim.rigid_body_set.get(joint.body1),
            sim.rigid_body_set.get(joint.body2),
        ) else {
            continue;
        };
        let data = &joint.data;
        let f1 = data.local_frame1;
        let f2 = data.local_frame2;
        let w1_rot = *rb1.rotation() * f1.rotation;
        let w2_rot = *rb2.rotation() * f2.rotation;
        let w1_pos = rb1.translation() + *rb1.rotation() * f1.translation;
        let w2_pos = rb2.translation() + *rb2.rotation() * f2.translation;
        let err_world = w2_pos - w1_pos;
        let err_frame1 = w1_rot.inverse() * err_world;
        let rel_rot = w1_rot.inverse() * w2_rot;
        if !first {
            s.push(',');
        }
        first = false;
        let _ = write!(
            s,
            "{{\"java_handle\":{},\"id_a\":{},\"id_b\":{},\"body1\":{},\"body2\":{},\"java_pos_a\":{},\"java_pos_b\":{},\"contacts_enabled\":{},\"native_contacts_enabled\":{},\"locked\":{},\"limit_axes\":{},\"motor_axes\":{},\"frame1\":{{\"pos\":{},\"rot\":{}}},\"frame2\":{{\"pos\":{},\"rot\":{}}},\"anchor1_world\":{},\"anchor2_world\":{},\"err_frame1\":{},\"rel_rot\":{},\"impulses\":[{}]",
            view.java_handle,
            id_json(view.id_a),
            id_json(view.id_b),
            names.name(Some(joint.body1)),
            names.name(Some(joint.body2)),
            v3(view.pos_a),
            v3(view.pos_b),
            view.contacts_enabled,
            data.contacts_enabled,
            data.locked_axes.bits(),
            data.limit_axes.bits(),
            data.motor_axes.bits(),
            v3(f1.translation),
            q4(f1.rotation),
            v3(f2.translation),
            q4(f2.rotation),
            v3(w1_pos + origin),
            v3(w2_pos + origin),
            v3(err_frame1),
            q4(rel_rot),
            joint.impulses.iter().map(|v| num(*v)).collect::<Vec<_>>().join(","),
        );
        s.push_str(",\"limits\":{");
        let mut first_axis = true;
        for axis in 0..6 {
            if data.limit_axes.bits() & (1 << axis) == 0 {
                continue;
            }
            let l = &data.limits[axis];
            if !first_axis {
                s.push(',');
            }
            first_axis = false;
            let _ = write!(s, "\"{}\":[{},{},{}]", axis, num(l.min), num(l.max), num(l.impulse));
        }
        s.push_str("},\"motors\":{");
        let mut first_axis = true;
        for axis in 0..6 {
            if data.motor_axes.bits() & (1 << axis) == 0 {
                continue;
            }
            let m = &data.motors[axis];
            if !first_axis {
                s.push(',');
            }
            first_axis = false;
            let _ = write!(
                s,
                "\"{}\":[{},{},{},{},{},{}]",
                axis,
                num(m.target_pos),
                num(m.target_vel),
                num(m.stiffness),
                num(m.damping),
                num(m.max_force),
                num(m.impulse)
            );
        }
        s.push_str("}}");
    }
    s.push(']');

    // Contact pairs with at least one active contact, aggregated per collider pair.
    s.push_str(",\"contacts\":[");
    let mut first = true;
    for pair in sim.narrow_phase.contact_pairs() {
        if !pair.has_any_active_contact() {
            continue;
        }
        let (Some(c1), Some(c2)) = (
            sim.collider_set.get(pair.collider1),
            sim.collider_set.get(pair.collider2),
        ) else {
            continue;
        };
        let kind = |c: &rapier3d::geometry::Collider| match c.shape().as_shape::<LevelCollider>() {
            Some(level) if level.is_static => "level_static",
            Some(_) => "level",
            None => "shape",
        };
        let mut points = 0usize;
        let mut solver_contacts = 0usize;
        let mut min_dist = f64::INFINITY;
        let mut total_impulse = 0.0f64;
        let mut max_impulse = 0.0f64;
        let mut deepest_world = DVec3::ZERO;
        let mut deepest_normal = DVec3::ZERO;
        let mut strongest_world = DVec3::ZERO;
        let mut strongest_normal = DVec3::ZERO;
        for manifold in pair.manifolds.iter() {
            solver_contacts += manifold.data.solver_contacts.len();
            // Manifold points/normals are in collider1's (sub-shape) frame, as in Rapier's own
            // solver-contact generation: collider pose, then the optional sub-shape pose.
            let frame1 = match manifold.subshape_pos1 {
                Some(sub) => *c1.position() * sub,
                None => *c1.position(),
            };
            let to_world = |p: DVec3| frame1.transform_point(p) + origin;
            let normal_world = frame1.rotation * manifold.local_n1;
            for point in manifold.points.iter() {
                points += 1;
                total_impulse += point.data.impulse;
                if point.dist < min_dist {
                    min_dist = point.dist;
                    deepest_world = to_world(point.local_p1);
                    deepest_normal = normal_world;
                }
                if point.data.impulse > max_impulse {
                    max_impulse = point.data.impulse;
                    strongest_world = to_world(point.local_p1);
                    strongest_normal = normal_world;
                }
            }
        }
        if !first {
            s.push(',');
        }
        first = false;
        let _ = write!(
            s,
            "{{\"body1\":{},\"body2\":{},\"kind1\":\"{}\",\"kind2\":\"{}\",\"manifolds\":{},\"points\":{},\"solver_contacts\":{},\"min_dist\":{},\"total_impulse\":{},\"max_impulse\":{},\"deepest_world\":{},\"deepest_normal\":{},\"strongest_world\":{},\"strongest_normal\":{}}}",
            names.name(c1.parent()),
            names.name(c2.parent()),
            kind(c1),
            kind(c2),
            pair.manifolds.len(),
            points,
            solver_contacts,
            num(min_dist),
            num(total_impulse),
            num(max_impulse),
            v3(deepest_world),
            v3(deepest_normal),
            v3(strongest_world),
            v3(strongest_normal),
        );
    }
    s.push_str("]}\n");

    if phase == "pre" && index.0 == 1 {
        write_sublevel_voxels(&mut s, scene_handle, &sable);
    }

    if let Ok(mut file) = out.file.lock() {
        let _ = file.write_all(s.as_bytes());
        let _ = file.flush();
    }
}

/// One record per sub-level: local bounds and every non-empty voxel cell its collider is built
/// from (plot coordinates, block id, physics state).
fn write_sublevel_voxels(s: &mut String, scene_handle: i64, sable: &SableSceneData) {
    let mut ids: Vec<_> = sable.level_colliders.keys().copied().collect();
    ids.sort();
    for id in ids {
        let info = &sable.level_colliders[&id];
        let (Some(min), Some(max)) = (info.local_bounds_min, info.local_bounds_max) else {
            continue;
        };
        let own = info.has_own_chunks();
        let _ = write!(
            s,
            "{{\"scene\":{},\"sublevel_voxels\":{},\"own_chunks\":{},\"bounds_min\":[{},{},{}],\"bounds_max\":[{},{},{}],\"cells\":[",
            scene_handle, id, own, min.x, min.y, min.z, max.x, max.y, max.z
        );
        let mut first = true;
        let mut missing_sections = 0u32;
        let shift = CHUNK_SHIFT as i32;
        for cx in (min.x >> shift)..=(max.x >> shift) {
            for cy in (min.y >> shift)..=(max.y >> shift) {
                for cz in (min.z >> shift)..=(max.z >> shift) {
                    let chunk = if own {
                        info.get_chunk(cx, cy, cz)
                    } else {
                        sable.get_chunk(cx, cy, cz)
                    };
                    let Some(chunk) = chunk else {
                        missing_sections += 1;
                        continue;
                    };
                    for y in 0..16 {
                        for z in 0..16 {
                            for x in 0..16 {
                                let (wx, wy, wz) = ((cx << shift) + x, (cy << shift) + y, (cz << shift) + z);
                                if wx < min.x || wy < min.y || wz < min.z || wx > max.x || wy > max.y || wz > max.z {
                                    continue;
                                }
                                let (block, state) = chunk.get_block(x, y, z);
                                if state == VoxelPhysicsState::Empty {
                                    continue;
                                }
                                if !first {
                                    s.push(',');
                                }
                                first = false;
                                let _ = write!(s, "[{},{},{},{},\"{:?}\"]", wx, wy, wz, block, state);
                            }
                        }
                    }
                }
            }
        }
        let _ = write!(s, "],\"missing_sections\":{}}}\n", missing_sections);
    }
}
