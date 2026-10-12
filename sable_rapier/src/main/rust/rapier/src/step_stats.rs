//! Cost accounting for one physics step, logged only when the step is slow.
//!
//! The Java side sees `Rapier3D.step` as one opaque native call (34 ms on average in the slow
//! ticks of the 2026-10-12 recording). This splits it into Sable's own pre-step work (rebase,
//! rope/joint refresh), Rapier's stages (from its built-in counters) and Sable's voxel collision
//! callbacks, plus the scene sizes that drive them. Counters are relaxed atomics because the
//! narrow phase runs the dispatcher on rayon worker threads; their nanos are therefore summed
//! across threads (CPU time), while Rapier's stage timers are wall time.

use log::info;
use rapier3d::counters::Counters;
use std::sync::atomic::{AtomicU64, Ordering::Relaxed};
use std::time::Instant;

/// A step at least this long gets a breakdown line in the log.
pub const SLOW_STEP_MILLIS: f64 = 25.0;

/// Calls, summed nanos and work items of one kind of callback during the current step.
pub struct StepCounter {
    calls: AtomicU64,
    nanos: AtomicU64,
    items: AtomicU64,
}

impl StepCounter {
    const fn new() -> Self {
        Self {
            calls: AtomicU64::new(0),
            nanos: AtomicU64::new(0),
            items: AtomicU64::new(0),
        }
    }

    pub fn record(&self, started: Instant, items: u64) {
        self.calls.fetch_add(1, Relaxed);
        self.nanos
            .fetch_add(started.elapsed().as_nanos() as u64, Relaxed);
        self.items.fetch_add(items, Relaxed);
    }

    /// Adds work items without timing them.
    pub fn count(&self, items: u64) {
        self.items.fetch_add(items, Relaxed);
    }

    fn take(&self) -> (u64, f64, u64) {
        (
            self.calls.swap(0, Relaxed),
            self.nanos.swap(0, Relaxed) as f64 / 1_000_000.0,
            self.items.swap(0, Relaxed),
        )
    }
}

/// Sub-level vs terrain / sub-level vs sub-level voxel pair search (items = block pairs found).
pub static VOXEL_VS_VOXEL: StepCounter = StepCounter::new();
/// The octree pair search inside voxel-vs-voxel (items = block pairs found).
pub static VOXEL_PAIR_SEARCH: StepCounter = StepCounter::new();
/// Cuboid-vs-cuboid manifold tests inside voxel-vs-voxel (items only, not timed per test).
pub static VOXEL_BOX_TESTS: StepCounter = StepCounter::new();
/// Octree nodes of the moving body visited by the pair search.
pub static PAIR_SEARCH_NODES: StepCounter = StepCounter::new();
/// Terrain octree-chunk hash lookups made by the pair search.
pub static PAIR_SEARCH_CHUNK_LOOKUPS: StepCounter = StepCounter::new();
/// Terrain octree cell queries scheduled by the pair search (upper bound: early exits stop sooner).
pub static PAIR_SEARCH_QUERIES: StepCounter = StepCounter::new();
static LARGEST_OCTREE_LOG_SIZE: AtomicU64 = AtomicU64::new(0);

pub fn note_octree_log_size(log_size: i32) {
    LARGEST_OCTREE_LOG_SIZE.fetch_max(log_size.max(0) as u64, Relaxed);
}

/// Terrain vs a primitive shape, scanning every block cell in its bounds (items = cells scanned).
pub static VOXEL_VS_SHAPE: StepCounter = StepCounter::new();
/// Contact-event callbacks into Java from the contact modification hook (items = calls).
pub static CONTACT_EVENT_CALLBACKS: StepCounter = StepCounter::new();

/// Clears the callback counters; call right before the pipeline step.
pub fn begin_pipeline_step() {
    VOXEL_VS_VOXEL.take();
    VOXEL_PAIR_SEARCH.take();
    VOXEL_BOX_TESTS.take();
    PAIR_SEARCH_NODES.take();
    PAIR_SEARCH_CHUNK_LOOKUPS.take();
    PAIR_SEARCH_QUERIES.take();
    LARGEST_OCTREE_LOG_SIZE.store(0, Relaxed);
    VOXEL_VS_SHAPE.take();
    CONTACT_EVENT_CALLBACKS.take();
}

/// Scene sizes at the end of the step (only gathered for slow steps).
pub struct SceneSizes {
    pub bodies: usize,
    pub awake_dynamic: usize,
    pub colliders: usize,
    pub impulse_joints: usize,
    pub contact_pairs: usize,
    pub touching_pairs: usize,
}

/// Wall-clock marks of one native step call.
pub struct StepMarks {
    pub started: Instant,
    pub rebased: Instant,
    pub constraints_refreshed: Instant,
    pub pipeline_started: Instant,
    pub finished: Instant,
}

fn millis(from: Instant, to: Instant) -> f64 {
    to.duration_since(from).as_secs_f64() * 1000.0
}

pub fn total_millis(marks: &StepMarks) -> f64 {
    millis(marks.started, marks.finished)
}

/// Logs one line describing where a slow step spent its time.
pub fn report_slow_step(marks: &StepMarks, counters: &Counters, sizes: impl FnOnce() -> SceneSizes) {
    let total = total_millis(marks);
    let (vv_calls, vv_ms, vv_pairs) = VOXEL_VS_VOXEL.take();
    let (_, search_ms, _) = VOXEL_PAIR_SEARCH.take();
    let (_, _, box_tests) = VOXEL_BOX_TESTS.take();
    let (_, _, nodes) = PAIR_SEARCH_NODES.take();
    let (_, _, chunk_lookups) = PAIR_SEARCH_CHUNK_LOOKUPS.take();
    let (_, _, queries) = PAIR_SEARCH_QUERIES.take();
    let largest_log_size = LARGEST_OCTREE_LOG_SIZE.swap(0, Relaxed);
    let (vs_calls, vs_ms, vs_cells) = VOXEL_VS_SHAPE.take();
    let (ev_calls, ev_ms, _) = CONTACT_EVENT_CALLBACKS.take();
    let sizes = sizes();
    info!(
        "slow physics step {:.1} ms: rebase {:.1}, rope+joints {:.1}, pipeline {:.1} \
         [user_changes {:.1}, collision {:.1} (broad {:.1}, narrow {:.1}, final_broad {:.1}), \
         islands {:.1}, update {:.1}, solver {:.1} (velocity {:.1}), ccd toi {:.1} x{} substeps]; \
         voxel callbacks (cpu, all threads): voxel-voxel {} calls {:.1} ms {} pairs (pair search {:.1} ms: {} nodes, {} chunk lookups, <={} cell queries, largest octree 2^{}; {} box tests), \
         voxel-shape {} calls {:.1} ms {} cells, java contact events {} calls {:.1} ms; \
         scene: {} bodies ({} awake dynamic), {} colliders, {} joints, {} contact pairs \
         ({} touching)",
        total,
        millis(marks.started, marks.rebased),
        millis(marks.rebased, marks.constraints_refreshed),
        millis(marks.pipeline_started, marks.finished),
        counters.stages.user_changes.time_ms(),
        counters.stages.collision_detection_time.time_ms(),
        counters.cd.broad_phase_time.time_ms(),
        counters.cd.narrow_phase_time.time_ms(),
        counters.cd.final_broad_phase_time.time_ms(),
        counters.stages.island_construction_time.time_ms(),
        counters.stages.update_time.time_ms(),
        counters.stages.solver_time.time_ms(),
        counters.solver.velocity_resolution_time.time_ms(),
        counters.ccd.toi_computation_time.time_ms(),
        counters.ccd.num_substeps,
        vv_calls,
        vv_ms,
        vv_pairs,
        search_ms,
        nodes,
        chunk_lookups,
        queries,
        largest_log_size,
        box_tests,
        vs_calls,
        vs_ms,
        vs_cells,
        ev_calls,
        ev_ms,
        sizes.bodies,
        sizes.awake_dynamic,
        sizes.colliders,
        sizes.impulse_joints,
        sizes.contact_pairs,
        sizes.touching_pairs,
    );
}
