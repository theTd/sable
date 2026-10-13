//! Cost accounting for physics steps: a breakdown line for every slow step, and a summary of all
//! steps about once a minute.
//!
//! The Java side sees `Rapier3D.step` as one opaque native call (34 ms on average in the slow
//! ticks of the 2026-10-12 recording). A slow step's line splits it into Sable's own pre-step work
//! (rebase, rope/joint refresh), Rapier's stages (from its built-in counters) and Sable's voxel
//! collision callbacks, plus the scene sizes that drive them. The summary shows what an ordinary
//! step costs, which the slow lines alone cannot. The narrow phase and the pair search are serial,
//! so every number is wall time on the stepping thread; the counters are relaxed atomics only
//! because they are statics.

use log::info;
use rapier3d::counters::Counters;
use std::sync::Mutex;
use std::sync::atomic::{AtomicU64, Ordering::Relaxed};
use std::time::Instant;

/// A step at least this long gets a breakdown line in the log.
pub const SLOW_STEP_MILLIS: f64 = 25.0;

/// How often the all-steps summary is logged; a window in which no step ran is not logged.
const SUMMARY_SECONDS: f64 = 60.0;

/// Upper bounds of the summary's step-time buckets [ms]; the last bucket is open.
const BUCKET_BOUNDS_MILLIS: [f64; 8] = [0.5, 1.0, 2.0, 5.0, 10.0, 25.0, 50.0, 100.0];

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
    let _ = take_callbacks();
}

/// The callback counters of one step.
pub struct CallbackTotals {
    voxel_voxel_calls: u64,
    voxel_voxel_millis: f64,
    voxel_voxel_pairs: u64,
    pair_search_millis: f64,
    pair_search_nodes: u64,
    pair_search_chunk_lookups: u64,
    pair_search_queries: u64,
    largest_octree_log_size: u64,
    box_tests: u64,
    voxel_shape_calls: u64,
    voxel_shape_millis: f64,
    voxel_shape_cells: u64,
    contact_event_calls: u64,
    contact_event_millis: f64,
}

/// Reads and clears the callback counters; call right after the pipeline step.
pub fn take_callbacks() -> CallbackTotals {
    let (voxel_voxel_calls, voxel_voxel_millis, voxel_voxel_pairs) = VOXEL_VS_VOXEL.take();
    let (_, pair_search_millis, _) = VOXEL_PAIR_SEARCH.take();
    let (_, _, box_tests) = VOXEL_BOX_TESTS.take();
    let (_, _, pair_search_nodes) = PAIR_SEARCH_NODES.take();
    let (_, _, pair_search_chunk_lookups) = PAIR_SEARCH_CHUNK_LOOKUPS.take();
    let (_, _, pair_search_queries) = PAIR_SEARCH_QUERIES.take();
    let (voxel_shape_calls, voxel_shape_millis, voxel_shape_cells) = VOXEL_VS_SHAPE.take();
    let (contact_event_calls, contact_event_millis, _) = CONTACT_EVENT_CALLBACKS.take();
    CallbackTotals {
        voxel_voxel_calls,
        voxel_voxel_millis,
        voxel_voxel_pairs,
        pair_search_millis,
        pair_search_nodes,
        pair_search_chunk_lookups,
        pair_search_queries,
        largest_octree_log_size: LARGEST_OCTREE_LOG_SIZE.swap(0, Relaxed),
        box_tests,
        voxel_shape_calls,
        voxel_shape_millis,
        voxel_shape_cells,
        contact_event_calls,
        contact_event_millis,
    }
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
pub fn report_slow_step(
    marks: &StepMarks,
    counters: &Counters,
    callbacks: &CallbackTotals,
    sizes: impl FnOnce() -> SceneSizes,
) {
    let sizes = sizes();
    info!(
        "slow physics step {:.1} ms: rebase {:.1}, rope+joints {:.1}, pipeline {:.1} \
         [user_changes {:.1}, collision {:.1} (broad {:.1}, narrow {:.1}, final_broad {:.1}), \
         islands {:.1}, update {:.1}, solver {:.1} (velocity {:.1}), ccd toi {:.1} x{} substeps]; \
         voxel callbacks: voxel-voxel {} calls {:.1} ms {} pairs (pair search {:.1} ms: {} nodes, {} chunk lookups, <={} cell queries, largest octree 2^{}; {} box tests), \
         voxel-shape {} calls {:.1} ms {} cells, java contact events {} calls {:.1} ms; \
         scene: {} bodies ({} awake dynamic), {} colliders, {} joints, {} contact pairs \
         ({} touching)",
        total_millis(marks),
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
        callbacks.voxel_voxel_calls,
        callbacks.voxel_voxel_millis,
        callbacks.voxel_voxel_pairs,
        callbacks.pair_search_millis,
        callbacks.pair_search_nodes,
        callbacks.pair_search_chunk_lookups,
        callbacks.pair_search_queries,
        callbacks.largest_octree_log_size,
        callbacks.box_tests,
        callbacks.voxel_shape_calls,
        callbacks.voxel_shape_millis,
        callbacks.voxel_shape_cells,
        callbacks.contact_event_calls,
        callbacks.contact_event_millis,
        sizes.bodies,
        sizes.awake_dynamic,
        sizes.colliders,
        sizes.impulse_joints,
        sizes.contact_pairs,
        sizes.touching_pairs,
    );
}

/// Running totals of every step since the last summary line.
struct Summary {
    window_started: Option<Instant>,
    /// Last step seen (run or skipped); a longer pause than a window closes the window there.
    last_step: Option<Instant>,
    steps: u64,
    skipped: u64,
    total_millis: f64,
    max_millis: f64,
    buckets: [u64; BUCKET_BOUNDS_MILLIS.len() + 1],
    pre_step_millis: f64,
    collision_millis: f64,
    pair_search_millis: f64,
    solver_millis: f64,
    voxel_voxel_calls: u64,
    pairs: u64,
}

impl Summary {
    const fn new() -> Self {
        Self {
            window_started: None,
            last_step: None,
            steps: 0,
            skipped: 0,
            total_millis: 0.0,
            max_millis: 0.0,
            buckets: [0; BUCKET_BOUNDS_MILLIS.len() + 1],
            pre_step_millis: 0.0,
            collision_millis: 0.0,
            pair_search_millis: 0.0,
            solver_millis: 0.0,
            voxel_voxel_calls: 0,
            pairs: 0,
        }
    }
}

static SUMMARY: Mutex<Summary> = Mutex::new(Summary::new());

/// Starts a fresh window if no step was called for longer than a window (world closed and
/// reopened, server paused, no sub-levels left): the closed window is logged up to its last step,
/// so a summary never spans the pause. All-asleep stretches still call the step and roll over in
/// `note_skipped_step`.
fn resume_window(summary: &mut Summary, now: Instant) {
    if let (Some(window_started), Some(last_step)) = (summary.window_started, summary.last_step)
        && now.duration_since(last_step).as_secs_f64() > SUMMARY_SECONDS
    {
        if summary.steps > 0 {
            log_summary(summary, last_step.duration_since(window_started).as_secs_f64());
        }
        *summary = Summary::new();
    }
    summary.window_started.get_or_insert(now);
    summary.last_step = Some(now);
}

/// Counts a step that returned early because nothing could move. While everything stays asleep
/// only these arrive, so the window also rolls over here: logged if it has run steps, otherwise
/// just restarted (nothing ran, nothing to report).
pub fn note_skipped_step() {
    let now = Instant::now();
    let mut summary = SUMMARY.lock().unwrap_or_else(|poisoned| poisoned.into_inner());
    resume_window(&mut summary, now);
    summary.skipped += 1;
    let window_started = summary.window_started.unwrap_or(now);
    let seconds = now.duration_since(window_started).as_secs_f64();
    if seconds < SUMMARY_SECONDS {
        return;
    }
    if summary.steps > 0 {
        log_summary(&summary, seconds);
    }
    *summary = Summary::new();
    summary.window_started = Some(now);
    summary.last_step = Some(now);
}

/// Adds one completed step to the summary and logs the summary once its window is over.
pub fn record_step(marks: &StepMarks, counters: &Counters, callbacks: &CallbackTotals) {
    let total = total_millis(marks);
    let mut summary = SUMMARY.lock().unwrap_or_else(|poisoned| poisoned.into_inner());
    resume_window(&mut summary, marks.started);
    let window_started = summary.window_started.unwrap_or(marks.started);
    summary.steps += 1;
    summary.total_millis += total;
    summary.max_millis = summary.max_millis.max(total);
    let bucket = BUCKET_BOUNDS_MILLIS
        .iter()
        .position(|bound| total <= *bound)
        .unwrap_or(BUCKET_BOUNDS_MILLIS.len());
    summary.buckets[bucket] += 1;
    summary.pre_step_millis += millis(marks.started, marks.constraints_refreshed);
    summary.collision_millis += counters.stages.collision_detection_time.time_ms();
    summary.pair_search_millis += callbacks.pair_search_millis;
    summary.solver_millis += counters.stages.solver_time.time_ms();
    summary.voxel_voxel_calls += callbacks.voxel_voxel_calls;
    summary.pairs += callbacks.voxel_voxel_pairs;

    let seconds = marks.finished.duration_since(window_started).as_secs_f64();
    if seconds < SUMMARY_SECONDS {
        return;
    }
    log_summary(&summary, seconds);
    *summary = Summary::new();
    summary.window_started = Some(marks.finished);
    summary.last_step = Some(marks.finished);
}

fn log_summary(summary: &Summary, seconds: f64) {
    let steps = summary.steps.max(1) as f64;
    let mut buckets = String::new();
    for (index, count) in summary.buckets.iter().enumerate() {
        if !buckets.is_empty() {
            buckets.push_str(", ");
        }
        match BUCKET_BOUNDS_MILLIS.get(index) {
            Some(bound) => buckets.push_str(&format!("<={bound} ms {count}")),
            None => buckets.push_str(&format!(
                ">{} ms {count}",
                BUCKET_BOUNDS_MILLIS[BUCKET_BOUNDS_MILLIS.len() - 1]
            )),
        }
    }
    info!(
        "physics sub-steps over {:.0} s (all dimensions, one per scene and sub-step): {} run, \
         {} skipped as all asleep (not timed); per run sub-step: mean {:.2} ms, max {:.1} ms; {}; \
         mean pre-step {:.2}, collision {:.2} \
         (pair search {:.2} in {:.1} voxel-voxel calls, {:.1} pairs), solver {:.2}",
        seconds,
        summary.steps,
        summary.skipped,
        summary.total_millis / steps,
        summary.max_millis,
        buckets,
        summary.pre_step_millis / steps,
        summary.collision_millis / steps,
        summary.pair_search_millis / steps,
        summary.voxel_voxel_calls as f64 / steps,
        summary.pairs as f64 / steps,
        summary.solver_millis / steps,
    );
}
