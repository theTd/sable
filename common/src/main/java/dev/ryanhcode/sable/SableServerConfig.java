package dev.ryanhcode.sable;

import net.neoforged.neoforge.common.ModConfigSpec;

public final class SableServerConfig {

    public static final ModConfigSpec SPEC;

    public static final ModConfigSpec.IntValue SUB_LEVEL_SUBSTEPS_PER_TICK;
    public static final ModConfigSpec.BooleanValue SUB_LEVEL_STORAGE_PRUNING;
    public static final ModConfigSpec.IntValue SOLVER_ITERATIONS;
    public static final ModConfigSpec.IntValue PGS_ITERATIONS;
    public static final ModConfigSpec.IntValue STABILIZATION_ITERATIONS;
    public static final ModConfigSpec.BooleanValue PHYSICS_SLEEP_SKIP;
    public static final ModConfigSpec.BooleanValue CONTACT_FORCE_EVENTS;
    public static final ModConfigSpec.BooleanValue BIOME_PROJECTION_FAST_PATH;
    public static final ModConfigSpec.BooleanValue EMPTY_CONTAINER_FAST_PATH;
    public static final ModConfigSpec.IntValue PHYSICS_TICK_INTERVAL;

    static {
        final ModConfigSpec.Builder builder = new ModConfigSpec.Builder();

        SUB_LEVEL_SUBSTEPS_PER_TICK = builder
                .comment("How many times the physics simulation is stepped in every second. Higher values will be significantly more performance intensive, but will have higher accuracy.")
                .defineInRange("sub_level_substeps_per_tick", 2, 1, 10);

        SUB_LEVEL_STORAGE_PRUNING = builder
                .comment("Prunes and deletes empty sub-level storage files. Experimental.")
                .define("sub_level_storage_pruning", false);

        SOLVER_ITERATIONS = builder
                .comment("The number of solver iterations run by the constraints solver for calculating forces.")
                .defineInRange("solver_iterations", 18, 1, 64);

        PGS_ITERATIONS = builder
                .comment("The number of PGS iterations run by the constraints solver.")
                .defineInRange("pgs_iterations", 2, 1, 8);

        STABILIZATION_ITERATIONS = builder
                .comment("The number of stabilization iterations run by the constraints solver.")
                .defineInRange("stabilization_iterations", 2, 1, 8);

        PHYSICS_SLEEP_SKIP = builder
                .comment("Allows the physics engine to skip simulating sleeping bodies.")
                .define("physics_sleep_skip", true);

        CONTACT_FORCE_EVENTS = builder
                .comment("Whether the physics engine reports contact force events, used for collision sounds and particles. Only applies to colliders created or rebuilt after the change.")
                .define("contact_force_events", true);

        BIOME_PROJECTION_FAST_PATH = builder
                .comment("Skips sub-level biome projection lookups in dimensions that have no sub-levels.")
                .define("biome_projection_fast_path", true);

        EMPTY_CONTAINER_FAST_PATH = builder
                .comment("Skips per-tick sub-level container work in dimensions that have no sub-levels and no pending sub-level storage or ticket work.")
                .define("empty_container_fast_path", true);

        PHYSICS_TICK_INTERVAL = builder
                .comment("The tick interval between physics simulation runs. Values above 1 simulate every N ticks with a proportionally larger time step.")
                .defineInRange("physics_tick_interval", 1, 1, 10);

        SPEC = builder.build();
    }
}
