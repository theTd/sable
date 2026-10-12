package dev.ryanhcode.sable.physics.config;

import dev.ryanhcode.sable.SableConfig;
import dev.ryanhcode.sable.SableServerConfig;

public class PhysicsConfigData {
    /**
     * The number of solver iterations run by the constraints solver for calculating forces.
     */
    public int solverIterations = 18;

    public int pgsIterations = 2;
    public int stabilizationIterations = 2;

    /**
     * The damping ratio used by the springs for contact constraint stabilization.
     */
    public double contactSpringDampingRatio = 5.0;

    /**
     * The natural frequency used by the springs for contact constraint regularization.
     */
    public double contactSpringFrequency = 40.0;

    /**
     * Minimum number of dynamic bodies in each active island.
     */
    public int minDynamicBodiesPerIsland = 128;

    /**
     * Physics ticks done per game tick in the physics pipeline.
     */
    public int substepsPerTick = 2;

    /**
     * Whether the physics engine skips simulating sleeping bodies.
     */
    public boolean physicsSleepSkip = true;

    /**
     * Whether the physics engine reports contact force events.
     */
    public boolean contactForceEvents = true;

    /**
     * Game ticks between physics simulation runs. Values above 1 simulate with a proportionally larger time step.
     */
    public int physicsTickInterval = 1;

    public void updateFromConfig() {
        if (!SableServerConfig.SPEC.isLoaded()) {
            return;
        }

        this.substepsPerTick = SableServerConfig.SUB_LEVEL_SUBSTEPS_PER_TICK.getAsInt();
        this.solverIterations = SableServerConfig.SOLVER_ITERATIONS.getAsInt();
        this.pgsIterations = SableServerConfig.PGS_ITERATIONS.getAsInt();
        this.stabilizationIterations = SableServerConfig.STABILIZATION_ITERATIONS.getAsInt();
        this.physicsSleepSkip = SableServerConfig.PHYSICS_SLEEP_SKIP.getAsBoolean();
        this.contactForceEvents = SableServerConfig.CONTACT_FORCE_EVENTS.getAsBoolean();
        this.physicsTickInterval = SableServerConfig.PHYSICS_TICK_INTERVAL.getAsInt();
    }
}
