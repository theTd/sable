package dev.ryanhcode.sable.util;

import dev.ryanhcode.sable.Sable;
import dev.ryanhcode.sable.companion.math.BoundingBox3d;
import dev.ryanhcode.sable.companion.math.BoundingBox3dc;
import dev.ryanhcode.sable.sublevel.SubLevel;
import net.minecraft.core.BlockPos;
import net.minecraft.world.level.Level;
import net.minecraft.world.level.block.state.BlockState;
import org.joml.Vector3d;

/**
 * Finds the height a sub-level shelters a world column at.
 *
 * <p>Vanilla {@code LevelRenderer} draws precipitation from a column height downwards, and that
 * height knows nothing about sub-levels, so a roof inside a vehicle is invisible to it. This class
 * answers "where does rain stop in this column, considering the sub-levels?". The renderer takes
 * the maximum of this and the vanilla height.
 *
 * <p>Lives in a normal package, not the mixin package: mixin packages may only hold mixin and
 * plugin classes, and game tests must be able to call this directly.
 */
public final class SubLevelRainColumn {

    /** Returned when no sub-level covers the column or none of them has a blocking block. */
    public static final int NONE = Integer.MIN_VALUE;

    /** Defensive cap on the downward walk, in blocks. */
    private static final int MAX_STEPS = 4096;

    /** Set {@code -Dsable.debug.rain.shelter=true} to log columns a sub-level failed to shelter. */
    private static final boolean DEBUG = Boolean.getBoolean("sable.debug.rain.shelter");

    /** Rate-limits the debug log so a whole screen of columns does not flood the console. */
    private static final long LOG_INTERVAL_NANOS = 2_000_000_000L;

    private static long lastLogNanos;

    /**
     * Reports, at most every two seconds and only under the debug flag, why a column was not
     * sheltered. Without this a column that misses the roof looks exactly like a column with no
     * sub-level at all, which is how this feature silently did nothing for so long.
     */
    public static void logMiss(final Level level, final int x, final int z, final int yOffset) {
        if (!DEBUG) {
            return;
        }

        final long now = System.nanoTime();

        if (now - lastLogNanos < LOG_INTERVAL_NANOS) {
            return;
        }

        lastLogNanos = now;

        final StringBuilder reason = new StringBuilder();
        final int height = describe(level, x, z, yOffset, reason);

        Sable.LOGGER.info("[rain-shelter] column ({}, {}) not sheltered: {} (result {})",
                x, z, reason.isEmpty() ? "sheltered" : reason, height);
    }

    /**
     * @param level   the level the sub-levels live in
     * @param x       world block X of the column
     * @param z       world block Z of the column
     * @param yOffset added to the block's Y, so the result matches the caller's own convention
     * @return the world Y of the first blocking block in that column plus {@code yOffset}, or
     *         {@link #NONE}
     */
    public static int height(final Level level, final int x, final int z, final int yOffset) {
        return query(level, x, z, yOffset, null);
    }

    /**
     * Debug variant. Same result, but reports through {@code reason} why it returned
     * {@link #NONE}: no sub-level in the column, or a sub-level with no blocking block.
     *
     * @return the same value {@link #height} would return
     */
    public static int describe(final Level level, final int x, final int z, final int yOffset,
                               final StringBuilder reason) {
        final int[] stats = new int[2];
        final int height = query(level, x, z, yOffset, stats);

        if (stats[0] == 0) {
            reason.append("no intersecting sub-level");
        } else if (height == NONE) {
            reason.append("scanned ").append(stats[0])
                    .append(" sub-level(s), ").append(stats[1]).append(" blocks: none blocking");
        }

        return height;
    }

    private static int query(final Level level, final int x, final int z, final int yOffset,
                             final int[] stats) {
        final BoundingBox3dc column = new BoundingBox3d(
                x, level.getMinBuildHeight(), z,
                x + 1, level.getMaxBuildHeight(), z + 1);

        int found = NONE;

        for (final SubLevel subLevel : Sable.HELPER.getAllIntersecting(level, column)) {
            if (stats != null) {
                stats[0]++;
            }

            final int height = scan(subLevel, level, x, z, yOffset, stats);

            if (height != NONE && (found == NONE || height > found)) {
                found = height;
            }
        }

        return found;
    }

    private static int scan(final SubLevel subLevel, final Level level, final int x, final int z,
                            final int yOffset, final int[] stats) {
        final BoundingBox3dc bounds = subLevel.boundingBox();
        final double span = bounds.maxY() - bounds.minY();

        if (span <= 0.0) {
            return NONE;
        }

        final LevelAccelerator accelerator = new LevelAccelerator(level);
        final BlockPos.MutableBlockPos pos = new BlockPos.MutableBlockPos();
        final Vector3d cursor = new Vector3d();

        // Entry point: the middle of the column at the top face of the world AABB, mapped into the
        // sub-level's own space. Sampling before stepping matters: on an axis-aligned vehicle this
        // lands exactly on the local roof plane, so a single-layer roof must be sampled first or a
        // rounding step drops straight past it.
        subLevel.logicalPose().transformPositionInverse(cursor.set(x + 0.5, bounds.maxY(), z + 0.5));

        final Vector3d localUp = new Vector3d();
        subLevel.logicalPose().transformNormalInverse(localUp.set(0.0, 1.0, 0.0));

        // Walk far enough to cross the whole vertical extent even when the vehicle is tilted, where
        // one local step covers less than one world block of height.
        final double vertical = Math.max(Math.abs(localUp.y), 0.05);
        final int steps = Math.min((int) Math.ceil(span / vertical) + 2, MAX_STEPS);

        for (int step = 0; step < steps; step++) {
            final BlockState state = accelerator.getBlockState(
                    pos.set(cursor.x, cursor.y, cursor.z));

            if (stats != null) {
                stats[1]++;
            }

            if (blocksPrecipitation(state, accelerator, pos)) {
                // Snap to the block we just sampled before transforming. The cursor is a float that
                // sits part-way into the block, so its transformed Y drifts by up to a block and the
                // result must be a block's corner, not a point inside it.
                cursor.set(Math.floor(cursor.x), Math.floor(cursor.y), Math.floor(cursor.z));
                subLevel.logicalPose().transformPosition(cursor);

                // ceil, not (int). A vehicle sits at a fractional world offset, so the corner
                // transforms to e.g. 69.99; +1 = 70.99; (int) truncates to 70, which is the
                // camera's own block and vanilla then treats the column as unsheltered and keeps
                // drawing rain. ceil gives 71, the first air block above the roof, which is what
                // Level.getHeight() means.
                return (int) Math.ceil(cursor.y + yOffset);
            }

            cursor.sub(localUp);
        }

        return NONE;
    }

    /**
     * A block shelters the column when it would physically stop rain. Thin blocks count (their
     * collision shape is not empty); torches, rails and other pass-through props do not.
     */
    private static boolean blocksPrecipitation(final BlockState state,
                                               final LevelAccelerator accelerator,
                                               final BlockPos.MutableBlockPos pos) {
        return state.blocksMotion()
                || !state.getFluidState().isEmpty()
                || !state.getCollisionShape(accelerator, pos).isEmpty();
    }

    private SubLevelRainColumn() {
    }
}