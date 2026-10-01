package dev.ryanhcode.sable.mixinhelpers.biome_projection;

import dev.ryanhcode.sable.Sable;
import dev.ryanhcode.sable.sublevel.SubLevel;
import net.minecraft.core.BlockPos;
import net.minecraft.world.level.Level;
import net.minecraft.world.phys.Vec3;

/**
 * Biome / climate queries on sub-level blocks must be answered for where the sub-level currently is,
 * not for the plot the blocks are stored in (plots sit far from the world, and their chunks only carry
 * the biome the sub-level was created in).
 */
public final class SubLevelBiomeProjectionHelper {

    private SubLevelBiomeProjectionHelper() {
    }

    /**
     * @return {@code pos} projected out of its sub-level into the level's real coordinates,
     * or {@code pos} itself if it is not inside a plot. Idempotent for already-projected positions.
     */
    public static BlockPos projectOutOfPlot(final Level level, final BlockPos pos) {
        final SubLevel subLevel = Sable.HELPER.getContaining(level, pos);
        if (subLevel == null) {
            return pos;
        }

        final Vec3 projected = Sable.HELPER.projectOutOfSubLevel(level, pos.getCenter());
        return BlockPos.containing(projected);
    }
}
