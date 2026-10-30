package dev.ryanhcode.sable.neoforge.gametest;

import dev.ryanhcode.sable.Sable;
import dev.ryanhcode.sable.api.sublevel.ServerSubLevelContainer;
import dev.ryanhcode.sable.api.sublevel.SubLevelContainer;
import dev.ryanhcode.sable.companion.math.BoundingBox3dc;
import dev.ryanhcode.sable.sublevel.ServerSubLevel;
import dev.ryanhcode.sable.util.SubLevelRainColumn;
import net.minecraft.core.BlockPos;
import net.minecraft.gametest.framework.GameTest;
import net.minecraft.gametest.framework.GameTestHelper;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.world.level.block.Blocks;
import net.neoforged.neoforge.gametest.GameTestHolder;
import org.jetbrains.annotations.Nullable;
import org.joml.Vector3d;

import static dev.ryanhcode.sable.neoforge.gametest.SableTestHelper.absolutePosition;
import static dev.ryanhcode.sable.neoforge.gametest.SableTestHelper.removeSubLevel;

/**
 * Game tests for the precipitation shelter of sub-levels.
 *
 * <p>These are self-calibrating on purpose: the sub-level's own world bounding box is the ground
 * truth for where its roof ended up, so the assertions do not depend on where the test framework
 * happens to place the structure.
 *
 * <p>The regression they cover: the scan returns a block's height by transforming its corner back
 * to world space. A vehicle almost never sits at an integer world offset, so that coordinate is
 * fractional, and truncating it with {@code (int)} reported the very block the camera occupies.
 * Vanilla then treated the column as unsheltered and kept drawing rain inside the vehicle.
 */
@GameTestHolder(Sable.MOD_ID)
public final class RainShelterTest {

    /** How far below the roof top the "eye" is placed, in blocks. */
    private static final double EYE_BELOW_ROOF = 1.5;

    /**
     * Lifts the test sub-level into positive world Y.
     *
     * <p>Java's {@code (int)} cast truncates toward zero, so for a negative Y it already behaves
     * like {@code ceil} and the regression is invisible. The bug this guards only shows on the
     * positive side, which is where real worlds sit.
     */
    private static final double POSITIVE_Y_OFFSET = 200.0;

    /**
     * A roofed column must report the roof's top face, not the block below it.
     *
     * <p>Before the ceil fix this reported exactly {@code floor(roofTop)}, which vanilla reads as
     * "this is the block the camera is in" and keeps drawing rain.
     */
    @GameTest(template = "gravity", batch = "rain_shelter")
    public static void roofedColumnReportsRoofTop(final GameTestHelper helper) {
        final ServerLevel level = helper.getLevel();
        final ServerSubLevelContainer container = SubLevelContainer.getContainer(level);

        if (container == null) {
            throw new IllegalStateException("No plot container in the test level");
        }

        final Vector3d spawn = absolutePosition(helper, new Vector3d(2.5, 4.4, 2.5)).add(0.0, POSITIVE_Y_OFFSET, 0.0);
        final ServerSubLevel subLevel = spawnRoof(container, spawn);

        helper.runAfterDelay(2, () -> {
            final BoundingBox3dc bounds = subLevel.boundingBox();
            final int x = (int) Math.floor((bounds.minX() + bounds.maxX()) / 2.0);
            final int z = (int) Math.floor((bounds.minZ() + bounds.maxZ()) / 2.0);
            final int height = SubLevelRainColumn.height(level, x, z, 1);

            // The roof's top face is the first air block above it. A fractional vehicle offset must
            // still round up to it; truncation lands one whole block lower.
            final int expected = (int) Math.ceil(bounds.maxY());
            final int eye = (int) Math.floor(bounds.maxY() - EYE_BELOW_ROOF);

            final String failure;

            if (height == SubLevelRainColumn.NONE) {
                failure = "A roofed column reported no shelter at all "
                        + "(the roof was stepped over or the read saw air); bounds y=["
                        + bounds.minY() + ", " + bounds.maxY() + "]";
            } else if (height != expected) {
                failure = "Shelter height " + height + " != roof top " + expected
                        + " (bounds y=[" + bounds.minY() + ", " + bounds.maxY()
                        + "], fractional offset truncated?)";
            } else if (height <= eye) {
                failure = "Shelter height " + height
                        + " is not above an eye standing " + EYE_BELOW_ROOF + " blocks under the roof";
            } else {
                failure = null;
            }

            settle(helper, container, subLevel, failure);
        });
    }

    /** A column outside the roof's footprint must stay unsheltered, so open vehicles show rain. */
    @GameTest(template = "gravity", batch = "rain_shelter")
    public static void openColumnStaysUnsheltered(final GameTestHelper helper) {
        final ServerLevel level = helper.getLevel();
        final ServerSubLevelContainer container = SubLevelContainer.getContainer(level);

        if (container == null) {
            throw new IllegalStateException("No plot container in the test level");
        }

        final Vector3d spawn = absolutePosition(helper, new Vector3d(2.5, 4.4, 2.5)).add(0.0, POSITIVE_Y_OFFSET, 0.0);
        final ServerSubLevel subLevel = spawnRoof(container, spawn);

        helper.runAfterDelay(2, () -> {
            final BoundingBox3dc bounds = subLevel.boundingBox();
            final int x = (int) Math.floor((bounds.minX() + bounds.maxX()) / 2.0) + 64;
            final int z = (int) Math.floor((bounds.minZ() + bounds.maxZ()) / 2.0) + 64;
            final int height = SubLevelRainColumn.height(level, x, z, 1);

            final String failure = height != SubLevelRainColumn.NONE
                    ? "An open column reported shelter " + height + ", expected none"
                    : null;

            settle(helper, container, subLevel, failure);
        });
    }

    /**
     * Removes the sub-level, then reports the result.
     *
     * <p>Order matters twice over. {@code succeed} and {@code fail} both tear the test structure
     * down and remove the sub-levels standing in it, so a later removal has nothing left to find.
     * And a sub-level left behind keeps a physics body alive, which crashes the next batch with
     * "Body has been removed" instead of reporting a plain test failure.
     */
    private static void settle(final GameTestHelper helper, final ServerSubLevelContainer container,
                               final ServerSubLevel subLevel, final @Nullable String failure) {
        removeSubLevel(container, subLevel);

        if (failure == null) {
            helper.succeed();
        } else {
            helper.fail(failure);
        }
    }

    /**
     * Spawns a 5x5 roof and refreshes the bounds.
     *
     * <p>The explicit {@code updateBoundingBox} calls are needed because a test never ticks the
     * sub-level, and the scan starts from the world AABB top.
     */
    private static ServerSubLevel spawnRoof(final ServerSubLevelContainer container,
                                            final Vector3d spawn) {
        final ServerSubLevel subLevel = SableTestHelper.spawnSubLevel(container, spawn, plot -> {
            for (int x = -2; x <= 2; x++) {
                for (int z = -2; z <= 2; z++) {
                    plot.setBlock(new BlockPos(x, 2, z), Blocks.STONE.defaultBlockState(), 3);
                }
            }
        });

        subLevel.getPlot().updateBoundingBox();
        subLevel.updateBoundingBox();

        return subLevel;
    }

    private RainShelterTest() {
    }
}
