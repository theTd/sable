package dev.ryanhcode.sable.mixin.stop_rain;

import com.llamalad7.mixinextras.injector.wrapoperation.Operation;
import com.llamalad7.mixinextras.injector.wrapoperation.WrapOperation;
import dev.ryanhcode.sable.util.SubLevelRainColumn;
import net.minecraft.client.renderer.LevelRenderer;
import net.minecraft.core.BlockPos;
import net.minecraft.world.level.Level;
import net.minecraft.world.level.LevelReader;
import net.minecraft.world.level.levelgen.Heightmap;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;

/**
 * Stops precipitation from being drawn inside sub-levels.
 *
 * <p>The column arithmetic lives in {@link SubLevelRainColumn} so it can be unit-tested and called
 * from game tests; this class only wires it into the renderer.
 */
@Mixin(LevelRenderer.class)
public class LevelRenderMixin {

    /**
     * Replaces the vanilla column height with the sub-level's own roof height when that is higher.
     *
     * <p>Vanilla draws precipitation from {@code height} downwards, so a taller height means the
     * rain starts above the roof instead of inside the vehicle.
     */
    @WrapOperation(method = "renderSnowAndRain",
            at = @At(value = "INVOKE",
                    target = "Lnet/minecraft/world/level/Level;getHeight(Lnet/minecraft/world/level/levelgen/Heightmap$Types;II)I"))
    public int sable$preventRainThroughSubLevel(final Level instance, final Heightmap.Types types,
                                                final int x, final int z,
                                                final Operation<Integer> original) {
        final int vanilla = original.call(instance, types, x, z);
        final int shelter = SubLevelRainColumn.height(instance, x, z, 1);

        if (shelter != SubLevelRainColumn.NONE) {
            return Math.max(vanilla, shelter);
        }

        SubLevelRainColumn.logMiss(instance, x, z, 1);

        return vanilla;
    }

    /**
     * The same for the splash particles of {@code tickRain}, which look up a heightmap position
     * rather than a height. Offset 2 matches vanilla's own one-below-the-height convention there.
     */
    @WrapOperation(method = "tickRain",
            at = @At(value = "INVOKE",
                    target = "Lnet/minecraft/world/level/LevelReader;getHeightmapPos(Lnet/minecraft/world/level/levelgen/Heightmap$Types;Lnet/minecraft/core/BlockPos;)Lnet/minecraft/core/BlockPos;"))
    public BlockPos sable$stopSplashParticles(final LevelReader instance, final Heightmap.Types types,
                                              final BlockPos blockPos,
                                              final Operation<BlockPos> original) {
        final int vanilla = original.call(instance, types, blockPos).getY();

        if (!(instance instanceof final Level level)) {
            return new BlockPos(blockPos.getX(), vanilla, blockPos.getZ());
        }

        final int shelter = SubLevelRainColumn.height(level, blockPos.getX(), blockPos.getZ(), 2);

        if (shelter == SubLevelRainColumn.NONE) {
            return new BlockPos(blockPos.getX(), vanilla, blockPos.getZ());
        }

        return new BlockPos(blockPos.getX(), Math.max(vanilla, shelter), blockPos.getZ());
    }
}