package dev.ryanhcode.sable.mixin.biome_projection;

import com.llamalad7.mixinextras.injector.wrapmethod.WrapMethod;
import com.llamalad7.mixinextras.injector.wrapoperation.Operation;
import dev.ryanhcode.sable.mixinhelpers.biome_projection.SubLevelBiomeProjectionHelper;
import net.minecraft.core.BlockPos;
import net.minecraft.core.Holder;
import net.minecraft.world.level.Level;
import net.minecraft.world.level.biome.Biome;
import net.minecraft.world.level.biome.BiomeManager;
import org.spongepowered.asm.mixin.Final;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.Shadow;

@Mixin(BiomeManager.class)
public class BiomeManagerMixin {

    @Shadow
    @Final
    private BiomeManager.NoiseBiomeSource noiseBiomeSource;

    /**
     * Blocks on a sub-level report the biome at the sub-level's current world position instead of the
     * frozen biome baked into its plot chunks when it was created.
     */
    @WrapMethod(method = "getBiome(Lnet/minecraft/core/BlockPos;)Lnet/minecraft/core/Holder;")
    private Holder<Biome> sable$projectBiomeLookup(final BlockPos pos, final Operation<Holder<Biome>> original) {
        if (this.noiseBiomeSource instanceof final Level level) {
            return original.call(SubLevelBiomeProjectionHelper.projectOutOfPlot(level, pos));
        }

        return original.call(pos);
    }
}
