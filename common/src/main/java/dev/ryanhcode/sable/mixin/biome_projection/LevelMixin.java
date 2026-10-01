package dev.ryanhcode.sable.mixin.biome_projection;

import com.llamalad7.mixinextras.injector.wrapoperation.Operation;
import com.llamalad7.mixinextras.injector.wrapoperation.WrapOperation;
import dev.ryanhcode.sable.mixinhelpers.biome_projection.SubLevelBiomeProjectionHelper;
import net.minecraft.core.BlockPos;
import net.minecraft.world.level.Level;
import net.minecraft.world.level.biome.Biome;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;

@Mixin(Level.class)
public class LevelMixin {

    /**
     * {@link Biome#getPrecipitationAt} derives temperature from the position's altitude, so it must be
     * asked about the sub-level's real world position rather than its plot position.
     */
    @WrapOperation(method = "isRainingAt", at = @At(value = "INVOKE", target = "Lnet/minecraft/world/level/biome/Biome;getPrecipitationAt(Lnet/minecraft/core/BlockPos;)Lnet/minecraft/world/level/biome/Biome$Precipitation;"))
    private Biome.Precipitation sable$projectPrecipitation(final Biome instance, final BlockPos pos, final Operation<Biome.Precipitation> original) {
        return original.call(instance, SubLevelBiomeProjectionHelper.projectOutOfPlot((Level) (Object) this, pos));
    }
}
