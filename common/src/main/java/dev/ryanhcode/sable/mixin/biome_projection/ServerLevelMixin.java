package dev.ryanhcode.sable.mixin.biome_projection;

import com.llamalad7.mixinextras.injector.wrapoperation.Operation;
import com.llamalad7.mixinextras.injector.wrapoperation.WrapOperation;
import dev.ryanhcode.sable.SableServerConfig;
import dev.ryanhcode.sable.api.sublevel.SubLevelContainer;
import dev.ryanhcode.sable.mixinhelpers.biome_projection.SubLevelBiomeProjectionHelper;
import net.minecraft.core.BlockPos;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.world.level.biome.Biome;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;

@Mixin(ServerLevel.class)
public class ServerLevelMixin {

    /**
     * Rain-driven block behaviour (cauldron filling, etc.) must use the temperature at the sub-level's
     * real world position, not at its plot position.
     */
    @WrapOperation(method = "tickPrecipitation", at = @At(value = "INVOKE", target = "Lnet/minecraft/world/level/biome/Biome;getPrecipitationAt(Lnet/minecraft/core/BlockPos;)Lnet/minecraft/world/level/biome/Biome$Precipitation;"))
    private Biome.Precipitation sable$projectPrecipitation(final Biome instance, final BlockPos pos, final Operation<Biome.Precipitation> original) {
        final ServerLevel level = (ServerLevel) (Object) this;
        if (SableServerConfig.SPEC.isLoaded() && SableServerConfig.BIOME_PROJECTION_FAST_PATH.getAsBoolean()) {
            final SubLevelContainer container = SubLevelContainer.getContainer(level);
            if (container == null || !container.hasSubLevels()) {
                return original.call(instance, pos);
            }
        }

        return original.call(instance, SubLevelBiomeProjectionHelper.projectOutOfPlot(level, pos));
    }
}
