package dev.ryanhcode.sable.mixin.stop_rain;

import com.llamalad7.mixinextras.injector.wrapoperation.Operation;
import com.llamalad7.mixinextras.injector.wrapoperation.WrapOperation;
import dev.ryanhcode.sable.Sable;
import dev.ryanhcode.sable.api.SubLevelHelper;
import dev.ryanhcode.sable.companion.math.BoundingBox3d;
import dev.ryanhcode.sable.companion.math.BoundingBox3dc;
import dev.ryanhcode.sable.sublevel.SubLevel;
import dev.ryanhcode.sable.util.LevelAccelerator;
import net.minecraft.client.renderer.LevelRenderer;
import net.minecraft.core.BlockPos;
import net.minecraft.world.level.Level;
import net.minecraft.world.level.LevelReader;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.level.levelgen.Heightmap;
import org.joml.Vector3d;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.Unique;
import org.spongepowered.asm.mixin.injection.At;

@Mixin(LevelRenderer.class)
public class LevelRenderMixin {

    @Unique
    private BlockPos.MutableBlockPos sable$tempPos;

    @Unique
    private static int sable$getSubLevelHeight(final Level level, final int pX, final int yOffset, final int pZ) {
        final LevelAccelerator accelerator = new LevelAccelerator(level);

        final BlockPos.MutableBlockPos mutableBlockPos = new BlockPos.MutableBlockPos();
        final Vector3d checkingPos = new Vector3d();
        final Vector3d localUp = new Vector3d(0, 1, 0);

        final BoundingBox3dc minMaxBB = new BoundingBox3d(pX, level.getMinBuildHeight(), pZ, pX + 1, level.getMaxBuildHeight(), pZ + 1);
        int maxHeight = Integer.MIN_VALUE;
        for (final SubLevel subLevel : Sable.HELPER.getAllIntersecting(level, minMaxBB)) {
            subLevel.logicalPose().transformPositionInverse(checkingPos.set(pX + 0.5f, subLevel.boundingBox().maxY(), pZ + 0.5f));
            subLevel.logicalPose().transformNormalInverse(localUp.set(0, 1, 0));

            final double checkingDistance = subLevel.boundingBox().maxY() - subLevel.boundingBox().minY();
            // 先采样后步进（2026-10-22 修复单层车顶漏雨）：入口点是世界 AABB 顶面逆变换回来的
            // 局部坐标，水平载具上数学值恰好是顶面平面 R+1，浮点回绕 δ<0 时旧写法（先步进）
            // 第一跳直接落到 R-1，把单层车顶整块跳过。先采样时 floor(R+1+δ) 只会是 R+1（空气，
            // 多耗一次迭代）或 R（命中车顶），两种舍入符号都收敛到车顶；循环上限 +1 保持探测
            // 深度不变（仍能扫到地板）。
            for (int i = 0; i < checkingDistance + 1; i++) {
                final BlockState gatheredState = accelerator.getBlockState(mutableBlockPos.set(checkingPos.x, checkingPos.y, checkingPos.z));
                // 薄顶也挡雨：原版谓词 blocksMotion 只看碰撞厚度（半砖/伪装板/活板门为 false），
                // 车厢单层薄顶会漏雨；放宽为「碰撞形状非空」即遮挡（与原版地面行为刻意分叉）
                if (gatheredState.blocksMotion() || !gatheredState.getFluidState().isEmpty()
                        || !gatheredState.getCollisionShape(accelerator, mutableBlockPos).isEmpty()) {
                    subLevel.logicalPose().transformPosition(checkingPos);
                    maxHeight = (int) Math.max(maxHeight, checkingPos.y + yOffset);
                    break;
                }
                checkingPos.sub(localUp);
            }
        }

        return maxHeight;
    }

    @WrapOperation(method = "renderSnowAndRain", at = @At(value = "INVOKE", target = "Lnet/minecraft/world/level/Level;getHeight(Lnet/minecraft/world/level/levelgen/Heightmap$Types;II)I"))
    public int sable$preventRainThoughSubLevel(final Level instance, final Heightmap.Types types, final int i, final int j, final Operation<Integer> original) {
        return Math.max(original.call(instance, types, i, j), sable$getSubLevelHeight(instance, i, 1, j));
    }

    @WrapOperation(method = "tickRain", at = @At(value = "INVOKE", target = "Lnet/minecraft/world/level/LevelReader;getHeightmapPos(Lnet/minecraft/world/level/levelgen/Heightmap$Types;Lnet/minecraft/core/BlockPos;)Lnet/minecraft/core/BlockPos;"))
    public BlockPos sable$stopSplashParticles(final LevelReader instance, final Heightmap.Types types, final BlockPos blockPos, final Operation<BlockPos> original) {
        int height = original.call(instance, types, blockPos).getY();
        if (instance instanceof final Level level) {
            height = Math.max(height, sable$getSubLevelHeight(level, blockPos.getX(), 2, blockPos.getZ()));
        }

        return new BlockPos(blockPos.getX(), height, blockPos.getZ());
    }



}
