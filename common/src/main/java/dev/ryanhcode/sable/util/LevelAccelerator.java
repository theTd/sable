package dev.ryanhcode.sable.util;

import dev.ryanhcode.sable.mixin.level_accelerator.ServerChunkCacheAccessor;
import it.unimi.dsi.fastutil.longs.Long2ObjectMap;
import it.unimi.dsi.fastutil.longs.Long2ObjectOpenHashMap;
import net.minecraft.core.BlockPos;
import net.minecraft.core.Vec3i;
import net.minecraft.server.level.ChunkHolder;
import net.minecraft.server.level.ChunkResult;
import net.minecraft.world.level.BlockGetter;
import net.minecraft.world.level.ChunkPos;
import net.minecraft.world.level.Level;
import net.minecraft.world.level.block.Blocks;
import net.minecraft.world.level.block.entity.BlockEntity;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.level.chunk.LevelChunk;
import net.minecraft.world.level.chunk.LevelChunkSection;
import net.minecraft.world.level.material.FluidState;
import net.minecraft.world.level.material.Fluids;
import org.jetbrains.annotations.Nullable;

/**
 * Speeds up block/fluid state, chunk, and level access with caching and raw access.
 *
 * <p>By default a missing server chunk is loaded synchronously, like {@link Level#getChunk}. An
 * accelerator built with {@code loadMissingChunks = false} never loads: {@link #getChunk} returns
 * null for a chunk that is not fully loaded, and block/fluid reads there see air. Use that mode on
 * per-tick server paths, where a synchronous load can stall the server thread for seconds.
 */
public class LevelAccelerator implements BlockGetter {
    public static final boolean USE_CACHE_MAP = false;

    private final Level level;
    private final boolean loadMissingChunks;
    private long cachedChunkPos = 0L;
    private LevelChunk cachedChunkObj = null;
    private final int minBuildHeight;
    private final int minSection;
    private final int maxBuildHeight;
    private final Long2ObjectMap<LevelChunk> cachedLevelChunks = new Long2ObjectOpenHashMap<>();

    public LevelAccelerator(final Level level) {
        this(level, true);
    }

    /**
     * @param loadMissingChunks whether a server chunk that is not fully loaded is loaded
     *                          synchronously (true) or treated as absent (false)
     */
    public LevelAccelerator(final Level level, final boolean loadMissingChunks) {
        this.level = level;
        this.loadMissingChunks = loadMissingChunks;
        this.minBuildHeight = level.getMinBuildHeight();
        this.maxBuildHeight = level.getMaxBuildHeight();
        this.minSection = level.getMinSection();
    }

    public void clearCache() {
        this.cachedLevelChunks.clear();
        this.cachedChunkObj = null;
        this.cachedChunkPos = 0L;
    }

    public void setBlockFast(final BlockPos blockPos, final BlockState blockState) {
        final LevelChunk chunk = this.getChunk(blockPos);
        final BlockState blockState2 = chunk.setBlockState(blockPos, blockState, false);
        if (blockState2 == null) {
            return;
        }

        this.level.sendBlockUpdated(blockPos, blockState2, blockState, 3);
    }

    @Override
    public @Nullable BlockEntity getBlockEntity(final BlockPos blockPos) {
        return this.level.getBlockEntity(blockPos);
    }

    @Override
    public BlockState getBlockState(final BlockPos pos) {
        final LevelChunk chunk = this.getChunk(pos);
        if (chunk == null) {
            return Blocks.AIR.defaultBlockState();
        }
        return this.getBlockState(chunk, pos);
    }

    /**
     * Gets the blockstate at a position in a chunk given that the chunk is already known.
     *
     * @param chunk The chunk to get the blockstate from
     * @param pos   The position to get the blockstate from
     * @return The blockstate at the position
     */
    public BlockState getBlockState(final LevelChunk chunk, final BlockPos pos) {
        if (pos.getY() < this.minBuildHeight || pos.getY() >= this.maxBuildHeight) {
            return Blocks.AIR.defaultBlockState();
        }

        final LevelChunkSection section = chunk.getSection((pos.getY() >> 4) - this.minSection);
        return section.getBlockState(pos.getX() & 15, pos.getY() & 15, pos.getZ() & 15);
    }

    @Override
    public FluidState getFluidState(final BlockPos pos) {
        final LevelChunk chunk = this.getChunk(pos);
        if (chunk == null) {
            return Fluids.EMPTY.defaultFluidState();
        }

        return chunk.getFluidState(pos);
    }

    /**
     * @return whether the chunk containing the position is available without loading it
     */
    public boolean hasChunk(final BlockPos pos) {
        return this.getChunk(pos) != null;
    }

    /**
     * @return the chunk, or null only when this accelerator does not load missing chunks
     */
    public LevelChunk getChunk(final BlockPos pos) {
        return this.getChunk(pos.getX() >> 4, pos.getZ() >> 4);
    }

    /**
     * @return the chunk, or null only when this accelerator does not load missing chunks
     */
    public LevelChunk getChunk(final int chunkX, final int chunkZ) {
        final long pos = ChunkPos.asLong(chunkX, chunkZ);

        if (pos == this.cachedChunkPos && this.cachedChunkObj != null) {
            return this.cachedChunkObj;
        }

        final LevelChunk chunk;

        if (USE_CACHE_MAP) {
            chunk = this.cachedLevelChunks.computeIfAbsent(pos, x -> this.grabChunkFast(chunkX, chunkZ, pos));
        } else {
            chunk = this.grabChunkFast(chunkX, chunkZ, pos);
        }

        if (chunk == null) {
            // Not cached: the chunk may finish loading before the next lookup.
            return null;
        }

        this.cachedChunkObj = chunk;
        this.cachedChunkPos = pos;

        return chunk;
    }

    private @Nullable LevelChunk grabChunkFast(final int chunkX, final int chunkZ, final long pos) {
        if (this.level.isClientSide) {
            return this.level.getChunk(chunkX, chunkZ);
        }

        final ChunkHolder holder = ((ServerChunkCacheAccessor) this.level.getChunkSource()).invokeGetVisibleChunkIfPresent(pos);

        if (holder != null) {
            final LevelChunk res = holder.getFullChunkFuture().getNow(ChunkResult.error("No chunk at position")).orElse(null);

            if (res != null)
                return res;
        }

        return this.loadMissingChunks ? this.level.getChunk(chunkX, chunkZ) : null;
    }

    public boolean isOutsideBuildHeight(final Vec3i pos) {
        return pos.getY() < this.minBuildHeight || pos.getY() >= this.maxBuildHeight;
    }

    @Override
    public int getHeight() {
        return this.level.getHeight();
    }

    @Override
    public int getMinBuildHeight() {
        return this.minBuildHeight;
    }
}
