package dev.ryanhcode.sable.api.physics;

import dev.ryanhcode.sable.api.sublevel.ServerSubLevelContainer;
import dev.ryanhcode.sable.api.sublevel.SubLevelContainer;
import dev.ryanhcode.sable.physics.config.PhysicsConfigData;
import dev.ryanhcode.sable.sublevel.system.SubLevelPhysicsSystem;
import net.minecraft.server.MinecraftServer;
import net.minecraft.server.level.ServerLevel;
import org.jetbrains.annotations.Nullable;

/**
 * Applies the server physics config to the physics systems of all running levels.
 */
public final class SablePhysicsTuning {

    private SablePhysicsTuning() {
    }

    /**
     * Re-reads the server config into every level's physics system and pushes it to its pipeline.
     * Does nothing when no server is currently running (e.g. on a connected client).
     *
     * @param server the current server, or null
     */
    public static void applyToAllLevels(final @Nullable MinecraftServer server) {
        if (server == null) {
            return;
        }

        for (final ServerLevel level : server.getAllLevels()) {
            final ServerSubLevelContainer container = SubLevelContainer.getContainer(level);
            if (container == null) {
                continue;
            }

            final SubLevelPhysicsSystem physicsSystem = container.physicsSystem();
            final PhysicsConfigData config = physicsSystem.getConfig();
            config.updateFromConfig();
            physicsSystem.getPipeline().updateConfigFrom(config);
        }
    }
}
