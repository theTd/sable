package dev.ryanhcode.sable.neoforge;

import dev.ryanhcode.sable.Sable;
import dev.ryanhcode.sable.SableCommonEvents;
import dev.ryanhcode.sable.SableConfig;
import dev.ryanhcode.sable.SableServerConfig;
import dev.ryanhcode.sable.api.physics.SablePhysicsTuning;
import dev.ryanhcode.sable.command.SableCommand;
import dev.ryanhcode.sable.command.argument.SubLevelSelectorModifiers;
import dev.ryanhcode.sable.index.SableAttributes;
import dev.ryanhcode.sable.physics.config.FloatingBlockMaterialDataHandler;
import dev.ryanhcode.sable.physics.config.block_properties.PhysicsBlockPropertiesDefinitionLoader;
import dev.ryanhcode.sable.physics.config.dimension_physics.DimensionPhysicsData;
import net.minecraft.core.registries.BuiltInRegistries;
import net.minecraft.server.MinecraftServer;
import net.minecraft.world.entity.ai.attributes.Attribute;
import net.neoforged.bus.api.IEventBus;
import net.neoforged.fml.CrashReportCallables;
import net.neoforged.fml.ModContainer;
import net.neoforged.fml.common.Mod;
import net.neoforged.fml.config.ModConfig;
import net.neoforged.fml.event.config.ModConfigEvent;
import net.neoforged.fml.event.lifecycle.FMLCommonSetupEvent;
import net.neoforged.neoforge.common.NeoForge;
import net.neoforged.neoforge.event.AddReloadListenerEvent;
import net.neoforged.neoforge.event.OnDatapackSyncEvent;
import net.neoforged.neoforge.event.RegisterCommandsEvent;
import net.neoforged.neoforge.registries.DeferredRegister;
import net.neoforged.neoforge.server.ServerLifecycleHooks;

@Mod(Sable.MOD_ID)
public final class SableNeoForge {
    public SableNeoForge(final ModContainer modContainer, final IEventBus modBus) {
        Sable.init();

        final IEventBus neoBus = NeoForge.EVENT_BUS;
        neoBus.addListener(this::registerCommand);
        neoBus.addListener(this::registerReloadListeners);
        modBus.addListener(this::serverSetup);
        neoBus.addListener(this::syncDataPack);

        SubLevelSelectorModifiers.registerModifiers();

        final DeferredRegister<Attribute> attributes = DeferredRegister.create(BuiltInRegistries.ATTRIBUTE, Sable.MOD_ID);
        SableAttributes.PUNCH_STRENGTH = attributes.register(SableAttributes.PUNCH_STRENGTH_NAME, () -> SableAttributes.PUNCH_STRENGTH_ATTRIBUTE);
        SableAttributes.PUNCH_COOLDOWN = attributes.register(SableAttributes.PUNCH_COOLDOWN_NAME, () -> SableAttributes.PUNCH_COOLDOWN_ATTRIBUTE);
        attributes.register(modBus);

        modContainer.registerConfig(ModConfig.Type.COMMON, SableConfig.SPEC);
        modContainer.registerConfig(ModConfig.Type.SERVER, SableServerConfig.SPEC);

        modBus.addListener(ModConfigEvent.Loading.class, this::onServerConfigUpdated);
        modBus.addListener(ModConfigEvent.Reloading.class, this::onServerConfigUpdated);

        CrashReportCallables.registerHeader(Sable::getCrashHeader);
    }

    private void onServerConfigUpdated(final ModConfigEvent event) {
        if (event.getConfig().getSpec() != SableServerConfig.SPEC) {
            return;
        }

        final MinecraftServer server = ServerLifecycleHooks.getCurrentServer();
        if (server == null) {
            // Null while the server isn't running; physics systems read the config when they initialize
            return;
        }

        // Config events fire off the server thread; apply on the server thread
        server.execute(() -> SablePhysicsTuning.applyToAllLevels(server));
    }

    public void registerReloadListeners(final AddReloadListenerEvent event) {
        event.addListener(PhysicsBlockPropertiesDefinitionLoader.INSTANCE);
        event.addListener(DimensionPhysicsData.ReloadListener.INSTANCE);
        event.addListener(FloatingBlockMaterialDataHandler.ReloadListener.INSTANCE);
    }

    private void serverSetup(final FMLCommonSetupEvent event) {
        SableAttributes.register();
    }

    private void registerCommand(final RegisterCommandsEvent event) {
        SableCommand.register(event.getDispatcher(), event.getBuildContext());
    }

    private void syncDataPack(final OnDatapackSyncEvent event) {
        SableCommonEvents.syncDataPacket(packet -> event.getRelevantPlayers().forEach(player -> player.connection.send(packet)));
    }
}
