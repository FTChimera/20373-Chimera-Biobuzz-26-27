package org.firstinspires.ftc.teamcode.testing;

import com.rmsftc.utils.ftc.DaemonManager;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/** Restores both indexes of the pinned FTCUtils registry without unloading unrelated daemons. */
final class DaemonRegistryScope implements AutoCloseable {
    private final List<DaemonManager.RegisteredDaemon> registry;
    private final Map<ClassLoader, List<DaemonManager.RegisteredDaemon>> byLoader;
    private final List<DaemonManager.RegisteredDaemon> previous;
    private final Map<ClassLoader, List<DaemonManager.RegisteredDaemon>> previousByLoader;

    @SuppressWarnings("unchecked")
    DaemonRegistryScope() throws ReflectiveOperationException {
        Field daemons = DaemonManager.class.getDeclaredField("daemons");
        Field loaders = DaemonManager.class.getDeclaredField("byLoader");
        daemons.setAccessible(true);
        loaders.setAccessible(true);
        registry = (List<DaemonManager.RegisteredDaemon>) daemons.get(null);
        byLoader = (Map<ClassLoader, List<DaemonManager.RegisteredDaemon>>) loaders.get(null);
        previous = new ArrayList<>(registry);
        previousByLoader = new LinkedHashMap<>();
        for (Map.Entry<ClassLoader, List<DaemonManager.RegisteredDaemon>> entry : byLoader.entrySet()) {
            previousByLoader.put(entry.getKey(), new ArrayList<>(entry.getValue()));
        }
    }

    @Override public void close() {
        registry.clear();
        registry.addAll(previous);
        byLoader.clear();
        byLoader.putAll(previousByLoader);
    }
}
