package org.firstinspires.ftc.teamcode.fakeHw;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.SerialNumber;
import java.util.*;

/**
 * Strict test map. {@link #register} populates compatible SDK mappings; SDK {@code put}
 * remains overall-map-only. Names are trimmed and case sensitive. Unlike the real SDK,
 * replacement and multiple drivers under one name are deliberately rejected.
 */
public class FakeHardwareMap extends HardwareMap {
    public static final class Call {
        public final String operation;
        public final String name;
        public final Class<?> type;
        public final double seconds;
        private Call(String operation, String name, Class<?> type, double seconds) {
            this.operation = operation;
            this.name = name;
            this.type = type;
            this.seconds = seconds;
        }
    }

    private final Map<String, HardwareDevice> devices = new LinkedHashMap<>();
    private final Map<SerialNumber, HardwareDevice> serials = new LinkedHashMap<>();
    private final List<Call> calls = new ArrayList<>();
    private double elapsedSeconds;

    public FakeHardwareMap() { this(null, null); }

    public FakeHardwareMap(Context appContext, OpModeManagerNotifier notifier) {
        super(appContext, notifier);
        allDeviceMappings.clear();
        dcMotorController = mapping(DcMotorController.class);
        dcMotor = mapping(DcMotor.class);
        servoController = mapping(ServoController.class);
        servo = mapping(Servo.class);
        crservo = mapping(CRServo.class);
        touchSensorMultiplexer = mapping(TouchSensorMultiplexer.class);
        analogInput = mapping(AnalogInput.class);
        digitalChannel = mapping(DigitalChannel.class);
        opticalDistanceSensor = mapping(OpticalDistanceSensor.class);
        touchSensor = mapping(TouchSensor.class);
        pwmOutput = mapping(PWMOutput.class);
        i2cDevice = mapping(I2cDevice.class);
        i2cDeviceSynch = mapping(I2cDeviceSynch.class);
        colorSensor = mapping(ColorSensor.class);
        led = mapping(LED.class);
        accelerationSensor = mapping(AccelerationSensor.class);
        compassSensor = mapping(CompassSensor.class);
        gyroSensor = mapping(GyroSensor.class);
        irSeekerSensor = mapping(IrSeekerSensor.class);
        lightSensor = mapping(LightSensor.class);
        ultrasonicSensor = mapping(UltrasonicSensor.class);
        voltageSensor = mapping(VoltageSensor.class);
    }

    private <T extends HardwareDevice> DeviceMapping<T> mapping(Class<T> type) {
        DeviceMapping<T> result = new StrictMapping<>(type);
        allDeviceMappings.add(result);
        return result;
    }

    private static String name(String value) {
        Objects.requireNonNull(value, "device name");
        String result = value.trim();
        if (result.isEmpty()) throw new IllegalArgumentException("Hardware name must not be blank");
        return result;
    }

    private void record(String operation, String name, Class<?> type) {
        calls.add(new Call(operation, name, type, elapsedSeconds));
    }

    /** Canonical configuration API; the returned object is convenient for stimulus/assertions. */
    public synchronized <T extends HardwareDevice> T register(String deviceName, T device) {
        put(deviceName, device);
        for (DeviceMapping<? extends HardwareDevice> mapping : allDeviceMappings) {
            if (mapping.getDeviceTypeClass().isInstance(device)) addLocal(mapping, deviceName, device);
        }
        return device;
    }

    private <T extends HardwareDevice> void addLocal(DeviceMapping<T> mapping, String name,
                                                    HardwareDevice device) {
        mapping.putLocal(name, mapping.cast(device));
    }

    @Override public synchronized void put(String name, HardwareDevice device) {
        internalPut(null, name, device);
    }

    @Override public synchronized void put(String name, List<HardwareDevice> instances) {
        Objects.requireNonNull(instances, "instances");
        if (instances.size() != 1) {
            throw new IllegalArgumentException("Strict fake map requires exactly one device per name");
        }
        put(name, instances.get(0));
    }

    @Override public synchronized void put(SerialNumber serial, String name, HardwareDevice device) {
        internalPut(Objects.requireNonNull(serial, "serial"), name, device);
    }

    @Override protected synchronized void internalPut(SerialNumber serial, String deviceName,
                                                      HardwareDevice device) {
        String key = name(deviceName);
        Objects.requireNonNull(device, "device");
        if (devices.containsKey(key)) {
            throw new IllegalArgumentException("Duplicate hardware name '" + key + "' already maps to "
                    + devices.get(key).getClass().getName());
        }
        if (serial != null && serials.containsKey(serial)) {
            throw new IllegalArgumentException("Duplicate hardware serial " + serial);
        }
        devices.put(key, device);
        if (serial != null) serials.put(serial, device);
        record("put", key, device.getClass());
    }

    private IllegalArgumentException lookupError(Class<?> type, String key) {
        HardwareDevice actual = devices.get(key);
        return new IllegalArgumentException("Hardware '" + key + "' requested as " + type.getName()
                + (actual == null ? " is missing" : " is actually " + actual.getClass().getName())
                + "; configured names: " + devices.keySet());
    }

    @Override public synchronized <T> T get(Class<? extends T> type, String deviceName) {
        Objects.requireNonNull(type, "type");
        String key = name(deviceName);
        record("get", key, type);
        HardwareDevice device = devices.get(key);
        if (!type.isInstance(device)) throw lookupError(type, key);
        return type.cast(device);
    }

    /** SDK-compatible optional lookup: null only for a missing device or incompatible type. */
    @Override public synchronized <T> T tryGet(Class<? extends T> type, String deviceName) {
        Objects.requireNonNull(type, "type");
        String key = name(deviceName);
        record("tryGet", key, type);
        HardwareDevice device = devices.get(key);
        return type.isInstance(device) ? type.cast(device) : null;
    }

    /** The SDK serial-number lookup is optional, unlike its strict named lookup. */
    @Override public synchronized <T> T get(Class<? extends T> type, SerialNumber serial) {
        Objects.requireNonNull(type, "type");
        Objects.requireNonNull(serial, "serial");
        record("getSerial", serial.toString(), type);
        HardwareDevice device = serials.get(serial);
        return type.isInstance(device) ? type.cast(device) : null;
    }

    @Override public HardwareDevice get(String name) { return get(HardwareDevice.class, name); }

    private List<HardwareDevice> uniqueDevices() {
        Set<HardwareDevice> seen = Collections.newSetFromMap(new IdentityHashMap<>());
        List<HardwareDevice> result = new ArrayList<>();
        for (HardwareDevice device : devices.values()) if (seen.add(device)) result.add(device);
        return result;
    }

    @Override public synchronized <T> List<T> getAll(Class<? extends T> type) {
        Objects.requireNonNull(type, "type");
        record("getAll", null, type);
        List<T> result = new ArrayList<>();
        for (HardwareDevice device : uniqueDevices()) {
            if (type.isInstance(device)) result.add(type.cast(device));
        }
        return result;
    }

    @Override public synchronized SortedSet<String> getAllNames(Class<? extends HardwareDevice> type) {
        Objects.requireNonNull(type, "type");
        SortedSet<String> result = new TreeSet<>();
        for (Map.Entry<String, HardwareDevice> entry : devices.entrySet()) {
            if (type.isInstance(entry.getValue())) result.add(entry.getKey());
        }
        return result;
    }

    @Override public synchronized Set<String> getNamesOf(HardwareDevice device) {
        Objects.requireNonNull(device, "device");
        Set<String> result = new LinkedHashSet<>();
        for (Map.Entry<String, HardwareDevice> entry : devices.entrySet()) {
            if (entry.getValue() == device) result.add(entry.getKey());
        }
        return result;
    }

    @Override public synchronized int size() { return uniqueDevices().size(); }
    @Override public synchronized Iterator<HardwareDevice> iterator() {
        return uniqueDevices().iterator();
    }
    @Override public synchronized Iterable<HardwareDevice> unsafeIterable() {
        return uniqueDevices();
    }

    @Override public synchronized boolean remove(String name, HardwareDevice device) {
        return remove(null, name, device);
    }

    @Override public synchronized boolean remove(SerialNumber serial, String deviceName,
                                                HardwareDevice device) {
        String key = name(deviceName);
        Objects.requireNonNull(device, "device");
        if (devices.get(key) != device) return false;
        if (serial != null && serials.get(serial) != device) return false;
        devices.remove(key);
        for (DeviceMapping<? extends HardwareDevice> mapping : allDeviceMappings) {
            ((StrictMapping<?>) mapping).local.remove(key);
        }
        boolean stillRegistered = false;
        for (HardwareDevice value : devices.values()) if (value == device) stillRegistered = true;
        if (!stillRegistered) serials.values().removeIf(value -> value == device);
        else if (serial != null) serials.remove(serial);
        record("remove", key, device.getClass());
        return true;
    }

    public synchronized Map<String, HardwareDevice> inspect() {
        return Collections.unmodifiableMap(new LinkedHashMap<>(devices));
    }
    public synchronized List<Call> getCallHistory() {
        return Collections.unmodifiableList(new ArrayList<>(calls));
    }
    public synchronized int getCallCount(String operation) {
        Objects.requireNonNull(operation, "operation");
        int count = 0;
        for (Call call : calls) if (operation.equals(call.operation)) count++;
        return count;
    }
    public synchronized void clearCallHistory() { calls.clear(); }
    @Override public synchronized void logDevices() {
        record("logDevices", null, HardwareDevice.class);
        for (Map.Entry<String, HardwareDevice> entry : devices.entrySet()) {
            System.out.println(entry.getKey() + ": " + entry.getValue().getClass().getName());
        }
    }

    private List<FakeSteppable> plants() {
        Set<FakeSteppable> seen = Collections.newSetFromMap(new IdentityHashMap<>());
        List<FakeSteppable> result = new ArrayList<>();
        for (HardwareDevice device : uniqueDevices()) {
            FakeSteppable plant = null;
            if (device instanceof FakeSteppable) plant = (FakeSteppable) device;
            else if (device instanceof DcMotor) {
                DcMotor motor = (DcMotor) device;
                if (motor.getController() instanceof FakeDcMotorController) {
                    plant = ((FakeDcMotorController) motor.getController()).getMotor(motor.getPortNumber());
                }
            }
            if (plant != null && seen.add(plant)) result.add(plant);
        }
        return result;
    }

    /** Advances each physical owner once, including SDK wrappers sharing a registered plant. */
    public synchronized void advance(double seconds) {
        FakeSensorDevice.finite(seconds, "seconds");
        if (seconds < 0 || !Double.isFinite(elapsedSeconds + seconds)) {
            throw new IllegalArgumentException("seconds must be nonnegative and total time finite");
        }
        for (FakeSteppable plant : plants()) plant.advance(seconds);
        elapsedSeconds += seconds;
        record("advance", null, FakeSteppable.class);
    }

    public synchronized double getElapsedSeconds() { return elapsedSeconds; }

    /** Explicit shared battery stimulus; arbitrary real hardware is not altered. */
    public synchronized void setBatteryVoltage(double voltage) {
        FakeSensorDevice.finite(voltage, "voltage");
        if (voltage < 0) throw new IllegalArgumentException("voltage must be nonnegative");
        for (FakeSteppable plant : plants()) {
            if (plant instanceof FakeDcMotorEx) ((FakeDcMotorEx) plant).setBatteryVoltage(voltage);
        }
        for (HardwareDevice device : uniqueDevices()) {
            if (device instanceof FakeVoltageSensor) ((FakeVoltageSensor) device).setVoltage(voltage);
        }
        record("setBatteryVoltage", null, VoltageSensor.class);
    }

    private final class StrictMapping<T extends HardwareDevice> extends DeviceMapping<T> {
        private final Class<T> type;
        private final Map<String, T> local = new LinkedHashMap<>();
        private StrictMapping(Class<T> type) { super(type); this.type = type; }
        @Override public T get(String deviceName) {
            synchronized (FakeHardwareMap.this) {
                String key = name(deviceName);
                record("mapping.get", key, type);
                T device = local.get(key);
                if (device == null) {
                    if (!type.isInstance(devices.get(key))) throw lookupError(type, key);
                    throw new IllegalArgumentException("Hardware '" + key + "' is not in the "
                            + type.getSimpleName() + " DeviceMapping; use register() or mapping.put()");
                }
                return device;
            }
        }
        @Override public void put(String name, T device) { internalPut(null, name, device); }
        @Override public void put(SerialNumber serial, String name, T device) {
            internalPut(Objects.requireNonNull(serial, "serial"), name, device);
        }
        @Override protected void internalPut(SerialNumber serial, String name, T device) {
            synchronized (FakeHardwareMap.this) {
                type.cast(Objects.requireNonNull(device, "device"));
                FakeHardwareMap.this.internalPut(serial, name, device);
                putLocal(name, device);
            }
        }
        @Override public void putLocal(String deviceName, T device) {
            synchronized (FakeHardwareMap.this) {
                String key = name(deviceName);
                type.cast(Objects.requireNonNull(device, "device"));
                if (local.containsKey(key)) throw new IllegalArgumentException("Duplicate mapping name '" + key + "'");
                // SDK's local-only insertion is supported only for an already globally registered device.
                if (devices.get(key) != device) {
                    throw new IllegalArgumentException("putLocal requires the same device registered as '" + key + "'");
                }
                local.put(key, device);
            }
        }
        @Override public boolean contains(String name) {
            synchronized (FakeHardwareMap.this) { return local.containsKey(name(name)); }
        }
        @Override public boolean remove(String name) { return remove(null, name); }
        @Override public boolean remove(SerialNumber serial, String deviceName) {
            synchronized (FakeHardwareMap.this) {
                String key = name(deviceName);
                T device = local.get(key);
                return device != null && FakeHardwareMap.this.remove(serial, key, device);
            }
        }
        @Override public Iterator<T> iterator() {
            synchronized (FakeHardwareMap.this) { return new ArrayList<>(local.values()).iterator(); }
        }
        @Override public Set<Map.Entry<String, T>> entrySet() {
            synchronized (FakeHardwareMap.this) {
                Set<Map.Entry<String, T>> result = new LinkedHashSet<>();
                for (Map.Entry<String, T> entry : local.entrySet()) {
                    result.add(new AbstractMap.SimpleImmutableEntry<>(entry));
                }
                return result;
            }
        }
        @Override public int size() {
            synchronized (FakeHardwareMap.this) { return local.size(); }
        }
    }
}
