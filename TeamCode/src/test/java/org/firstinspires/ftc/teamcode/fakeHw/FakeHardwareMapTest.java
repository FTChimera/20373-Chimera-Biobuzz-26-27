package org.firstinspires.ftc.teamcode.fakeHw;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.SerialNumber;
import java.util.*;
import org.junit.Test;
import org.firstinspires.ftc.teamcode.testing.AndroidLogScope;
import static org.junit.Assert.*;

public class FakeHardwareMapTest {
    @Test public void constructorsAndStrictInformativeLookups() {
        assertEquals(0, new FakeHardwareMap(null, null).size());
        FakeHardwareMap map = new FakeHardwareMap();
        FakeServo servo = map.register(" deployer ", new FakeServo());
        assertSame(servo, map.get(Servo.class, "deployer"));
        assertSame(servo, map.servo.get(" deployer "));
        assertSame(servo, map.get("deployer"));
        IllegalArgumentException missing = assertThrows(IllegalArgumentException.class,
                () -> map.get(DcMotorEx.class, "missing"));
        assertTrue(missing.getMessage().contains("missing"));
        assertTrue(missing.getMessage().contains("DcMotorEx"));
        assertTrue(missing.getMessage().contains("deployer"));
        IllegalArgumentException mismatch = assertThrows(IllegalArgumentException.class,
                () -> map.dcMotor.get("deployer"));
        assertTrue(mismatch.getMessage().contains("FakeServo"));
        assertNull(map.tryGet(DcMotorEx.class, "deployer"));
        assertNull(map.tryGet(Servo.class, "unknown"));
        assertThrows(IllegalArgumentException.class, () -> map.register("deployer", servo));
        assertThrows(IllegalArgumentException.class, () -> map.servo.put("deployer", new FakeServo()));
        assertSame(servo, map.servo.get("deployer"));
        assertThrows(IllegalArgumentException.class, () -> map.get(" "));
        assertThrows(NullPointerException.class, () -> map.put("bad", (HardwareDevice) null));
        assertThrows(IllegalArgumentException.class,
                () -> map.put("bad", Arrays.asList(servo, servo)));
        assertEquals(1, map.size());
    }

    @Test public void sdkPutIsOverallOnlyAndMappingPutIsLocalAndGlobal() {
        FakeHardwareMap map = new FakeHardwareMap();
        FakeServo global = new FakeServo();
        map.put("global", global);
        assertSame(global, map.get(Servo.class, "global"));
        assertFalse(map.servo.contains("global"));
        assertThrows(IllegalArgumentException.class, () -> map.servo.get("global"));
        map.servo.putLocal("global", global);
        assertSame(global, map.servo.get("global"));
        FakeServo local = new FakeServo();
        map.servo.put("local", local);
        assertSame(local, map.get(Servo.class, "local"));
        assertSame(local, map.servo.get("local"));
        assertThrows(IllegalArgumentException.class,
                () -> map.servo.putLocal("ghost", new FakeServo()));
        assertThrows(IllegalArgumentException.class,
                () -> map.servo.putLocal("global", global));
        assertEquals(2, map.servo.size());
    }

    @Test public void aliasesDeduplicateOverallDevicesButNotMappingEntries() {
        FakeHardwareMap map = new FakeHardwareMap();
        FakeVoltageSensor battery = map.register("battery", new FakeVoltageSensor());
        map.register("alias", battery);
        map.register("servo", new FakeServo());
        assertEquals(2, map.size());
        assertEquals(2, map.getAll(HardwareDevice.class).size());
        assertEquals(1, map.getAll(VoltageSensor.class).size());
        assertEquals(2, map.voltageSensor.size());
        List<VoltageSensor> mappingValues = new ArrayList<>();
        map.voltageSensor.forEach(mappingValues::add);
        assertEquals(Arrays.asList(battery, battery), mappingValues);
        assertEquals(new TreeSet<>(Arrays.asList("alias", "battery")),
                map.getAllNames(VoltageSensor.class));
        assertEquals(new HashSet<>(Arrays.asList("battery", "alias")), map.getNamesOf(battery));
        Iterator<HardwareDevice> before = map.iterator();
        assertTrue(map.voltageSensor.remove("alias"));
        assertEquals(2, map.size());
        assertEquals(1, map.voltageSensor.size());
        assertEquals(2, count(before));
        assertTrue(map.remove("battery", battery));
        assertFalse(map.voltageSensor.contains("battery"));
        assertNull(map.tryGet(VoltageSensor.class, "battery"));
        assertFalse(map.voltageSensor.remove("battery"));
        assertEquals(1, map.size());
    }

    @Test public void serialLookupsAndRemovalFollowSdkOptionalSemantics() {
        FakeHardwareMap map = new FakeHardwareMap();
        SerialNumber serial = SerialNumber.fromString("fake-battery");
        FakeVoltageSensor battery = new FakeVoltageSensor();
        map.voltageSensor.put(serial, "battery", battery);
        assertSame(battery, map.get(VoltageSensor.class, serial));
        assertNull(map.get(Servo.class, serial));
        assertNull(map.get(VoltageSensor.class, SerialNumber.fromString("missing")));
        assertThrows(IllegalArgumentException.class,
                () -> map.put(serial, "other", new FakeVoltageSensor()));
        assertNull(map.tryGet(HardwareDevice.class, "other"));
        assertTrue(map.voltageSensor.remove(serial, "battery"));
        assertNull(map.get(VoltageSensor.class, serial));
    }

    @Test public void registrationPopulatesEveryApplicableMappingAndImmutableInspection() {
        FakeHardwareMap map = new FakeHardwareMap();
        FakeColorSensor color = map.register("color", new FakeColorSensor());
        assertSame(color, map.colorSensor.get("color"));
        assertSame(color, map.get(DistanceSensor.class, "color"));
        Map<String, HardwareDevice> before = map.inspect();
        assertThrows(UnsupportedOperationException.class,
                () -> before.put("invalid", color));
        Set<Map.Entry<String, ColorSensor>> entries = map.colorSensor.entrySet();
        assertThrows(UnsupportedOperationException.class,
                () -> entries.iterator().next().setValue(new FakeColorSensor()));
        map.register("servo", new FakeServo());
        assertEquals(1, before.size());
        List<FakeHardwareMap.Call> calls = map.getCallHistory();
        map.get("servo");
        assertTrue(map.getCallHistory().size() > calls.size());
        assertThrows(UnsupportedOperationException.class, calls::clear);
        assertEquals(1, map.getCallCount("mapping.get"));
        map.clearCallHistory();
        assertEquals(0, map.getCallHistory().size());
    }

    @Test public void plantAliasesAdvanceOnceAndBatteryStimulusIsShared() {
        FakeHardwareMap map = new FakeHardwareMap();
        FakeDcMotorEx motor = map.register("motor", new FakeDcMotorEx());
        map.register("encoderAlias", motor);
        FakeVoltageSensor battery = map.register("battery", new FakeVoltageSensor());
        motor.setFreeSpeedTicksPerSecond(1000);
        motor.setTimeConstant(0.1);
        map.setBatteryVoltage(6);
        motor.setPower(1);
        map.advance(1);
        assertEquals(6, battery.getVoltage(), 0);
        assertEquals(6, motor.getBatteryVoltage(), 0);
        assertEquals(500 * (1 - Math.exp(-10)), motor.getVelocity(), 1e-8);
        assertEquals(1, motor.getElapsedSeconds(), 0);
        assertEquals(1, map.getElapsedSeconds(), 0);
        assertEquals(1, motor.getCallCount("advance"));
        assertThrows(IllegalArgumentException.class, () -> map.advance(-1));
        assertThrows(IllegalArgumentException.class, () -> map.advance(Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> map.advance(Double.POSITIVE_INFINITY));
        assertThrows(IllegalArgumentException.class, () -> map.setBatteryVoltage(-1));
        assertEquals(1, map.getElapsedSeconds(), 0);
    }

    @Test public void realSdkMotorWrappersResolveSamePhysicalPlant() {
        FakeHardwareMap map = new FakeHardwareMap();
        FakeDcMotorController controller = new FakeDcMotorController(1);
        FakeDcMotorEx plant = controller.getMotor(0);
        plant.setFreeSpeedTicksPerSecond(1000);
        DcMotorImplEx wrapper;
        try (AndroidLogScope logging = new AndroidLogScope()) {
            wrapper = new DcMotorImplEx(controller, 0, DcMotorSimple.Direction.FORWARD,
                    plant.getMotorType());
        }
        map.register("wrapped", wrapper);
        map.register("physical", plant);
        map.register("controller", controller);
        wrapper.setPower(0.5);
        map.advance(0.5);
        assertEquals(plant.getCurrentPosition(), wrapper.getCurrentPosition());
        assertEquals(plant.getVelocity(), wrapper.getVelocity(), 1e-8);
        assertEquals(0.5, plant.getElapsedSeconds(), 0);
        assertEquals(1, plant.getCallCount("advance"));
        map.setBatteryVoltage(9);
        assertEquals(9, plant.getBatteryVoltage(), 0);
    }

    private static int count(Iterator<?> iterator) {
        int count = 0;
        while (iterator.hasNext()) { iterator.next(); count++; }
        return count;
    }
}
