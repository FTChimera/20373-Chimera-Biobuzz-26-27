package org.firstinspires.ftc.teamcode.library.hardware.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.lang.reflect.Field;

final class GamepadTestUtils {
    private static final Object UNSAFE = loadUnsafe();
    private static final java.lang.reflect.Method ALLOCATE_INSTANCE = loadAllocateInstanceMethod();

    private GamepadTestUtils() {}

    static Gamepad newGamepad(Gamepad.Type type) {
        try {
            Gamepad gamepad = (Gamepad) ALLOCATE_INSTANCE.invoke(UNSAFE, Gamepad.class);
            gamepad.type = type;
            return gamepad;
        } catch (ReflectiveOperationException e) {
            throw new AssertionError("Unable to allocate FTC Gamepad for tests", e);
        }
    }

    private static Object loadUnsafe() {
        try {
            Class<?> unsafeClass = Class.forName("sun.misc.Unsafe");
            Field field = unsafeClass.getDeclaredField("theUnsafe");
            field.setAccessible(true);
            return field.get(null);
        } catch (ReflectiveOperationException e) {
            throw new AssertionError("Unable to access Unsafe for Gamepad allocation", e);
        }
    }

    private static java.lang.reflect.Method loadAllocateInstanceMethod() {
        try {
            java.lang.reflect.Method method = UNSAFE.getClass().getMethod("allocateInstance", Class.class);
            method.setAccessible(true);
            return method;
        } catch (ReflectiveOperationException e) {
            throw new AssertionError("Unable to access Unsafe.allocateInstance for Gamepad allocation", e);
        }
    }
}
