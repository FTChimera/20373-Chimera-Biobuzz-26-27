package org.firstinspires.ftc.teamcode.configs;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import com.rmsftc.library.robot.Driver;

import java.util.ArrayList;
import java.util.List;

/**
 * Blackboard - A shared data structure for storing global information
 *
 * This class serves as a central repository for data that needs to be accessed across multiple subsystems and op modes.
 * It can be used to store the robot's current pose, sensor readings, or any other information that should be globally accessible.
 *
 * Key Features:
 * - Static variables for easy access across the codebase
 * - Configurable fields for tuning parameters at runtime
 *
 */
@Configurable
public class Blackboard {
    private static final List<Item> items = new ArrayList<>(List.of(new Item(Caption.POSE, new Pose()),  new Item(Caption.DRIVER, Driver.DEFAULT)));

    public static Item get(Caption cap) {
        for (Item item : items) {
            if (item.cap == cap) {
                return item;
            }
        }
        return null;
    }
    public static void put(Caption cap, Object value) {
        put(new Item(cap, value));
    }
    public static void put(Item item) {
        for (Item existing : items) {
            if (existing.cap == item.cap) {
                existing.setValue(item.val);
                return;
            }
        }
        items.add(item);
    }
    public static void resetAll() {
        for (Item item: items) {
            item.reset();
        }
    }


    public enum Caption {
        POSE(Pose.class),
        DRIVER(Driver.class);

        Class<?> clazz;
        Caption(Class<?> objectClass) {
            clazz = objectClass;
        }
    }

    public static class Item {
        protected Object val;
        protected Caption cap;
        protected final Object defaultV;
        public Item(Caption caption, Object value) {
            if (caption.clazz != value.getClass()) {
                throw new IllegalArgumentException("The value class does not match the caption");
            }
            val = value;
            cap = caption;
            defaultV = value;
        }

        public <T> T getValue() {
            return (T) cap.clazz.cast(val);
        }
        public Caption getCaption() {
            return cap;
        }
        public void setValue(Object value) {
            if (cap.clazz != value.getClass()) {
                throw new IllegalArgumentException("The value class does not match the caption");
            }
            val = value;
        }
        public void reset() {
            val = defaultV;
        }
    }
}
