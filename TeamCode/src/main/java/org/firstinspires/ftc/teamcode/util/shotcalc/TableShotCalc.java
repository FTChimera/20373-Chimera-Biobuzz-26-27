package org.firstinspires.ftc.teamcode.util.shotcalc;

import org.firstinspires.ftc.teamcode.util.interpolation.Interpolate;
import org.firstinspires.ftc.teamcode.util.math.MathUtils;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class TableShotCalc implements ShotCalculator {
    String debug = "";
    public static final List<TableEntry> shooterMap = new ArrayList<>();
    static {
        shooterMap.add(new TableEntry(30, 800));
        shooterMap.add(new TableEntry(40, 1000));
        shooterMap.add(new TableEntry(50, 1400));
        //todo tune
    }
    // TODO: tune — counterRoller table is a placeholder copied from the main
    // flywheel table. Measure the counterRoller's real distance -> velocity
    // entries and replace these.
    public static final List<TableEntry> counterRollerMap = new ArrayList<>();
    static {
        counterRollerMap.add(new TableEntry(30, 800));
        counterRollerMap.add(new TableEntry(40, 1000));
        counterRollerMap.add(new TableEntry(50, 1400));
        //todo tune
    }

    @Override
    public ShotSolution calculateShot(ShotContext context) {
        if (context == null) {
            throw new IllegalArgumentException("Shot context is required.");
        }
        debug = "";
        ShotContext ctx = context.copy();
        ctx.validate();
        debug+= ("SOTM: "); debug+=(ctx.sotm);
        if (ctx.sotm) {
            ctx.robotPose = ctx.poseAtRelease();
        }
        double distanceToTarget = ctx.distanceToTarget();
        debug+=("\ndistanceToTarget: ");debug+=(String.format(Locale.US, "(%.2f)", distanceToTarget));
        double mainVelocity = interpolate(shooterMap, distanceToTarget);
        double counterRollerVelocity = interpolate(counterRollerMap, distanceToTarget);
        return recordSolution(new ShotSolution(mainVelocity, counterRollerVelocity));
    }

    /** Interpolates a distance -> velocity table, clamping outside its range. */
    private double interpolate(List<TableEntry> map, double distanceToTarget) {
        if (distanceToTarget <= map.get(0).distance) {
            return map.get(0).flywheelTicksPerSecond;
        }
        for (int i = 1; i < map.size(); i++) {
            TableEntry entry = map.get(i);
            if (distanceToTarget <= entry.distance) {
                TableEntry last = map.get(i - 1);
                double d0 = last.distance;
                double d1 = entry.distance;
                double r0 = last.flywheelTicksPerSecond;
                double r1 = entry.flywheelTicksPerSecond;
                double ratio = (distanceToTarget - d0) / (d1 - d0);

                return Interpolate.func(ratio, r0, r1, MathUtils.func.smoothFunc());
            }
        }

        TableEntry last = map.get(map.size() - 1);
        return last.flywheelTicksPerSecond;
    }

    private ShotSolution recordSolution(ShotSolution solution) {
        debug+=("\nsolution ticks/sec: main=");debug+=(solution.flywheelTicksPerSecond);
        debug+=(" counter=");debug+=(solution.counterRollerTicksPerSecond);
        return solution;
    }

    @Override
    public String getDebugString() {
        return debug;
    }

    public static class TableEntry {
        public double distance;
        public double flywheelTicksPerSecond;

        public TableEntry(double distance, double flywheelTicksPerSecond) {
            if (!Double.isFinite(distance) || distance < 0.0
                    || !Double.isFinite(flywheelTicksPerSecond) || flywheelTicksPerSecond < 0.0) {
                throw new IllegalArgumentException("Shot-table distance and velocities must be finite and nonnegative.");
            }
            this.distance = distance;
            this.flywheelTicksPerSecond = flywheelTicksPerSecond;
        }
    }
}
