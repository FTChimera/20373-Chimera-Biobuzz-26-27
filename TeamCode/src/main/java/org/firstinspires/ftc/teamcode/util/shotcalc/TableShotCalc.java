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
        shooterMap.add(new TableEntry(36.9, 800));
        shooterMap.add(new TableEntry(49.5, 820));
        shooterMap.add(new TableEntry(69.5, 880));
    }
    public static final List<TableEntry> counterRollerMap = new ArrayList<>();
    static {
        counterRollerMap.add(new TableEntry(30, 600));
        counterRollerMap.add(new TableEntry(40, 940));
        counterRollerMap.add(new TableEntry(69.5, 880));
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
        if (ctx.sotm && ctx.velocity != null) {
            // Shoot-on-the-move: the ball inherits robot velocity, so the
            // wheel speed that makes the shot is lower when driving toward
            // the goal (and the aim heading rotates — see
            // AutoAimerCalculator.updatePedro). Each motor's table speed is
            // compensated from its own implied launch speed; a shot that
            // cannot be made degrades to 0 ticks ("do not shoot") instead
            // of throwing — the shooter loop calls this with no guard.
            double angleToGoal = ctx.angleToTarget();
            double shotDirX = Math.cos(angleToGoal);
            double shotDirY = Math.sin(angleToGoal);
            mainVelocity = ProjectileShotCalc.compensateTicks(
                    mainVelocity, ctx.velocity.vx, ctx.velocity.vy, shotDirX, shotDirY);
            counterRollerVelocity = ProjectileShotCalc.compensateTicks(
                    counterRollerVelocity, ctx.velocity.vx, ctx.velocity.vy, shotDirX, shotDirY);
        }
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
