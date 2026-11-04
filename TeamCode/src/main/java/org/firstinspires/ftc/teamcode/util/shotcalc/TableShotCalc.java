package org.firstinspires.ftc.teamcode.util.shotcalc;

import org.firstinspires.ftc.teamcode.util.game.Ball;
import org.firstinspires.ftc.teamcode.util.interpolation.Interpolate;
import org.firstinspires.ftc.teamcode.util.math.MathUtils;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class TableShotCalc implements ShotCalculator {
    String debug = "";
    public static final List<TableEntry> shooterMap = new ArrayList<>();
    static {
        shooterMap.add(new TableEntry(30, 1000, 800));
        shooterMap.add(new TableEntry(40, 1200, 1000));
        shooterMap.add(new TableEntry(50, 1300, 1400));
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
        boolean nectar = ctx.transferBallCol.ball==Ball.NECTAR;
        debug+=("\nballcol: ");debug+=(ctx.transferBallCol);
        double distanceToTarget = ctx.distanceToTarget();
        debug+=("\ndistanceToTarget: ");debug+=(String.format(Locale.US, "(%.2f)", distanceToTarget));
        if (distanceToTarget <= shooterMap.get(0).distance) {
            return recordSolution(new ShotSolution(nectar
                    ? shooterMap.get(0).flywheelTicksPerSecondNectar
                    : shooterMap.get(0).flywheelTicksPerSecondPollen));
        }
        for (int i = 1; i < shooterMap.size(); i++) {
            TableEntry entry = shooterMap.get(i);
            if (distanceToTarget <= entry.distance) {
                TableEntry last = shooterMap.get(i - 1);
                double d0 = last.distance;
                double d1 = entry.distance;
                double r0 = nectar
                        ? last.flywheelTicksPerSecondNectar
                        : last.flywheelTicksPerSecondPollen;
                double r1 = nectar
                        ? entry.flywheelTicksPerSecondNectar
                        : entry.flywheelTicksPerSecondPollen;
                double ratio = (distanceToTarget - d0) / (d1 - d0);

                return recordSolution(new ShotSolution(
                        Interpolate.func(ratio, r0, r1, MathUtils.func.smoothFunc())
                ));
            }
        }

        TableEntry last = shooterMap.get(shooterMap.size() - 1);
        return recordSolution(new ShotSolution(nectar
                ? last.flywheelTicksPerSecondNectar
                : last.flywheelTicksPerSecondPollen));
    }

    private ShotSolution recordSolution(ShotSolution solution) {
        debug+=("\nsolution ticks/sec: ");debug+=(solution.flywheelTicksPerSecond);
        return solution;
    }

    @Override
    public String getDebugString() {
        return debug;
    }

    public static class TableEntry {
        public double distance;
        public double flywheelTicksPerSecondNectar;
        public double flywheelTicksPerSecondPollen;

        public TableEntry(double distance, double flywheelTicksPerSecondNectar, double flywheelTicksPerSecondPollen) {
            if (!Double.isFinite(distance) || distance < 0.0
                    || !Double.isFinite(flywheelTicksPerSecondNectar) || flywheelTicksPerSecondNectar < 0.0
                    || !Double.isFinite(flywheelTicksPerSecondPollen) || flywheelTicksPerSecondPollen < 0.0) {
                throw new IllegalArgumentException("Shot-table distance and velocities must be finite and nonnegative.");
            }
            this.distance = distance;
            this.flywheelTicksPerSecondNectar = flywheelTicksPerSecondNectar;
            this.flywheelTicksPerSecondPollen = flywheelTicksPerSecondPollen;
        }
    }
}
