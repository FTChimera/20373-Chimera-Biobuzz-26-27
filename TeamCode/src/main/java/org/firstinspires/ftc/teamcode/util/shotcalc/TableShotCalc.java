package org.firstinspires.ftc.teamcode.util.shotcalc;

import com.pedropathing.math.Twist;

import org.firstinspires.ftc.teamcode.util.game.Ball;
import org.firstinspires.ftc.teamcode.util.interpolation.Interpolate;
import org.firstinspires.ftc.teamcode.util.math.MathUtils;

import java.util.Collections;
import java.util.List;

public class TableShotCalc implements ShotCalculator {
    public static final List<TableEntry> shooterMap = Collections.emptyList();
    static {
        shooterMap.add(new TableEntry(30, 1000, 800));
        shooterMap.add(new TableEntry(40, 1200, 1000));
        shooterMap.add(new TableEntry(50, 1400, 1200));
        //todo tune
    }
    @Override
    public ShotSolution calculateShot(ShotContext context) {
        ShotContext ctx = context.copy();
        if (ctx.sotm) {
            ctx.robotPose = ctx.robotPose.exp(ctx.velocity, ctx.dt);
        }
        ShotSolution solution = null;
        boolean nectar = ctx.transferBallCol.ball==Ball.NECTAR;
        if (ctx.distanceToTarget() <= shooterMap.get(0).distance) {
            solution = new ShotSolution(nectar? shooterMap.get(0).flywheelRPMNectar : shooterMap.get(0).flywheelRPMPollen);
        }
        for (int i = 0; i < shooterMap.size(); i++) {
            TableEntry entry = shooterMap.get(i);
            if (ctx.distanceToTarget() < entry.distance) {
                TableEntry last = shooterMap.get(i - 1);
                double d0 = last.distance;
                double d1 = entry.distance;
                double r0 = nectar?last.flywheelRPMNectar:last.flywheelRPMPollen;
                double r1 = nectar?entry.flywheelRPMNectar:entry.flywheelRPMPollen;
                double ratio = (ctx.distanceToTarget() - d0) / (d1 - d0);

                solution = new ShotSolution(Interpolate.func(ratio, r0, r1, MathUtils.func.smoothFunc()));
            }
        }
        if (solution == null) {
            solution = new ShotSolution(nectar? shooterMap.get(shooterMap.size() - 1).flywheelRPMNectar : shooterMap.get(shooterMap.size()-1).flywheelRPMPollen);
        }
        return solution;
    }
    public static class TableEntry {
        public double distance;
        public double flywheelRPMNectar;
        public double flywheelRPMPollen;
        public TableEntry(double distance, double flywheelRPMNectar, double flywheelRPMPollen) {
            this.distance = distance;
            this.flywheelRPMNectar = flywheelRPMNectar;
            this.flywheelRPMPollen = flywheelRPMPollen;
        }
    }
}
