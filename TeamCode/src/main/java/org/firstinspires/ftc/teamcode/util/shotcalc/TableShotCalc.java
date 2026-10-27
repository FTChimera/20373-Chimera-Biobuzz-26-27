package org.firstinspires.ftc.teamcode.util.shotcalc;

import com.pedropathing.math.Twist;

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
        shooterMap.add(new TableEntry(50, 1400, 1200));
        //todo tune
    }
    @Override
    public ShotSolution calculateShot(ShotContext context) {
        debug = "";
        ShotContext ctx = context.copy();
        debug+= ("SOTM: "); debug+=(ctx.sotm);
        if (ctx.sotm) {
            ctx.robotPose = ctx.robotPose.exp(ctx.velocity, ctx.dt);
        }
        ShotSolution solution = null;
        boolean nectar = ctx.transferBallCol.ball==Ball.NECTAR;
        debug+=("\nballcol: ");debug+=(ctx.transferBallCol);
        debug+=("\ndistanceToTarget: ");debug+=(String.format(Locale.US, "(%.2f)", ctx.distanceToTarget()));
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
        debug+=("\nsolution Speed: ");debug+=(solution.flywheelSpeed);
        return solution;
    }

    @Override
    public String getDebugString() {
        return debug;
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
