package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.util.vars.kt.TypesKt.BoolVar;

import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.control.Controller;
import org.firstinspires.ftc.teamcode.util.control.DoubleFlywheelFullVelocityConstants;
import org.firstinspires.ftc.teamcode.util.game.BallCol;
import org.firstinspires.ftc.teamcode.util.game.Field;
import org.firstinspires.ftc.teamcode.util.Direction;
import org.firstinspires.ftc.teamcode.util.hardware.actuator.SmartFlywheel;
import org.firstinspires.ftc.teamcode.util.poses.Pose2d;
import org.firstinspires.ftc.teamcode.util.shotcalc.ShotCalculator;
import org.firstinspires.ftc.teamcode.util.shotcalc.ShotContext;
import org.firstinspires.ftc.teamcode.util.shotcalc.ShotSolution;
import org.firstinspires.ftc.teamcode.util.vars.Var;

public class Shooter {
    private static final SmartFlywheel.Params SHOOTER_PARAMS = new SmartFlywheel.Params(28, 1);
    public static final DoubleFlywheelFullVelocityConstants shooterVelocityConstants = new DoubleFlywheelFullVelocityConstants(
            0.0169037, 0.0, 0.00377037,
            0.0000, 0.00570009, 0.00666907,
            30.0000, 1000.00,
            true, 12.000
    );
    public SmartFlywheel motor;

    public ShotCalculator shotCalculator;
    public AllianceColor color;
    public boolean panicking=false;
    private int target_velocity = 0;
    public Var<Boolean> useSotm = BoolVar("Use ShootOnMove (SOTM)", false);

    public Shooter(HardwareMap hardwareMap, AllianceColor color) {
        this.color = color;
        motor = new SmartFlywheel(
                hardwareMap.get(DcMotorEx.class, "shooter"),
                Controller.doubleFlywheelFullVelocity(shooterVelocityConstants),
                SHOOTER_PARAMS,
                hardwareMap.voltageSensor
        );
        motor.motor.setDirection(Direction.FORWARD.toMotor());
        motor.motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shotCalculator = ShotCalculator.table();
    }

    /** Sets velocity of motor in ticks per second */
    public void setVelocity(int ticksPerSecond) {
        if (ticksPerSecond < 0) {
            throw new IllegalArgumentException("Shooter velocity must be nonnegative ticks per second.");
        }
        motor.setVelocity(ticksPerSecond);
        motor.update();
        target_velocity = ticksPerSecond;
        panicking = false;
    }

    public void panic() {
        motor.setPower(1);
        motor.update();
        panicking = true;
    }
    public void solveAndSetVelocity
            (Pose2d robotPosition, Velocity velocity, Field.HiveState hiveState, BallCol transferBallCol, double deltaTime)
    {
        panicking = false;
        ShotContext context = new ShotContext();
        context.robotPose = robotPosition;
        context.velocity = velocity;
        context.dt = deltaTime;
        context.hiveState = hiveState;
        context.allianceColor = color;
        context.transferBallCol = transferBallCol;
        context.sotm = useSotm.get();
        ShotSolution solution = shotCalculator.calculateShot(context);
        setVelocity((int) Math.round(solution.flywheelTicksPerSecond));
    }

    public boolean upToSpeed() {
        return hasNonzeroVelocityTarget() && Math.abs(target_velocity - motor.getTicksPerSecond()) <= 20;
    }

    public boolean hasNonzeroVelocityTarget() {
        return target_velocity != 0;
    }

    public void update() {
        motor.update();
    }

    public String getDebugString() {
        return "Set Velocity: " + target_velocity+ "\n Get Velocity: " + motor.getTicksPerSecond();
    }

    public void stopImmediately() {
        target_velocity = 0;
        panicking = false;
        motor.stopImmediately();
    }

    public boolean panicking() {
        return panicking;
    }
}
