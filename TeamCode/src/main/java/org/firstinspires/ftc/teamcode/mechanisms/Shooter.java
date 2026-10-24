package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.util.vars.kt.TypesKt.BoolVar;

import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    public static final DoubleFlywheelFullVelocityConstants shooterPIDConstants = new DoubleFlywheelFullVelocityConstants(
            0, 0, 0, 0, 0, 0, 0, 0
    );
    public SmartFlywheel motor;

    public ShotCalculator shotCalculator;
    public AllianceColor color;
    public boolean panicking=false;
    public Var<Boolean> useSotm = BoolVar("Use ShootOnMove (SOTM)", false);

    public Shooter(HardwareMap hardwareMap, AllianceColor color) {
        this.color = color;
        motor = new SmartFlywheel(
                hardwareMap.get(DcMotorEx.class, "shooter"),
                Controller.doubleFlywheelFullVelocity(shooterPIDConstants),
                new SmartFlywheel.Params(28, 1) // for the Gobilda 5203 6000 RPM Motor
        );
        motor.motor.setDirection(Direction.REVERSE.toMotor());
        motor.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shotCalculator = ShotCalculator.table();
    }

    /** Sets velocity of motor in ticks per second */
    public void setVelocity(double ticksPerSecond) {
        motor.setVelocity(ticksPerSecond);
        panicking = false;
    }

    public void panic() {
        motor.setPower(1);
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
        setVelocity(solution.flywheelSpeed);
    }

    public boolean upToSpeed() {
        return Math.abs(motor.targetTicksPerSecond - motor.getTicksPerSecond()) <= 20;
    }

    public void update() {
        motor.update();
    }

    public String getDebugString() {
        return "Set Velocity: " + motor.targetTicksPerSecond + "\n Get Velocity: " + motor.getTicksPerSecond();
    }

    public void stopImmediately() {
        motor.stopImmediately();
    }

    public boolean panicking() {
        return panicking;
    }
}
