package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.util.vars.kt.TypesKt.BoolVar;

import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

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
    // TODO: switch back to DOuble Flywheel Full Velocity.
    public static final PIDFCoefficients shooterPIDConstants = new PIDFCoefficients(
            99, 1, 0, 0//todo tune
    );
    public DcMotorEx motor;

    public ShotCalculator shotCalculator;
    public AllianceColor color;
    public boolean panicking=false;
    private int target_velocity = 0;
    public Var<Boolean> useSotm = BoolVar("Use ShootOnMove (SOTM)", false);

    public Shooter(HardwareMap hardwareMap, AllianceColor color) {
        this.color = color;
        motor = hardwareMap.get(DcMotorEx.class, "shooter");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDConstants);
        motor.setDirection(Direction.FORWARD.toMotor());
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shotCalculator = ShotCalculator.table();
    }

    /** Sets velocity of motor in ticks per second */
    public void setVelocity(int ticksPerSecond) {
        motor.setVelocity(ticksPerSecond);
        target_velocity = ticksPerSecond;
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
        setVelocity((int) solution.flywheelSpeed);
    }

    public boolean upToSpeed() {
        return hasNonzeroVelocityTarget() && Math.abs(target_velocity - motor.getVelocity()) <= 20;
    }

    public boolean hasNonzeroVelocityTarget() {
        return target_velocity != 0;
    }

    public void update() {

    }

    public String getDebugString() {
        return "Set Velocity: " + target_velocity+ "\n Get Velocity: " + motor.getVelocity();
    }

    public void stopImmediately() {
        motor.setPower(0);
    }

    public boolean panicking() {
        return panicking;
    }
}
