package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.util.vars.kt.TypesKt.BoolVar;

import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.control.Controller;
import org.firstinspires.ftc.teamcode.util.control.DoubleFlywheelFullVelocityConstants;
import org.firstinspires.ftc.teamcode.util.game.Field;
import org.firstinspires.ftc.teamcode.util.Direction;
import org.firstinspires.ftc.teamcode.util.hardware.actuator.SmartFlywheel;
import org.firstinspires.ftc.teamcode.util.poses.Pose2d;
import org.firstinspires.ftc.teamcode.util.shotcalc.ShotCalculator;
import org.firstinspires.ftc.teamcode.util.shotcalc.ShotContext;
import org.firstinspires.ftc.teamcode.util.shotcalc.ShotSolution;
import org.firstinspires.ftc.teamcode.util.vars.Var;

public class Shooter {
    private static final SmartFlywheel.Params MAIN_MOTOR_PARAMS = new SmartFlywheel.Params(28, 1);
    private static final SmartFlywheel.Params COUNTER_ROLLER_PARAMS = new SmartFlywheel.Params(28, 1);
    /** Ticks/sec within which a motor counts as up to speed. */
    private static final double UP_TO_SPEED_TOLERANCE_TICKS_PER_SECOND = 20;

    DoubleFlywheelFullVelocityConstants shooterVelocityConstants = new DoubleFlywheelFullVelocityConstants(
            0.0169966,  // kP
            0.00000,  // kI
            0.00668876,  // kD
            0.549526,  // kS
            0.00766308,  // kV
            0.00589096,  // kA
            30.0000,  // tolerance (ticks/sec)
            1000.00,  // integralLimit (ticks)
            true,  // voltageCompensation
            12.5000); // nominalVoltage (V)

    // TODO: tune — counterRoller velocity constants copied from the main flywheel.
    // They may differ from the main motor's, so measure and tune them separately.
    DoubleFlywheelFullVelocityConstants counterRollerVelocityConstants = new DoubleFlywheelFullVelocityConstants(
            0.0169966,  // kP
            0.00000,  // kI
            0.00668876,  // kD
            0.549526,  // kS
            0.00766308,  // kV
            0.00589096,  // kA
            30.0000,  // tolerance (ticks/sec)
            1000.00,  // integralLimit (ticks)
            true,  // voltageCompensation
            12.5000); // nominalVoltage (V)

    public SmartFlywheel mainMotor;
    public SmartFlywheel counterRoller;

    public ShotCalculator shotCalculator;
    public AllianceColor color;
    public boolean panicking=false;
    private int target_main_velocity = 0;
    private int target_counter_velocity = 0;
    public Var<Boolean> useSotm = BoolVar("Use ShootOnMove (SOTM)", false);

    public Shooter(HardwareMap hardwareMap, AllianceColor color) {
        this.color = color;
        mainMotor = new SmartFlywheel(
                hardwareMap.get(DcMotorEx.class, "mainMotor"),
                Controller.doubleFlywheelFullVelocity(shooterVelocityConstants),
                MAIN_MOTOR_PARAMS,
                hardwareMap.voltageSensor
        );
        mainMotor.motor.setDirection(Direction.FORWARD.toMotor());
        mainMotor.motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        counterRoller = new SmartFlywheel(
                hardwareMap.get(DcMotorEx.class, "counterRoller"),
                Controller.doubleFlywheelFullVelocity(counterRollerVelocityConstants),
                COUNTER_ROLLER_PARAMS,
                hardwareMap.voltageSensor
        );
        counterRoller.motor.setDirection(Direction.REVERSE.toMotor());
        counterRoller.motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        shotCalculator = ShotCalculator.table();
    }

    /** Sets velocity of both motors in ticks per second */
    public void setVelocity(int mainTicksPerSecond, int counterRollerTicksPerSecond) {
        if (mainTicksPerSecond < 0 || counterRollerTicksPerSecond < 0) {
            throw new IllegalArgumentException("Shooter velocity must be nonnegative ticks per second.");
        }
        mainMotor.setVelocity(mainTicksPerSecond);
        mainMotor.update();
        counterRoller.setVelocity(counterRollerTicksPerSecond);
        counterRoller.update();
        target_main_velocity = mainTicksPerSecond;
        target_counter_velocity = counterRollerTicksPerSecond;
        panicking = false;
    }

    public void panic() {
        mainMotor.setPower(1);
        mainMotor.update();
        counterRoller.setPower(1);
        counterRoller.update();
        panicking = true;
    }

    public void solveAndSetVelocity
            (Pose2d robotPosition, Velocity velocity, Field.HiveState hiveState, double deltaTime)
    {
        panicking = false;
        ShotContext context = new ShotContext();
        context.robotPose = robotPosition;
        context.velocity = velocity;
        context.dt = deltaTime;
        context.hiveState = hiveState;
        context.allianceColor = color;
        context.sotm = useSotm.get();
        ShotSolution solution = shotCalculator.calculateShot(context);
        setVelocity((int) Math.round(solution.flywheelTicksPerSecond),
                (int) Math.round(solution.counterRollerTicksPerSecond));
    }

    public boolean upToSpeed() {
        return hasNonzeroVelocityTarget()
                && Math.abs(target_main_velocity - mainMotor.getTicksPerSecond()) <= UP_TO_SPEED_TOLERANCE_TICKS_PER_SECOND
                && Math.abs(target_counter_velocity - counterRoller.getTicksPerSecond()) <= UP_TO_SPEED_TOLERANCE_TICKS_PER_SECOND;
    }

    public boolean hasNonzeroVelocityTarget() {
        return target_main_velocity != 0 && target_counter_velocity != 0;
    }

    public void update() {
        mainMotor.update();
        counterRoller.update();
    }

    public String getDebugString() {
        return "Set Velocity: main=" + target_main_velocity + " counter=" + target_counter_velocity
                + "\n Get Velocity: main=" + mainMotor.getTicksPerSecond()
                + " counter=" + counterRoller.getTicksPerSecond();
    }

    public void stopImmediately() {
        target_main_velocity = 0;
        target_counter_velocity = 0;
        panicking = false;
        mainMotor.stopImmediately();
        counterRoller.stopImmediately();
    }

    public boolean panicking() {
        return panicking;
    }
}
