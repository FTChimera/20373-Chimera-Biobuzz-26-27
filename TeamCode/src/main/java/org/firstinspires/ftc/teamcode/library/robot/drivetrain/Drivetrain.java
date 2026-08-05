package org.firstinspires.ftc.teamcode.library.robot.drivetrain;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.configs.Blackboard;
import org.firstinspires.ftc.teamcode.library.DriveVector;
import org.firstinspires.ftc.teamcode.library.hardware.actuators.Motor;
import org.firstinspires.ftc.teamcode.library.robot.Driver;
import org.firstinspires.ftc.teamcode.library.robot.RobotConstants;

import java.util.function.DoubleUnaryOperator;
import java.util.function.Supplier;

public class Drivetrain {
    public static class DRIVETRAIN_PARAMS {
        public DoubleUnaryOperator DEFAULT_STICK_CURVE = DoubleUnaryOperator.identity();
        public boolean START_TELEOP_DRIVE_ON_INIT = true;

        public DRIVETRAIN_PARAMS() {}

        private DRIVETRAIN_PARAMS(DRIVETRAIN_PARAMS other) {
            DEFAULT_STICK_CURVE = other.DEFAULT_STICK_CURVE;
            START_TELEOP_DRIVE_ON_INIT = other.START_TELEOP_DRIVE_ON_INIT;
        }

        public DRIVETRAIN_PARAMS defaultStickCurve(DoubleUnaryOperator defaultStickCurve) {
            this.DEFAULT_STICK_CURVE = defaultStickCurve;
            return this;
        }

        public DRIVETRAIN_PARAMS startTeleOpDriveOnInit(boolean startTeleOpDriveOnInit) {
            this.START_TELEOP_DRIVE_ON_INIT = startTeleOpDriveOnInit;
            return this;
        }

        public DRIVETRAIN_PARAMS copy() {
            return new DRIVETRAIN_PARAMS(this);
        }
    }

    private final Motor[] motors;
    private DoubleUnaryOperator stickCurve;
    public final DRIVETRAIN_PARAMS params;

    public Drivetrain(HardwareMap map, RobotConstants bot) {
        this(map, bot, new DRIVETRAIN_PARAMS());
    }

    public Drivetrain(HardwareMap map, RobotConstants bot, DRIVETRAIN_PARAMS params) {
        this.params = params.copy();
        motors = new Motor[] {
                new Motor(map, bot.FRONT_LEFT_MOTOR_NAME).setDirection(bot.FRONT_LEFT_MOTOR_DIRECTION),
                new Motor(map, bot.FRONT_RIGHT_MOTOR_NAME).setDirection(bot.FRONT_RIGHT_MOTOR_DIRECTION),
                new Motor(map, bot.BACK_LEFT_MOTOR_NAME).setDirection(bot.BACK_LEFT_MOTOR_DIRECTION),
                new Motor(map, bot.BACK_RIGHT_MOTOR_NAME).setDirection(bot.BACK_RIGHT_MOTOR_DIRECTION)
        };
        setStickCurve(this.params.DEFAULT_STICK_CURVE);
        if (this.params.START_TELEOP_DRIVE_ON_INIT) {
            startTeleOpDrive();
        }
    }

    public void setStickCurve(DoubleUnaryOperator curve) {
        this.stickCurve = curve;
    }

    public void setTeleOpDrive(DriveVector driveVector, boolean fieldCentric, double robotHeading) {
        DriveVector vector = driveVector.copy();
        vector.applyStickCurve(stickCurve);
        vector.forward = -vector.forward;
        if (fieldCentric) {
            vector.setRobotHeading(robotHeading);
            vector = vector.rotateFieldCentric();
        }
        double[] motorPowers = vector.calculateDrivePowers();
        for (int i = 0; i < motors.length; i++) {
            motors[i].setPower(motorPowers[i]);
        }
    }

    public void setTeleOpDrive(DriveVector vector, double robotHeading) {
        Driver currentDriver = Blackboard.get(Blackboard.Caption.DRIVER) != null ? Blackboard.get(Blackboard.Caption.DRIVER).getValue() : Driver.DEFAULT;
        setTeleOpDrive(vector, currentDriver.USE_FIELD_CENTRIC, robotHeading);
    }

    public void startTeleOpDrive() {
        setTeleOpDrive(DriveVector.IDLE, 0);
    }

    public Motor[] getDriveMotors() {
        return motors;
    }

    public Command setTeleOpDriveCommand(Supplier<DriveVector> vector, Supplier<Boolean> fieldCentric, Supplier<Double> robotHeading) {
        return Commands.infinite(() -> setTeleOpDrive(vector.get(), fieldCentric.get(), robotHeading.get()));
    }

    public Command setTeleOpDriveCommand(Supplier<DriveVector> vector, Supplier<Double> robotHeading) {
        return Commands.infinite(() -> setTeleOpDrive(vector.get(), robotHeading.get()));
    }

    public Command startTeleOpDriveCommand() {
        return Commands.instant(this::startTeleOpDrive);
    }
}
