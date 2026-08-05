package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.configs.Blackboard;
import org.firstinspires.ftc.teamcode.library.DriveVector;
import org.firstinspires.ftc.teamcode.library.hardware.actuators.Motor;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.Bindings;
import org.firstinspires.ftc.teamcode.library.robot.Driver;
import org.firstinspires.ftc.teamcode.library.robot.RobotConstants;

import java.util.function.DoubleUnaryOperator;

public class Drivetrain extends Mechanism {
    public static class DRIVETRAIN_PARAMS implements Mechanism.Params<DRIVETRAIN_PARAMS> {
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
        @Override
        public DRIVETRAIN_PARAMS updateFromBot(RobotConstants bot) {
            return this;
        }
        @Override
        public DRIVETRAIN_PARAMS copy() {
            return new DRIVETRAIN_PARAMS(this);
        }
    }

    Motor[] motors;
    DoubleUnaryOperator stickCurve;
    public DRIVETRAIN_PARAMS params;
    public Drivetrain(HardwareMap map, RobotConstants bot) {
        super(map, bot);
        params = resolveParams(Drivetrain.class, new DRIVETRAIN_PARAMS());
        motors = new Motor[] {
                new Motor(map, bot.FRONT_LEFT_MOTOR_NAME).setDirection(bot.FRONT_LEFT_MOTOR_DIRECTION),
                new Motor(map, bot.FRONT_RIGHT_MOTOR_NAME).setDirection(bot.FRONT_RIGHT_MOTOR_DIRECTION),
                new Motor(map, bot.BACK_LEFT_MOTOR_NAME).setDirection(bot.BACK_LEFT_MOTOR_DIRECTION),
                new Motor(map, bot.BACK_RIGHT_MOTOR_NAME).setDirection(bot.BACK_RIGHT_MOTOR_DIRECTION)
        };
        setStickCurve(params.DEFAULT_STICK_CURVE);
        if (params.START_TELEOP_DRIVE_ON_INIT) {
            startTeleOpDrive();
        }
    }

    /** set stick curve for teleop drive (applied to all drive inputs) */
    public void setStickCurve(DoubleUnaryOperator curve) {
        this.stickCurve = curve;
    }
    /**
     * Set teleop drive using vector with field centric options
     *
     * @param driveVector Drive vector with all values
     * @param fieldCentric If true, uses field-centric drive (relative to field). If false, uses robot-centric (relative to robot)
     */

    public void setTeleOpDrive(DriveVector driveVector, boolean fieldCentric, double robotHeading) {
        DriveVector vector = driveVector.copy();
        vector.applyStickCurve(stickCurve); // Apply stick curve if set
        vector.forward = -vector.forward;
        if (fieldCentric) {
            vector.setRobotHeading(robotHeading);
            vector = vector.rotateFieldCentric();
        }
        double[] motorPowers = vector.calculateDrivePowers();

        // Set motor powers
        for (int i = 0; i < motors.length; i++) {
            motors[i].setPower(motorPowers[i]);
        }
    }
    /**
     * Set teleop drive
     *
     * @param vector Drive vector with all values
     * */

    public void setTeleOpDrive(DriveVector vector, double robotHeading) {
        Driver currentDriver = Blackboard.get(Blackboard.Caption.DRIVER) != null ? Blackboard.get(Blackboard.Caption.DRIVER).getValue() : Driver.DEFAULT;
        setTeleOpDrive(vector, currentDriver.USE_FIELD_CENTRIC, robotHeading);
    }

    /**
     * Start teleop drive mode (stop all drive motors)
     * Should be called once at the start of teleop
     */
    public void startTeleOpDrive() {
        setTeleOpDrive(DriveVector.IDLE, 0);
    }

    /**
     * Returns the drive motors as a Motor[]
     */
    public Motor[] getDriveMotors() {
        return motors;
    }

    @Override
    public void periodic(Bindings bindings) {}
}
