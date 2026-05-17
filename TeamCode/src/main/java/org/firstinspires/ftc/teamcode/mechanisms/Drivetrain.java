package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.DriveVector;
import org.firstinspires.ftc.teamcode.library.hardware.actuators.Motor;

import java.util.function.DoubleUnaryOperator;

public class Drivetrain {
    Motor[] motors;
    DoubleUnaryOperator stickCurve;
    public Drivetrain(HardwareMap map) {
        motors = new Motor[] {
                new Motor(map, Constants.currentBot.FRONT_LEFT_MOTOR_NAME).setDirection(Constants.currentBot.FRONT_LEFT_MOTOR_DIRECTION),
                new Motor(map, Constants.currentBot.FRONT_RIGHT_MOTOR_NAME).setDirection(Constants.currentBot.FRONT_RIGHT_MOTOR_DIRECTION),
                new Motor(map, Constants.currentBot.BACK_LEFT_MOTOR_NAME).setDirection(Constants.currentBot.BACK_LEFT_MOTOR_DIRECTION),
                new Motor(map, Constants.currentBot.BACK_RIGHT_MOTOR_NAME).setDirection(Constants.currentBot.BACK_RIGHT_MOTOR_DIRECTION)
        };
        startTeleOpDrive();
        setStickCurve(DoubleUnaryOperator.identity()); // Identity curve f(x) = x
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
        if (fieldCentric) {
            vector.setRobotHeading(robotHeading);
            vector = vector.rotateFieldCentric();
        }
        double[] motorPowers = vector.rotateDrivePowers();

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
        setTeleOpDrive(vector, Constants.DRIVE_FIELD_RELATIVE, robotHeading);
    }

    /**
     * Start teleop drive mode (stop all drive motors)
     * Should be called once at the start of teleop
     */
    public void startTeleOpDrive() {
        setTeleOpDrive(DriveVector.IDLE, 0);
    }


}

