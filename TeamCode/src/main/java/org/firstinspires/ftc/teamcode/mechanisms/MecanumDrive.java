package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.configs.Blackboard;
import org.firstinspires.ftc.teamcode.library.DriveVector;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.robot.Driver;
import org.firstinspires.ftc.teamcode.library.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.CommandConverter;

import java.util.function.DoubleUnaryOperator;
import java.util.function.Supplier;

/**
 * MecanumDrive - Drivetrain subsystem using PedroPathing Follower
 *
 * This subsystem wraps the PedroPathing Follower to provide teleop drive control.
 * The Follower handles all motor control, odometry, and path following automatically.
 *
 * Key Features:
 * - Uses PedroPathing Follower for motor control
 * - Supports robot-centric and field-centric drive modes
 * - Provides pose tracking and velocity information
 * - Handles odometry updates automatically
 *
 * @author Team
 * @version 1.0 - Initial implementation based on DuneStrider
 */
public class MecanumDrive extends Mechanism {

    // PedroPathing Follower that handles all localization
    public Follower follower;
    // Drivetrain for teleop control (don't use follower directly)
    public Drivetrain drivetrain;
    public boolean useManualDrive; // Set to false to use follower for teleop drive

    // Last known pose (updated in periodic)
    public static Pose lastPose = new Pose(0, 0, 0);
    public double headingOffset = 0; // Offset to apply to heading for field-centric control

    /**
     * Constructor - Initializes the Follower with starting pose
     *
     * @param map HardwareMap for initializing motors and sensors
     */

    public MecanumDrive(HardwareMap map, RobotConstants bot) {
        super(map, bot);
        // Create follower using Constants configuration
        this.follower = Constants.createFollower(map);
        useManualDrive = !bot.USE_FOLLOWER_DRIVE;
        // Initialize drivetrain
        drivetrain = Constants.createDrivetrain(map);
        // Set initial curve
        resetStickCurve();
        // Initial update to sync pose
        follower.update();
    }

    public void setStartingPose(Pose pose) {
        follower.setStartingPose(pose);
    }
    /**
     * Periodic update - should be called every loop iteration
     * Updates pose tracking and follower state
     */
    public void periodic() {
        // Update last known pose
        lastPose = follower.getPose();

        // Update follower (handles odometry and motor control)
        follower.update();
    }

    /**
     * Set teleop drive using vector with field centric options
     *
     * @param driveVector Drive vector with all values
     * @param fieldCentric If true, uses field-centric drive (relative to field). If false, uses robot-centric (relative to robot)
     */

    public void setTeleOpDrive(DriveVector driveVector, boolean fieldCentric) {
        if (useManualDrive) {
            drivetrain.setTeleOpDrive(driveVector, fieldCentric, lastPose.getHeading() - headingOffset);
        } else {
            follower.setTeleOpDrive(
                    driveVector.forward, driveVector.strafe, driveVector.rotate, fieldCentric, fieldCentric ? headingOffset : 0
            );
        }
    }
    /**
     * Set teleop drive
     *
     * @param vector Drive vector with all values
     * */

    public void setTeleOpDrive(DriveVector vector) {
        Driver currentDriver = Blackboard.get(Blackboard.Caption.DRIVER) != null ? Blackboard.get(Blackboard.Caption.DRIVER).getValue() : Driver.DEFAULT;
        setTeleOpDrive(vector, currentDriver.USE_FIELD_CENTRIC);
    }

    /**
     * Start teleop drive mode (stop all drive motors)
     * Should be called once at the start of teleop
     */
    public void startTeleOpDrive() {
        if (useManualDrive) {
            drivetrain.startTeleOpDrive();
        } else {
            follower.startTeleopDrive();
            follower.setTeleOpDrive(0, 0, 0, false); // Stop all motion
        }

    }

    /**
     * Get the current estimated robot pose
     *
     * @return Current pose (x, y, heading in radians)
     */
    public Pose getPose() {
        return follower.getPose();
    }

    /**
     * Reset robot heading to a new value
     * Useful for relocalization or IMU resets
     *
     * @param newHeading New heading in radians
     */
    public void resetHeading(double newHeading) {
        Pose currentPose = follower.getPose();
        follower.setPose(currentPose.setHeading(newHeading));
    }
    /**
     * Set robot heading offset to a new value
     * Useful for field-centric control adjustments
     *
     * @param newHeading New heading in radians
     */
    public void setHeadingOffset(double newHeading) {
        headingOffset = newHeading;
    }
    /**
     * Set robot heading offset to the current heading
     * <p>
     * Useful for resetting field-centric control to current orientation
     */
    public void zeroHeadingOffset() {
        headingOffset = follower.getPose().getHeading();
    }
    /**
     * Reset robot position to a new pose
     * <p>
     * Useful for relocalization
     *
     * @param pose New pose
     */
    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    /**
     * Get current robot velocity
     *
     * @return Velocity vector (x, y components in inches per second)
     */
    public Vector getVelocity() {
        return follower.getVelocity();
    }

    /**
     * Get current angular velocity
     *
     * @return Angular velocity in radians per second
     */
    public double getAngularVelocity() {
        return follower.getAngularVelocity();
    }

    /**
     * Get current acceleration
     *
     * @return Acceleration vector (x, y components)
     */
    public Vector getAcceleration() {
        return follower.getAcceleration();
    }

    // Ivy Commands
    /** Command to set teleop drive with field centric options */
    public Command teleOp(Supplier<DriveVector> vector, boolean fieldCentric) {
        return Groups.sequential(
                CommandConverter.toCommand(this::startTeleOpDrive),
                Commands.infinite(() -> this.setTeleOpDrive(vector.get(), fieldCentric))
        );
    }
    /** Command to set teleop drive */
    public Command teleOp(Supplier<DriveVector> vector) {
        Driver currentDriver = Blackboard.get(Blackboard.Caption.DRIVER) != null ? Blackboard.get(Blackboard.Caption.DRIVER).getValue() : Driver.DEFAULT;
        return teleOp(vector, currentDriver.USE_FIELD_CENTRIC);
    }

    /** Command to update the follower every iteration */
    public Command periodicCommand() {
        return Commands.infinite(this::periodic);
    }

    public void setStickCurve(DoubleUnaryOperator curve) {
        drivetrain.setStickCurve(curve);
    }

    public void resetStickCurve() {
        drivetrain.setStickCurve(DoubleUnaryOperator.identity());
    }
}