package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.library.DriveVector;
import org.firstinspires.ftc.teamcode.Constants;
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
public class MecanumDrive {

    // PedroPathing Follower that handles all localization
    public Follower follower;
    // Drivetrain for teleop control (don't use follower directly)
    public Drivetrain drivetrain;

    // Last known pose (updated in periodic)
    public static Pose lastPose = new Pose(0, 0, 0);

    /**
     * Constructor - Initializes the Follower with starting pose
     *
     * @param map HardwareMap for initializing motors and sensors
     * @param startingPose Initial robot pose (x, y, heading). If null, uses (0, 0, 0)
     */

    public static MecanumDrive customLocalizer(HardwareMap map, Pose startingPose) {
        return new MecanumDrive(map, startingPose, true);
    }
    public static MecanumDrive pinpointLocalizer(HardwareMap map, Pose startingPose) {
        return new MecanumDrive(map, startingPose, false);
    }
    MecanumDrive(HardwareMap map, Pose startingPose, boolean customLocalizer) {
        // Create follower using Constants configuration
        if (customLocalizer) {
            this.follower = Constants.customLocalizerFollower(map);
        } else {
            this.follower = Constants.pinpointFollower(map);
        }

        // Initialize drivetrain
        Mechanisms.drivetrain(map);
        // Set starting pose (use (0, 0, 0) if not provided)
        follower.setStartingPose(startingPose == null ? new Pose(0, 0, 0) : startingPose);
        // Set initial curve
        resetStickCurve();
        // Initial update to sync pose
        follower.update();
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
        drivetrain.setTeleOpDrive(driveVector, fieldCentric, lastPose.getHeading());
    }
    /**
     * Set teleop drive
     *
     * @param vector Drive vector with all values
     * */

    public void setTeleOpDrive(DriveVector vector) {
       setTeleOpDrive(vector, Constants.DRIVE_FIELD_RELATIVE);
    }

    /**
     * Start teleop drive mode (stop all drive motors)
     * Should be called once at the start of teleop
     */
    public void startTeleopDrive() {
        drivetrain.startTeleopDrive();
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
     * Reset robot position to a new pose
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
                CommandConverter.toCommand(this::startTeleopDrive),
                Commands.infinite(() -> this.setTeleOpDrive(vector.get(), fieldCentric))
        );
    }
    /** Command to set teleop drive */
    public Command teleOp(Supplier<DriveVector> vector) {
        return teleOp(vector, Constants.DRIVE_FIELD_RELATIVE);
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