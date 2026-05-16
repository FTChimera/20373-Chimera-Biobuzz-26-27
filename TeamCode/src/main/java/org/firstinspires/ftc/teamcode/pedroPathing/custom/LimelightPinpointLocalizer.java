package org.firstinspires.ftc.teamcode.pedroPathing.custom;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Deprecated
public class LimelightPinpointLocalizer implements Localizer {
    Limelight3A limelight;
    PinpointLocalizer pinpoint;
    public LimelightPinpointLocalizer(
            PinpointConstants pinpointConstants,
            HardwareMap hardwareMap,
            String limelightName,
            int aprilTagPipelineIndex
    ) {
        pinpoint = new PinpointLocalizer(hardwareMap, pinpointConstants);
        limelight = hardwareMap.get(Limelight3A.class, limelightName);
        limelight.start();
        limelight.pipelineSwitch(aprilTagPipelineIndex);
    }
    /**
     * This returns the current pose estimate from the Localizer.
     *
     * @return returns the pose as a Pose object.
     */
    @Override
    public Pose getPose() {
        return pinpoint.getPose();
    }

    /**
     * This returns the current velocity estimate from the Localizer.
     *
     * @return returns the velocity as a Pose object.
     */
    @Override
    public Pose getVelocity() {
        return pinpoint.getVelocity();
    }

    /**
     * This returns the current velocity estimate from the Localizer as a Vector.
     *
     * @return returns the velocity as a Vector.
     */
    @Override
    public Vector getVelocityVector() {
        return pinpoint.getVelocityVector();
    }

    /**
     * This sets the start pose of the Localizer. Changing the start pose should move the robot as if
     * all its previous movements were displacing it from its new start pose.
     *
     * @param setStart the new start pose
     */
    @Override
    public void setStartPose(Pose setStart) {
        pinpoint.setStartPose(setStart);
    }

    /**
     * This sets the current pose estimate of the Localizer. Changing this should just change the
     * robot's current pose estimate, not anything to do with the start pose.
     *
     * @param setPose the new current pose estimate
     */
    @Override
    public void setPose(Pose setPose) {
        pinpoint.setPose(setPose);
    }

    /**
     * This calls an update to the Localizer, updating the current pose estimate and current velocity
     * estimate.
     */
    @Override
    public void update() {
        pinpoint.update();
        try {
            limelight.updateRobotOrientation(pinpoint.getPose().getAsCoordinateSystem(FTCCoordinates.INSTANCE).getHeading());
            Pose3D botpose = limelight.getLatestResult().getBotpose_MT2();
            double x = botpose.getPosition().toUnit(DistanceUnit.INCH).x;
            double y = botpose.getPosition().toUnit(DistanceUnit.INCH).z;
            double heading = botpose.getOrientation().getYaw(AngleUnit.RADIANS);
            if (Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(heading)) return;
            Pose limelightPose = new Pose(x, y, heading);
            pinpoint.setPose(FTCCoordinates.INSTANCE.convertToPedro(limelightPose));
        } catch (Exception ignored) {
            // You can add stuff in here if you want, but it is ignored for now
        }
    }

    /**
     * This returns how far the robot has turned in radians, in a number not clamped between 0 and
     * 2 * pi radians. This is used for some tuning things and nothing actually within the following.
     *
     * @return returns how far the robot has turned in total, in radians.
     */
    @Override
    public double getTotalHeading() {
        return pinpoint.getTotalHeading();
    }

    /**
     * This returns the multiplier applied to forward movement measurement to convert from encoder
     * ticks to inches. This is found empirically through a tuner.
     *
     * @return returns the forward ticks to inches multiplier
     */
    @Override
    public double getForwardMultiplier() {
        return pinpoint.getForwardMultiplier();
    }

    /**
     * This returns the multiplier applied to lateral/strafe movement measurement to convert from
     * encoder ticks to inches. This is found empirically through a tuner.
     *
     * @return returns the lateral/strafe ticks to inches multiplier
     */
    @Override
    public double getLateralMultiplier() {
        return pinpoint.getLateralMultiplier();
    }

    /**
     * This returns the multiplier applied to turning movement measurement to convert from encoder
     * ticks to radians. This is found empirically through a tuner.
     *
     * @return returns the turning ticks to radians multiplier
     */
    @Override
    public double getTurningMultiplier() {
        return pinpoint.getTurningMultiplier();
    }

    /**
     * This resets the IMU of the localizer, if applicable.
     */
    @Override
    public void resetIMU() {
        pinpoint.resetIMU();
    }

    /**
     * This is overridden to return the IMU's heading estimate, if there is one.
     *
     * @return returns the IMU's heading estimate if it exists
     */
    @Override
    public double getIMUHeading() {
        return pinpoint.getIMUHeading();
    }

    /**
     * This returns whether if any component of robot's position is NaN.
     *
     * @return returns if any component of the robot's position is NaN
     */
    @Override
    public boolean isNAN() {
        return pinpoint.isNAN();
    }
}
