package org.firstinspires.ftc.teamcode.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.library.control.*;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.Button;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.mechanisms.Vision;

import java.util.List;


/** A Proof of concept
 * This has been tested on the 20373 RMS Chimera Decode Gen 2 robot on Sun May 17, 2026
 * Results: Because of factors such as the lighting, the limelight was not able to detect the artifact reliably, and
 * the PIDF values haven't been tuned much. However, the robot was able to somewhat move towards whatever was detected.
 */
@Utility(name = "Preseason Artifact Sensor Auto")
public class PreseasonWebcamAuto extends LinearOpMode {
    // CONSTANTS

    com.qualcomm.robotcore.hardware.PIDFCoefficients forwardPIDF = new com.qualcomm.robotcore.hardware.PIDFCoefficients(0.9, 0, 0, 0);
    com.qualcomm.robotcore.hardware.PIDFCoefficients strafePIDF = new com.qualcomm.robotcore.hardware.PIDFCoefficients(0, 0, 0, 0);
    com.qualcomm.robotcore.hardware.PIDFCoefficients headingPIDF = new com.qualcomm.robotcore.hardware.PIDFCoefficients(0.001, 0, 0, 0);

    /**
     * Override this method and place your code here.
     * <p>
     * Please do not catch {@link InterruptedException}s that are thrown in your OpMode
     * unless you are doing it to perform some brief cleanup, in which case you must exit
     * immediately afterward. Once the OpMode has been told to stop, your ability to
     * control hardware will be limited.
     *
     * @throws InterruptedException When the OpMode is stopped while calling a method
     *                              that can throw {@link InterruptedException}
     */
    @Override
    public void runOpMode() throws InterruptedException {
        Vision vision = Constants.createVision(hardwareMap);
        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());
        PIDFController<Pose> controller = getController();
        waitForStart();
        GamepadEx driver = new GamepadEx(gamepad1);
        vision.start(Vision.Pipeline.COLOR);
        follower.startTeleopDrive();
        while (opModeIsActive()) {
            driver.update();
            vision.update();
            List<LLResultTypes.ColorResult> colors = vision.getColors();
            if (!vision.hasValidResult() || colors == null || colors.isEmpty()) {
                follower.setTeleOpDrive(0, 0, 0, true);
                controller.reset();
                telemetry.addData("Artifact Detection", "No artifact detected");
            } else {
                LLResultTypes.ColorResult colorResult = colors.get(0);
                double tx = colorResult.getTargetXPixels();
                double area = Range.clip(colorResult.getTargetArea(), 2, 70);
                double distanceProportionalApprox = 1 / Math.sqrt(area);
                controller.setTarget(new Pose(0, -distanceProportionalApprox, tx));
                controller.update(new Pose());
                Pose output = controller.getOutput();
                telemetry.addData("Controller Output", "x: %.2f, y: %.2f, heading: %.2f", output.getX(), output.getY(), output.getHeading());
                follower.setTeleOpDrive(
                        -output.getY(),
                        -output.getX(),
                        -output.getHeading(),
                        true
                );
                telemetry.addData("Artifact Detection", "Artifact detected at tx: %.2f, area: %.2f", tx, area);
            }

            // MANUAL CONTROL
            if (driver.wasJustPressed(Button.BACK)) {
                controller.reset();
            }

            if (driver.isPressed(Button.A)) {
                follower.setTeleOpDrive(0,0,0, true);
            }

            follower.update();
            telemetry.update();
        }
        vision.stop();
    }

    PIDFCoefficients<Pose> getPIDFCoeff() {
        return new PIDFCoefficients<>(
                pose -> new double[]{pose.getX(), pose.getY(), pose.getHeading()},
                array -> new Pose(array[0], array[1], array[2])
        ).setPIDF(
                new double[]{strafePIDF.p, forwardPIDF.p, headingPIDF.p},
                new double[]{strafePIDF.i, forwardPIDF.i, headingPIDF.i},
                new double[]{strafePIDF.d, forwardPIDF.d, headingPIDF.d},
                new double[]{strafePIDF.f, forwardPIDF.f, headingPIDF.f}
        ).setTolerance(0.5).setIntegralLimit(1.0);
    }
    PIDFController<Pose> getController() {
        return new PIDFController<>(getPIDFCoeff());
    }
}
