package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.Button;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.mechanisms.Vision;

@Utility(name = "LL Tester")
public class LLTester extends LinearOpMode {
    enum Mode {
        NEURAL_DETECTOR(Vision.Pipeline.NEURAL_DETECTOR),
        APRIL_TAG(Vision.Pipeline.APRIL_TAG);

        private final Vision.Pipeline pipeline;

        Mode(Vision.Pipeline pipeline) {
            this.pipeline = pipeline;
        }

        public Vision.Pipeline getPipeline() {
            return pipeline;
        }
    }

    private Mode mode = Mode.APRIL_TAG;
    private Mode initializedMode;

    @Override
    public void runOpMode() throws InterruptedException {
        IMU imu = Constants.createIMU(hardwareMap);
        imu.resetYaw();
        Vision vision = Constants.createVision(hardwareMap);
        GamepadEx driver = new GamepadEx(gamepad1);
        try {
            while (opModeInInit()) {
                driver.update();
                if (driver.isPressed(Button.X)) {
                    mode = Mode.APRIL_TAG;
                } else if (driver.isPressed(Button.Y)) {
                    mode = Mode.NEURAL_DETECTOR;
                }

                if (initializedMode != mode) {
                    vision.start(mode.getPipeline());
                    initializedMode = mode;
                }

                if (driver.isPressed(Button.BACK)) {
                    imu.resetYaw();
                }
                if (mode == Mode.APRIL_TAG) {
                    vision.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
                }
                vision.update();
                switch (mode) {
                    case APRIL_TAG:
                        telemetry.addData("TID", vision.getLastTagId());
                        telemetry.addData("Estimated Pedro Pose", vision.getBotposeMT2());
                        break;
                    case NEURAL_DETECTOR:
                        telemetry.addData("Detector Count", vision.getDetectors() == null ? 0 : vision.getDetectors().size());
                        telemetry.addData("Highest Confidence", vision.getDetectorHighestConfidence());
                        break;
                }
                telemetry.addData("Status", "Initializing Vision");
                telemetry.addData("Mode", mode);
                telemetry.addData("Pipeline", mode.getPipeline());
                telemetry.addData("Disconnected", vision.isDisconnected());
                telemetry.addLine("Press X for AprilTag pipeline");
                telemetry.addLine("Press Y for Neural Detector pipeline");
                telemetry.addLine("Press Start when the camera has initialized");
                telemetry.update();
                idle();
            }

            if (isStopRequested()) {
                return;
            }

            while (opModeIsActive()) {
                vision.update();
                telemetry.addData("Mode", mode);
                telemetry.addData("Pipeline", mode.getPipeline());
                telemetry.addData("Disconnected", vision.isDisconnected());
                telemetry.addData("Last Tag ID", vision.getLastTagId());
                telemetry.addData("Approx Distance", vision.getLastDistanceInches());
                telemetry.addData("Tag Count", vision.getFiducials() == null ? 0 : vision.getFiducials().size());
                telemetry.addData("Detector Count", vision.getDetectors() == null ? 0 : vision.getDetectors().size());
                telemetry.update();
            }
        } finally {
            vision.stop();
        }
    }
}
