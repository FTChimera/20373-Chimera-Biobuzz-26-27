package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.hardware.webcam.Limelight;

@TeleOp(name = "LL Tester", group = "1: Tests")
public class LLTester extends LinearOpMode {
    enum Mode {
        NEURAL_DETECTOR(Limelight.Pipeline.NEURAL_DETECTOR),
        APRIL_TAG(Limelight.Pipeline.APRIL_TAG);

        private final Limelight.Pipeline pipeline;

        Mode(Limelight.Pipeline pipeline) {
            this.pipeline = pipeline;
        }

        public Limelight.Pipeline getPipeline() {
            return pipeline;
        }
    }

    private Mode mode = Mode.APRIL_TAG;
    private Mode initializedMode;

    @Override
    public void runOpMode() throws InterruptedException {
        if (!Constants.hasMechanism(Limelight.class)) {
            while (opModeInInit()) {
                telemetry.addData("Limelight", "Not registered in selected profile");
                telemetry.addData("Configured Mechanisms", Constants.getMechanisms());
                telemetry.update();
                idle();
            }
            return;
        }
        IMU imu = Constants.createIMU(hardwareMap);
        imu.resetYaw();
        Limelight limelight = Constants.createLimelight(hardwareMap);
        try {
            while (opModeInInit()) {
                if (gamepad1.x) {
                    mode = Mode.APRIL_TAG;
                } else if (gamepad1.y) {
                    mode = Mode.NEURAL_DETECTOR;
                }

                if (initializedMode != mode) {
                    limelight.start(mode.getPipeline());
                    initializedMode = mode;
                }

                if (gamepad1.back) {
                    imu.resetYaw();
                }
                if (mode == Mode.APRIL_TAG) {
                    limelight.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
                }
                limelight.LLUpdate();
                switch (mode) {
                    case APRIL_TAG:
                        telemetry.addData("TID", limelight.tid);
                        telemetry.addData("Estimated Pedro Pose", limelight.getBotposeMT2());
                        break;
                    case NEURAL_DETECTOR:
                        telemetry.addData("Detector Count", limelight.getDetectors() == null ? 0 : limelight.getDetectors().size());
                        telemetry.addData("Highest Confidence", limelight.getDetectorHighestConfidence());
                        break;
                }
                telemetry.addData("Status", "Initializing Limelight");
                telemetry.addData("Mode", mode);
                telemetry.addData("Pipeline", mode.getPipeline());
                telemetry.addData("Disconnected", limelight.isDisconnected);
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
                limelight.LLUpdate();
                telemetry.addData("Mode", mode);
                telemetry.addData("Pipeline", mode.getPipeline());
                telemetry.addData("Disconnected", limelight.isDisconnected);
                telemetry.addData("Last Tag ID", limelight.tid);
                telemetry.addData("Approx Distance", limelight.dist);
                telemetry.addData("Tag Count", limelight.getFiducials() == null ? 0 : limelight.getFiducials().size());
                telemetry.addData("Detector Count", limelight.getDetectors() == null ? 0 : limelight.getDetectors().size());
                telemetry.update();
            }
        } finally {
            limelight.stop();
        }
    }
}
