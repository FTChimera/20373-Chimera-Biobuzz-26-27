package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.TuningOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.planners.Constants;

public class DeployerManualTuner extends Procedure {
    private static final String DEPLOYER_NAME = "deployer";

    public DeployerManualTuner() {
        super("Deployer Manual Tuner",
                "Manually tunes the deployer ground, deployed, and stowed positions.");
    }

    @Override
    public void run() throws InterruptedException {
        confirmation(
                "Manual deployer tuning",
                "This runs three OpModes: ground, deployed, then stowed. In each one, hold "
                        + "D-pad down/up to move the servo, use the left trigger to slow the "
                        + "step, and press Stop when the position is correct. Keep the mechanism clear.");

        double groundPosition = runOpMode(new DeployerPositionOpMode(
                DEPLOYER_NAME,
                "Ground",
                Constants.Intake.Deployer.GROUND_POSITION + Constants.Intake.Deployer.TRANSLATION_FACTOR));
        double deployedPosition = runOpMode(new DeployerPositionOpMode(
                DEPLOYER_NAME,
                "Deployed",
                Constants.Intake.Deployer.DEPLOYED_POSITION + Constants.Intake.Deployer.TRANSLATION_FACTOR));
        double stowedPosition = runOpMode(new DeployerPositionOpMode(
                DEPLOYER_NAME,
                "Stowed",
                Constants.Intake.Deployer.STOWED_POSITION + Constants.Intake.Deployer.TRANSLATION_FACTOR));

        result("groundPosition", groundPosition);
        result("deployedPosition", deployedPosition);
        result("stowedPosition", stowedPosition);
        result("translationFactor", 0.0);
        code(Language.JAVA,
                "const val GROUND_POSITION = " + groundPosition + "\n"
                        + "const val TRANSLATION_FACTOR = 0.0\n"
                        + "const val DEPLOYED_POSITION = " + deployedPosition + "\n"
                        + "const val STOWED_POSITION = " + stowedPosition + ";");
    }
}

class DeployerPositionOpMode extends TuningOpMode<Double> {
    private final String servoName;
    private final String positionName;
    private final double startPosition;

    DeployerPositionOpMode(String servoName, String positionName, double startPosition) {
        super(
                "Deployer " + positionName + " Position",
                "Hold D-pad down/up to adjust the " + positionName.toLowerCase()
                        + " position. Left trigger slows movement; press Stop to save.",
                true);
        this.servoName = servoName;
        this.positionName = positionName;
        this.startPosition = startPosition;
    }

    @Override
    protected Double runTuningOpMode() throws InterruptedException {
        Servo deployer = hardwareMap.get(Servo.class, servoName);
        double position = startPosition;
        deployer.setPosition(position);

        waitForStart();
        while (!isStopRequested()) {
            position = DeployerTuningControls.adjust(
                    position,
                    gamepad1.dpad_up,
                    gamepad1.dpad_down,
                    gamepad1.left_trigger);
            deployer.setPosition(position);

            telemetry.addData(positionName + " position", position);
            telemetry.addData("Step", DeployerTuningControls.step(gamepad1.left_trigger));
            telemetry.addLine("D-pad down/up: move | Left trigger: slow | Stop: save");
            telemetry.update();
            sleep(DeployerTuningControls.ADJUSTMENT_INTERVAL_MILLIS);
        }
        return position;
    }
}
