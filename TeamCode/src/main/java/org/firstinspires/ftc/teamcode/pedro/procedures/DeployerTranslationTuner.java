package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.TuningOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;

public class DeployerTranslationTuner extends Procedure {
    private static final String DEPLOYER_NAME = "deployer";
    private static final double START_POSITION = 0.2;

    public DeployerTranslationTuner() {
        super("Deployer Translation Tuner",
                "Recalibrates the intake deployer after its linkage moves.");
    }

    @Override
    public void run() throws InterruptedException {
        confirmation(
                "Deployer translation tuning",
                "Support the robot so the deployer can move safely. The servo has no physical "
                        + "position feedback, so you must identify contact with the ground. "
                        + "Hold D-pad down/up to adjust, hold the left trigger to slow the step, "
                        + "and press A as soon as the deployer reaches the ground.");

        Double groundPosition = runOpMode(
                new DeployerGroundPositionOpMode(DEPLOYER_NAME, START_POSITION));
        if (groundPosition == null) {
            abort("Ground position was not confirmed. Run the tuner again and press A at ground contact.");
            return;
        }

        double translationFactor = DeployerTuningControls.translationFactor(
                groundPosition, Intake.DEPLOYER_GROUND_POSITION);

        result("groundPosition", groundPosition);
        result("translationFactor", translationFactor);
        code(Language.JAVA,
                "public static double DEPLOYER_TRANSLATION_FACTOR = "
                        + translationFactor + ";");
    }
}

class DeployerGroundPositionOpMode extends TuningOpMode<Double> {
    private final String servoName;
    private final double startPosition;

    DeployerGroundPositionOpMode(String servoName, double startPosition) {
        super(
                "Deployer Ground Position",
                "Hold D-pad down/up to move. Left trigger slows movement; press A at ground contact.",
                true);
        this.servoName = servoName;
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
            position = Math.min(startPosition, position);
            deployer.setPosition(position);

            telemetry.addData("Commanded position", position);
            telemetry.addData("Step", DeployerTuningControls.step(gamepad1.left_trigger));
            telemetry.addLine("D-pad down/up: move | Left trigger: slow | A: confirm ground");
            telemetry.update();

            if (gamepad1.a) {
                return position;
            }
            sleep(DeployerTuningControls.ADJUSTMENT_INTERVAL_MILLIS);
        }
        return null;
    }
}
