package org.firstinspires.ftc.teamcode.robotDev.sim;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.rmsftc.utils.AllianceColor;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.planners.Constants;

@TeleOp(name = "Sim #6 Intake + Transfer Sequence", group = "Sim diagnostics")
public class SimIntakeTransferTest extends OpMode {
    private Intake intake;
    private Transfer transfer;
    private DcMotorEx intakeMotor, transferMotor;
    private Servo deployer;
    private IntakeSequenceMetrics metrics;
    private IntakeSequenceMetrics.Phase phase;
    private boolean started, done;
    private long startNanos;
    private double elapsed, intakePower, transferPower, servoPosition, intakeVelocity, transferVelocity;
    private int intakeEncoder, transferEncoder, initialIntakeEncoder, initialTransferEncoder;
    private String intakeMode = "IDLE", transferMode = "IDLE";
    private String result = "INIT: STOP/STOW; START runs 5.5 s empty sequence";

    @Override public void init() {
        boolean success = false;
        try {
            intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
            transferMotor = hardwareMap.get(DcMotorEx.class, "transfer");
            deployer = hardwareMap.get(Servo.class, "deployer");
            metrics = new IntakeSequenceMetrics(Constants.Intake.FULL_POWER, Constants.Transfer.SLOW_POWER,
                    Constants.Intake.Deployer.DEPLOYED_POSITION + Constants.Intake.Deployer.TRANSLATION_FACTOR,
                    Constants.Intake.Deployer.STOWED_POSITION + Constants.Intake.Deployer.TRANSLATION_FACTOR);
            intake = new Intake(hardwareMap, AllianceColor.RED);
            transfer = new Transfer(hardwareMap);
            shutdown();
            snapshot();
            if (!IntakeSequenceMetrics.finite(intakePower, transferPower, servoPosition,
                    intakeVelocity, transferVelocity)
                    || !IntakeSequenceMetrics.matches(intakePower, 0, IntakeSequenceMetrics.POWER_TOLERANCE)
                    || !IntakeSequenceMetrics.matches(transferPower, 0, IntakeSequenceMetrics.POWER_TOLERANCE)
                    || !IntakeSequenceMetrics.matches(servoPosition,
                    Constants.Intake.Deployer.STOWED_POSITION + Constants.Intake.Deployer.TRANSLATION_FACTOR,
                    IntakeSequenceMetrics.SERVO_TOLERANCE)) {
                throw new IllegalStateException("INIT observed outputs are not finite STOP/STOW");
            }
            initialIntakeEncoder = intakeEncoder;
            initialTransferEncoder = transferEncoder;
            display();
            success = true;
        } finally {
            if (!success) finish("FAIL: INIT exception (propagated)");
        }
    }

    @Override public void init_loop() { display(); }

    @Override public void start() {
        if (done) return;
        startNanos = System.nanoTime();
        started = true;
        result = "RUNNING";
    }

    @Override public void loop() {
        if (done || !started) { display(); return; }
        boolean success = false;
        try {
            elapsed = (System.nanoTime() - startNanos) / 1e9;
            IntakeSequenceMetrics.Phase next = IntakeSequenceMetrics.phaseAt(elapsed);
            if (phase != next) {
                phase = next;
                intake.setDeployed(phase.deployed());
                if (phase.intakeOn()) intake.setFullPower(); else intake.stop();
                if (phase == IntakeSequenceMetrics.Phase.TRANSFER) transfer.slow(); else transfer.stop();
            }
            intake.update();
            transfer.update();
            snapshot();
            metrics.sample(elapsed, intakePower, transferPower, servoPosition,
                    intakeVelocity, transferVelocity, intakeEncoder, transferEncoder);
            if (metrics.failure != null) finish("FAIL: " + metrics.failure);
            else if (metrics.passed) finish("PASS: complete sampled sequence; both motors responded");
            display();
            success = true;
        } finally {
            if (!success) finish("FAIL: loop exception (propagated); last complete snapshot");
        }
    }

    private void snapshot() {
        // Commit the numeric snapshot only after every read succeeds.
        double ip = intakeMotor.getPower(), tp = transferMotor.getPower(), sp = deployer.getPosition();
        double iv = intakeMotor.getVelocity(), tv = transferMotor.getVelocity();
        int ie = intakeMotor.getCurrentPosition(), te = transferMotor.getCurrentPosition();
        intakePower = ip; transferPower = tp; servoPosition = sp;
        intakeVelocity = iv; transferVelocity = tv; intakeEncoder = ie; transferEncoder = te;
        intakeMode = intake == null ? "unavailable" : intake.mode().name();
        transferMode = transfer == null ? "unavailable" : transfer.mode().name();
    }

    private void finish(String status) {
        result = status;
        done = true;
        shutdown();
    }

    private void shutdown() {
        // Observation handles stay read-only; attempt all mechanism outputs even on exceptions.
        try {
            if (intake != null) intake.stopImmediately();
        } finally {
            try {
                if (transfer != null) transfer.stopImmediately();
            } finally {
                if (intake != null) intake.setDeployed(false);
            }
        }
    }

    private void display() {
        telemetry.addData("Result", result);
        telemetry.addData("Outputs", done ? "OFF / STOW requested; final numbers frozen" : started ? "SEQUENCE ACTIVE" : "OFF / STOW");
        telemetry.addData("Elapsed / total s", "%.3f / %.3f", elapsed, IntakeSequenceMetrics.TOTAL_S);
        telemetry.addData("Phase / phase elapsed s", "%s / %.3f", phase, phase == null ? 0 : elapsed - phase.start);
        telemetry.addData("Commanded intake / transfer modes (snapshot)", "%s / %s", intakeMode, transferMode);
        telemetry.addData("Expected intake / transfer / SDK servo", "%.5f / %.5f / %.5f",
                metrics == null || phase == null ? 0 : metrics.expectedIntake(phase),
                metrics == null || phase == null ? 0 : metrics.expectedTransfer(phase),
                Constants.Intake.Deployer.TRANSLATION_FACTOR + (phase != null && phase.deployed()
                        ? Constants.Intake.Deployer.DEPLOYED_POSITION : Constants.Intake.Deployer.STOWED_POSITION));
        telemetry.addData("Observed applied intake / transfer power", "%.5f / %.5f", intakePower, transferPower);
        telemetry.addData("Servo SDK commanded position (NOT physical feedback)", "%.5f", servoPosition);
        telemetry.addData("Intake / transfer velocity TPS", "%.2f / %.2f", intakeVelocity, transferVelocity);
        telemetry.addData("Encoder intake / transfer; delta from INIT", "%d / %d; %d / %d",
                intakeEncoder, transferEncoder, (long) intakeEncoder - initialIntakeEncoder, (long) transferEncoder - initialTransferEncoder);
        telemetry.addLine("Phases [s]: STOP/STOW [0,.5), DEPLOY [.5,1), INTAKE [1,3), TRANSFER [3,5), STOP [5,5.5].");
        telemetry.addLine("Grace 0.15 s/phase; max tick gap 0.10 s; inclusive power/servo tolerance 0.005.");
        telemetry.addLine("Checked spans >=0.25/1.75 s; samples >=4/10 (idle/active); motion >=1 tick OR |velocity|>=5 TPS.");
        if (metrics != null) {
            telemetry.addData("Measured intake / transfer motion", "%s / %s", metrics.intakeMoved(), metrics.transferMoved());
            for (IntakeSequenceMetrics.Phase p : IntakeSequenceMetrics.Phase.values()) {
                int i = p.ordinal();
                telemetry.addData(p.name() + " checked n/span; max |I/T/servo error|", "%d / %.3f; %.5f / %.5f / %.5f",
                        metrics.samples[i], metrics.span(p), metrics.maxIntakeError[i], metrics.maxTransferError[i], metrics.maxServoError[i]);
            }
        }
        telemetry.addLine("No balls, scoring, stopping-velocity assertion, or physical ramp proof. No Robot/follower/Dashboard.");
        telemetry.update();
    }

    @Override public void stop() {
        if (done) { shutdown(); display(); return; }
        try {
            if (started) elapsed = (System.nanoTime() - startNanos) / 1e9;
            if (intakeMotor != null && transferMotor != null && deployer != null) snapshot();
        } finally {
            finish("ABORTED STOP: sequence incomplete (not PASS); outputs OFF / STOW");
        }
        display();
    }
}
