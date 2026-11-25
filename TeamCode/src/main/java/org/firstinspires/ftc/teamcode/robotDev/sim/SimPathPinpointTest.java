package org.firstinspires.ftc.teamcode.robotDev.sim;

import com.pedropathing.api.Paths;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathSegment;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.rmsftc.utils.AllianceColor;
import com.rmsftc.utils.RobotMode;
import com.rmsftc.utils.math.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.planners.CommonKt;
import org.firstinspires.ftc.teamcode.planners.Constants;
import org.firstinspires.ftc.teamcode.util.game.Field;
import org.firstinspires.ftc.teamcode.util.poses.PoseFactory;

import java.util.Arrays;

@Autonomous(name = "Sim #1 Path + Pinpoint (RUN FIRST)", group = "Sim diagnostics")
public class SimPathPinpointTest extends OpMode {
    protected Robot robot;
    protected Path path;
    protected Pose endpoint;
    private final DcMotor[] motors = new DcMotor[4];
    private GoBildaPinpointDriver pinpoint;
    private DriveDiagnosticProgress progress;
    private PinpointLocalizer.ResetMode previousReset;
    private boolean previousBrake, configurationSaved, outputsStopped, pathStarted, observedProgress;
    private long runId, startedNanos;
    private String route = "", failure;
    private String liveTelemetry = "Waiting for first live baseline";

    @Override
    public void init() {
        progress = new DriveDiagnosticProgress(timeoutSeconds());
        previousReset = org.firstinspires.ftc.teamcode.pedro.Constants.PINPOINT_CONFIG.resetMode.get();
        previousBrake = org.firstinspires.ftc.teamcode.pedro.Constants.FOLLOWER_BRAKE_AT_END.get();
        configurationSaved = true;
        boolean initialized = false;
        try {
            CommonKt.init(AllianceColor.RED);
            Pose shared = Constants.StartPoses.gardenArea;
            Pose start = PoseFactory.of(shared.x(), shared.y(), shared.heading());
            robot = new Robot(telemetry, hardwareMap, AllianceColor.RED, RobotMode.AUTONOMOUS, start);
            robot.smartIntakeControl = false;
            robot.smartShooterControl = false;
            robot.setTeleOpDrive = false;
            robot.debug = false;
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,
                    org.firstinspires.ftc.teamcode.pedro.Constants.PINPOINT_CONFIG.name.get());
            motors[0] = hardwareMap.get(DcMotor.class, org.firstinspires.ftc.teamcode.pedro.Constants.MECANUM_CONFIG.frontLeftName.get());
            motors[1] = hardwareMap.get(DcMotor.class, org.firstinspires.ftc.teamcode.pedro.Constants.MECANUM_CONFIG.backLeftName.get());
            motors[2] = hardwareMap.get(DcMotor.class, org.firstinspires.ftc.teamcode.pedro.Constants.MECANUM_CONFIG.frontRightName.get());
            motors[3] = hardwareMap.get(DcMotor.class, org.firstinspires.ftc.teamcode.pedro.Constants.MECANUM_CONFIG.backRightName.get());
            try {
                path = buildPath(start);
            } catch (Field.NoRouteException noRoute) {
                failure = "FAIL: no route: " + noRoute.getMessage();
            }
            if (failure == null) {
                endpoint = PoseFactory.of(path.endPose());
                StringBuilder exactRoute = new StringBuilder();
                for (PathSegment segment : path.getSegments()) {
                    com.pedropathing.math.Pose from = segment.get(0), to = segment.endPose();
                    if (!DriveDiagnosticProgress.finite(from.x(), from.y(), from.heading(),
                            to.x(), to.y(), to.heading())) {
                        failure = "FAIL: nonfinite route waypoint";
                    }
                    if (exactRoute.length() == 0) exactRoute.append(format(segment.get(0)));
                    exactRoute.append(" -> ").append(format(segment.endPose()));
                }
                route = exactRoute.toString();
                if (!DriveDiagnosticProgress.finite(endpoint.x(), endpoint.y(), endpoint.heading())) {
                    failure = "FAIL: nonfinite path endpoint";
                }
            }
            initialized = true;
        } finally {
            if (!initialized || failure != null) stopOutputs();
        }
        display();
    }

    protected Path buildPath(Pose start) {
        Pose shared = Constants.ShootPoses.garden;
        Pose shoot = PoseFactory.of(shared.x(), shared.y(), shared.heading());
        return Paths.line(start, shoot).linear(start, shoot);
    }

    protected double timeoutSeconds() {
        return 20;
    }

    protected String routeDescription() {
        return "ONE Pedro line: shared RED garden start -> shared RED garden shoot";
    }

    protected boolean atDestination(Pose measured) {
        return DriveDiagnosticProgress.atEndpoint(measured.x(), measured.y(), measured.heading(),
                endpoint.x(), endpoint.y(), endpoint.heading());
    }

    @Override
    public void init_loop() {
        display();
    }

    @Override
    public void start() {
        if (failure != null) return;
        boolean started = false;
        try {
            robot.start();
            startedNanos = System.nanoTime();
            started = true;
        } finally {
            if (!started) stopOutputs();
        }
    }

    @Override
    public void loop() {
        if (failure != null || progress.finished()) {
            display();
            return;
        }
        double elapsed = (System.nanoTime() - startedNanos) / 1e9;
        if (elapsed >= timeoutSeconds()) {
            failure = progress.wheelMovement < DriveDiagnosticProgress.WHEEL_TICKS_REQUIRED
                    ? "FAIL: timeout; no independently observed drivetrain movement"
                    : "FAIL: timeout; path did not complete";
            stopOutputs();
            display();
            return;
        }
        boolean tickSucceeded = false;
        try {
            // Robot owns the only follower/localizer update. Raw getters below do not refresh hardware.
            robot.update();
            Pose measured = robot.getPose();
            double rawX = pinpoint.getPosX(DistanceUnit.INCH);
            double rawY = pinpoint.getPosY(DistanceUnit.INCH);
            double rawHeading = pinpoint.getHeading(AngleUnit.RADIANS);
            double[] ticks = new double[4], powers = new double[4];
            for (int i = 0; i < motors.length; i++) {
                ticks[i] = motors[i].getCurrentPosition();
                powers[i] = motors[i].getPower();
            }
            elapsed = (System.nanoTime() - startedNanos) / 1e9;
            observedProgress |= pathStarted && robot.follower.hasPathProgress();
            boolean trackerDone = robot.follower.pathTracker != null && robot.follower.pathTracker.done();
            boolean completed = pathStarted && observedProgress
                    && (trackerDone || !robot.follower.following());
            com.pedropathing.math.Pose target = robot.follower.following() && observedProgress
                    ? robot.follower.closestPose() : endpoint;
            if (target == null || !DriveDiagnosticProgress.finite(target.x(), target.y(), target.heading())) {
                failure = "FAIL: nonfinite path target";
            } else if (pathStarted && robot.follower.pathRunId() != runId) {
                failure = "FAIL: diagnostic path replaced";
            } else {
                progress.sample(elapsed, measured.x(), measured.y(), measured.heading(),
                        rawX, rawY, rawHeading, ticks, powers, robot.follower.completion(),
                        pathStarted && observedProgress, completed,
                        DriveDiagnosticProgress.finite(measured.x(), measured.y(), measured.heading())
                                && atDestination(measured), false);
            }
            if (target == null) target = endpoint;
            liveTelemetry = "elapsed(s)=" + elapsed
                    + "\nFollower Pinpoint-derived localizer: " + format(measured)
                    + "\nRaw Pinpoint hardware (cached): " + format(PoseFactory.of(rawX, rawY, rawHeading))
                    + "\nActual Foresight closest path target: " + format(target)
                    + "\nTarget errors(in,rad): " + measured.distance(target) + ", "
                    + DriveDiagnosticProgress.headingError(measured.heading(), target.heading())
                    + "\nEndpoint errors(in,rad): " + measured.distance(endpoint) + ", "
                    + DriveDiagnosticProgress.headingError(measured.heading(), endpoint.heading())
                    + "\nMeasured destination criterion satisfied: "
                    + (DriveDiagnosticProgress.finite(measured.x(), measured.y(), measured.heading())
                    && atDestination(measured))
                    + "\nmode/busy/completion/end: " + robot.follower.mode() + "/"
                    + robot.follower.isBusy() + "/" + robot.follower.completion() + "/" + trackerDone
                    + "\nDrive FL,BL,FR,BR power: " + Arrays.toString(powers)
                    + "\nDrive FL,BL,FR,BR encoder ticks: " + Arrays.toString(ticks)
                    + "\nMovement from FIRST LIVE baseline (localizer/raw in, wheel ticks): "
                    + progress.poseMovement + "/" + progress.pinpointMovement + "/" + progress.wheelMovement;
            if (failure != null || progress.finished()) {
                stopOutputs();
            } else if (!pathStarted) {
                // The first Robot.update() supplies the baseline BEFORE any commanded path motion.
                robot.follower.follow(path);
                runId = robot.follower.pathRunId();
                pathStarted = true;
            }
            tickSucceeded = true;
        } finally {
            if (!tickSucceeded) {
                failure = "FAIL: drive tick exception (see Driver Station error)";
                stopOutputs();
            }
        }
        display();
    }

    private void display() {
        telemetry.addData("Result", failure != null ? failure : progress.result());
        telemetry.addData("Route", routeDescription());
        telemetry.addData("EXACT waypoints (in,rad)", route);
        telemetry.addData("Limits", "position <= 2 in, heading <= 8 deg (#1); raw/localizer movement >= .5 in; "
                + "wheels >= 12 ticks; raw stall < .15 in for 2 s with wheel movement; timeout " + timeoutSeconds() + " s");
        telemetry.addData("Ground truth", "NOT AVAILABLE. Wheel encoders prove wheel motion, not physical chassis motion.");
        telemetry.addData("Live/final snapshot", liveTelemetry);
        telemetry.addData("Outputs", outputsStopped ? "STOPPED; final snapshot retained until STOP" : "enabled at START");
        telemetry.update();
    }

    private static String format(com.pedropathing.math.Pose pose) {
        return "(" + pose.x() + ", " + pose.y() + ", " + pose.heading() + ")";
    }

    private void stopOutputs() {
        if (outputsStopped) return;
        try {
            if (robot != null) robot.stopEverything();
        } finally {
            try {
                // Do not let an unrelated subsystem's stop failure leave the diagnostic drive enabled.
                if (robot != null) robot.follower.stop();
            } finally {
                if (configurationSaved) {
                    org.firstinspires.ftc.teamcode.pedro.Constants.PINPOINT_CONFIG.resetMode.set(previousReset);
                    org.firstinspires.ftc.teamcode.pedro.Constants.FOLLOWER_BRAKE_AT_END.set(previousBrake);
                }
                outputsStopped = true;
            }
        }
    }

    @Override
    public void stop() {
        progress.cancel();
        stopOutputs();
        display();
    }
}
