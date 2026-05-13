package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathPoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Blackboard;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.Timer;
import org.firstinspires.ftc.teamcode.library.hardware.webcam.Limelight;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;

import java.util.concurrent.TimeUnit;

// This is a sample OpMode, we don't need variants for this
@Autonomous()
public class PreseasonWebcamAuto extends OpMode {
    Follower follower;
    Intake intake;
    Limelight limelight;
    State state = State.TO_RELEASE_PRELOADS;
    Timer opModeTimer;
    int iteration = 0;
    long deltaTime = 0;
    static class Poses {
        static final Pose STARTING_POSE = new Pose(36, 12, Math.toRadians(180));
        static final Pose RELEASE_POSE = new Pose(36, 36, Math.toRadians(180));
        static final Pose PARK_POSE = new Pose(24, 12, Math.toRadians(180));
        static final double PERCENTAGE_TURN_BACK_TO_RELEASE = 0.7; // percentage of the path to turn back to release
        static Path PRELOAD() {
            Path preload = new Path(new BezierLine(STARTING_POSE, RELEASE_POSE));
            preload.setLinearHeadingInterpolation(STARTING_POSE.getHeading(), RELEASE_POSE.getHeading());
            return preload;
        }
        static Path generateIntakePath() {
            return null;
        }
        static Path PARK(double heading) {
            Path park = new Path(new BezierLine(RELEASE_POSE, PARK_POSE));
            park.setLinearHeadingInterpolation(heading, PARK_POSE.getHeading());
            return park;
        }
        static Path goPast(Pose pose) {
            double heading = pose.getHeading();
            Vector direction = new Vector(Math.cos(heading), Math.sin(heading));
            direction.setMagnitude(10); // go 10 inches past the current position
            Path goPast = new Path(new BezierLine(pose, pose.plus(new Pose(direction.getXComponent(), direction.getYComponent()))));
            goPast.setLinearHeadingInterpolation(heading, heading);
            return goPast;
        }
        static Path backToRelease(Pose pose) {
            double heading = pose.getHeading();
            Path backToRelease = new Path(new BezierLine(pose, RELEASE_POSE));
            backToRelease.setHeadingInterpolation(HeadingInterpolator.piecewise(
                    new HeadingInterpolator.PiecewiseNode(0, PERCENTAGE_TURN_BACK_TO_RELEASE, HeadingInterpolator.linear(heading, heading + Math.toRadians(180))),
                    new HeadingInterpolator.PiecewiseNode(PERCENTAGE_TURN_BACK_TO_RELEASE, 1, HeadingInterpolator.constant(heading))));
            return backToRelease;
        }
    }
    enum State {
        TO_RELEASE_PRELOADS,
        TO_RELEASE,
        RELEASING,
        TO_INTAKE,
        INTAKING,
        TO_PARK,
        IDLE;
        final int MAX_ITERATIONS = 5;
        Timer timer;
        State() {timer = new Timer();}
        public State next(int iteration) {
            if (this == TO_RELEASE_PRELOADS) return RELEASING;
            else if (this == RELEASING && iteration >= MAX_ITERATIONS) return TO_PARK;
            else return values()[(ordinal() + 1) % values().length];
        }
        public long getElapsedTime() {
            return timer.get(); // return in milliseconds
        }
         public void resetTimer() {
             timer.reset();
         }
    }
    /**
     * User-defined init method
     * <p>
     * This method will be called once, when the INIT button is pressed.
     */
    @Override
    public void init() {
        // Don't use the limelight fusion as we are going to use color detection pipelines.
        // We will fuse in loop
        follower = Constants.pinpointFollower(hardwareMap);
        follower.setStartingPose(Poses.STARTING_POSE);
        intake = new Intake(hardwareMap);
        limelight = new Limelight(hardwareMap, Constants.LIMELIGHT_CAMERA_NAME);
    }
    @Override
    public void start() {
        limelight.start(Limelight.Pipeline.COLOR);
        follower.followPath(Poses.PRELOAD());
        opModeTimer = new Timer();
    }
    /**
     * User-defined loop method
     * <p>
     * This method will be called repeatedly during the period between when
     * the play button is pressed and when the OpMode is stopped.
     */
    @Override
    public void loop() {
        follower.update();
        limelight.LLUpdate();
        deltaTime = opModeTimer.dt();
        // This is where we would normally fuse limelight pose estimates. Todo
        switch (state) {
            case TO_RELEASE_PRELOADS:
            case TO_RELEASE:
                if (!follower.isBusy()) {
                    state = state.next(iteration);
                    intake.ReverseFullPower();
                }
                break;
            case RELEASING:
                if (state.getElapsedTime() >= 1000) {
                    state = state.next(iteration);
                    intake.FullPower();
                    if (state != State.TO_PARK) follower.followPath(Poses.generateIntakePath());
                }
                break;
            case TO_INTAKE:
                if (!follower.isBusy()) {
                    state = state.next(iteration);
                    follower.followPath(Poses.goPast(follower.getPose()));
                }
                break;
            case INTAKING:
                if (!follower.isBusy()) {
                    state = state.next(iteration);
                    follower.followPath(Poses.backToRelease(follower.getPose()));
                }
                break;
            case TO_PARK:
                if (!follower.isBusy()) {
                    follower.followPath(Poses.PARK(follower.getHeading()));
                }
                break;
            case IDLE:
                 if (!follower.isBusy()) {
                     requestOpModeStop();
                 }
                 break;
        }
        telemetry.addData("Pose", follower.getPose());
        telemetry.addData("Path State", state);
        telemetry.addData("OpModeTimer", opModeTimer.get(TimeUnit.SECONDS));
        telemetry.addData("Delta Time", deltaTime);
        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
        follower.update();
        Blackboard.pose = follower.getPose();
    }
}
