//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by Fernflower decompiler)
//

package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.config.ConfigVar;
import com.pedropathing.config.Validator;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.FollowerLog;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.math.Vector2D;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathSegment;
import com.pedropathing.paths.PathTracker;
import com.pedropathing.paths.curves.Curve;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;

public class Follower {
    public final Localizer localizer;
    public final Drivetrain drivetrain;
    public final ConfigVar<Boolean> holdEnd = ConfigVar.of(true, new Validator[0]);
    public Algorithm algorithm;
    public PathTracker pathTracker = null;
    public Pose holdPose = null;
    public DrivePowers manualPowers = null;
    public Mode mode;
    public long previousNanoTime;
    public boolean useHoldScaling;
    public final List<Consumer<FollowerLog>> loggers;
    public FollowerLog debug;

    public Follower(com.pedropathing.follower.Follower follower) {
        this(follower.localizer, follower.drivetrain, follower.algorithm());
    }

    public Follower(Localizer localizer, Drivetrain drivetrain, Algorithm algorithm) {
        this.mode = Follower.Mode.IDLE;
        this.previousNanoTime = 0L;
        this.loggers = new ArrayList();
        this.localizer = localizer;
        this.algorithm = algorithm;
        this.drivetrain = drivetrain;
    }

    public Follower withLogger(Consumer<FollowerLog> logger) {
        this.loggers.add(logger);
        return this;
    }

    public Mode mode() {
        return this.mode;
    }

    public void update() {
        long nanoTime = System.nanoTime();
        double deltaTime = (double)0.0F;
        if (this.previousNanoTime != 0L) {
            deltaTime = (double)(nanoTime - this.previousNanoTime) / (double)1.0E9F;
        }

        this.update(deltaTime);
        this.previousNanoTime = nanoTime;
    }

    public void update(double deltaTime) {
        this.debug = null;
        this.localizer.update();
        switch (this.mode.ordinal()) {
            case 0:
                if (this.pathTracker.done()) {
                    if ((Boolean)this.holdEnd.get()) {
                        this.hold(this.pathTracker.endPose());
                    } else {
                        this.stop();
                    }

                    this.pathTracker = null;
                } else {
                    DrivePowers powers = this.algorithm.calculatePath(this.drivetrain, this.pathTracker, this.localizer.state(), deltaTime);
                    this.drivetrain.drive(powers, false);
                }
                break;
            case 1:
                DrivePowers powers = this.algorithm.calculateHold(this.drivetrain, this.holdPose, this.localizer.state(), this.useHoldScaling, deltaTime);
                this.drivetrain.drive(powers, false);
                break;
            case 2:
                this.drivetrain.drive(this.manualPowers, true);
                break;
            case 3:
                this.drivetrain.stop();
        }

        if (this.debug != null && !this.loggers.isEmpty()) {
            this.createDebug();
        }

        for(Consumer<FollowerLog> logger : this.loggers) {
            logger.accept(this.debug());
        }

    }

    public FollowerLog debug() {
        if (this.debug == null) {
            this.createDebug();
        }

        return this.debug;
    }

    public void createDebug() {
        Map<String, Object> map = new HashMap();
        map.put("mode", this.mode);
        if (this.mode == Follower.Mode.FOLLOW || this.mode == Follower.Mode.HOLD) {
            map.put("isBusy", this.isBusy());
            map.put("atParametricEnd", this.atParametricEnd());
            map.put("pathIndex", this.pathIndex());
        }

        this.debug = FollowerLog.of(map, this.localizer.debug(), this.drivetrain.debug(), this.algorithm.debug());
    }

    public void clearState() {
        if (this.pathTracker != null) {
            this.pathTracker.release();
        }

        this.pathTracker = null;
        this.holdPose = null;
        this.manualPowers = null;
        this.useHoldScaling = true;
    }

    public void follow(Path path) {
        this.clearState();
        this.mode = Follower.Mode.FOLLOW;
        this.pathTracker = new PathTracker(path);
        this.algorithm.reset();
    }

    public void hold(Pose pose) {
        this.hold(pose, false);
    }

    public void hold(Pose pose, boolean useScaling) {
        this.clearState();
        this.mode = Follower.Mode.HOLD;
        this.holdPose = pose;
        this.useHoldScaling = useScaling;
    }

    public void manual(DrivePowers powers) {
        this.clearState();
        this.mode = Follower.Mode.MANUAL;
        this.manualPowers = powers;
    }

    public void manualSafe(DrivePowers drivePowers, boolean fieldCentric) {
        if (following()) return;
        manual(drivePowers, fieldCentric);
    }

    public void manual(DrivePowers powers, boolean fieldCentric) {
        if (fieldCentric) {this.manual(ManualDrive.fieldCentric(powers, this.localizer.pose().heading()));}
        this.manual(powers);
    }

    public void driveOrHold(DrivePowers drivePowers, double inputThreshold, double velocityThreshold) {
        boolean inputActive = hasInput(drivePowers, inputThreshold);
        if ((manual() || idle()) && !inputActive && this.localizer.velocity().toVector2D().magnitude() < velocityThreshold) {
            hold(localizer.pose());
        } else if (inputActive || manual()) {
            manual(drivePowers);
        }
    }

    public void driveOrHold(DrivePowers drivePowers) {
        driveOrHold(drivePowers, 0.1, 0.2);
    }

    public boolean hasInput(DrivePowers powers, double threshold) {
        if (powers == null) {
            return false;
        } else {
            return Math.abs(powers.forward()) > threshold || Math.abs(powers.strafe()) > threshold || Math.abs(powers.turn()) > threshold;
        }
    }

    public void manual(double forward, double lateral, double heading) {
        this.manual(new DrivePowers(forward, lateral, heading));
    }

    public void manual(double forward, double lateral, double heading, boolean fieldCentric) {
        if (fieldCentric) {this.manual(ManualDrive.fieldCentric(new DrivePowers(forward, lateral, heading), this.localizer.pose().heading()));}
        this.manual(new DrivePowers(forward, lateral, heading));
    }

    public void stop() {
        this.clearState();
        this.mode = Follower.Mode.IDLE;
    }
    public double distanceToEndpoint() {
        return this.pathTracker == null ? (double)0.0F : this.currentPath().endPose().distance(this.localizer.pose());
    }

    public boolean following() {
        return this.mode == Follower.Mode.FOLLOW;
    }

    public boolean isBusy() {
        return !idle()&&!manual()&&this.algorithm.isBusy();
    }

    public boolean holding() {
        return this.mode == Follower.Mode.HOLD;
    }

    public boolean manual() {
        return this.mode == Follower.Mode.MANUAL;
    }

    public boolean idle() {
        return this.mode == Follower.Mode.IDLE;
    }


    public boolean atParametricEnd() {
        if (!this.mode.equals(Follower.Mode.FOLLOW)) {
            return true;
        } else if (this.pathTracker == null) {
            return true;
        } else {
            return this.pathTracker.remainingPaths() > 1 ? false : this.algorithm.atParametricEnd();
        }
    }

    public double completion() {
        return this.algorithm.completion();
    }

    public Vector2D closestTangent() {
        return this.algorithm.closestTangent();
    }

    public Vector2D closestNormal() {
        return this.algorithm.closestNormal();
    }

    public double curvature() {
        return this.algorithm.curvature();
    }

    public Pose closestPose() {
        return this.algorithm.closestPose();
    }

    public double parametricCompletion() {
        return this.algorithm.parametricCompletion();
    }

    public double remainingDistance() {
        return this.algorithm.remainingDistance();
    }

    public int pathIndex() {
        return this.pathTracker == null ? -1 : this.pathTracker.currentIndex();
    }

    public PathSegment currentSegment() {
        return this.pathTracker == null ? null : this.pathTracker.current();
    }

    public Curve currentCurve() {
        return this.pathTracker == null ? null : this.currentSegment().curve;
    }

    public Path currentPath() {
        return this.pathTracker == null ? null : this.pathTracker.path();
    }

    public double tangentialVelocity() {
        return this.localizer.velocity().toVector2D().dot(this.closestTangent());
    }

    public Pose poseAt(double completion) {
        if (this.pathTracker == null) {
            return this.holdPose == null ? this.localizer.pose() : this.holdPose;
        } else {
            return this.currentSegment().get(this.currentCurve().parameter(completion));
        }
    }

    public Vector2D tangentAt(double completion) {
        return this.pathTracker == null ? this.closestTangent() : this.currentCurve().tangent(this.currentCurve().parameter(completion));
    }

    public Vector2D normalAt(double completion) {
        return this.pathTracker == null ? this.closestNormal() : this.currentCurve().leftNormal(this.currentCurve().parameter(completion));
    }

    public double curvatureAt(double completion) {
        return this.pathTracker == null ? this.curvature() : this.currentCurve().curvature(this.currentCurve().parameter(completion));
    }

    public static enum Mode {
        FOLLOW,
        HOLD,
        MANUAL,
        IDLE;

        private Mode() {
        }
    }
}
