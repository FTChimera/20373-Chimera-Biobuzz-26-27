package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.AtomicPath;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathTracker;
import com.pedropathing.paths.curves.Line;

import java.lang.reflect.Proxy;
import java.util.Collections;

/** Interface proxies keep lifecycle tests independent of Android hardware. */
public final class FollowerTestRig {
    public Pose pose = Pose.zero();
    public Velocity velocity = Velocity.zero();
    public double progress;
    public boolean busy;
    public boolean atEnd;
    public boolean finishSegment;
    public boolean holdChangesCompletion;
    public int resets;
    public int stops;
    public int holdCalls;
    public double pathHeading;

    public final Localizer localizer = proxy(Localizer.class, (method, args) -> {
        switch (method) {
            case "pose": return pose;
            case "velocity": return velocity;
            case "state": return MotionState.ofVelocity(pose, velocity);
            case "setPose": pose = (Pose) args[0]; return null;
            case "debug": return Collections.emptyMap();
            default: return null;
        }
    });

    public final Drivetrain drivetrain = proxy(Drivetrain.class, (method, args) -> {
        if (method.equals("stop")) stops++;
        if (method.equals("debug")) return Collections.emptyMap();
        return null;
    });

    public final Algorithm algorithm = proxy(Algorithm.class, (method, args) -> {
        switch (method) {
            case "reset":
                resets++;
                busy = true;
                progress = 1; // Foresight reports 1 until calculatePath binds its tracker.
                atEnd = false;
                return null;
            case "calculatePath":
                pathHeading = ((PathTracker) args[1]).current().heading(0.5);
                if (finishSegment) {
                    ((PathTracker) args[1]).advance();
                    progress = 1;
                    atEnd = false; // Foresight resets on advancing the last segment.
                }
                return DrivePowers.zero();
            case "calculateHold":
                holdCalls++;
                if (holdChangesCompletion) {
                    progress = 1;
                    atEnd = true;
                }
                return DrivePowers.zero();
            case "completion":
            case "parametricCompletion": return progress;
            case "atParametricEnd": return atEnd;
            case "isBusy": return busy;
            case "debug": return Collections.emptyMap();
            default: return null;
        }
    });

    public final Follower follower = new Follower(localizer, drivetrain, algorithm);

    public static Path path() {
        return new AtomicPath(new Line(Pose.zero(), new Pose(10, 0, 0))).constant(0);
    }

    private interface Call { Object invoke(String name, Object[] args); }

    private static <T> T proxy(Class<T> type, Call call) {
        return type.cast(Proxy.newProxyInstance(type.getClassLoader(), new Class<?>[]{type},
                (proxy, method, args) -> {
                    switch (method.getName()) {
                        case "hashCode": return System.identityHashCode(proxy);
                        case "equals": return proxy == args[0];
                        case "toString": return type.getSimpleName() + "Fake";
                        default: return call.invoke(method.getName(), args);
                    }
                }));
    }
}
