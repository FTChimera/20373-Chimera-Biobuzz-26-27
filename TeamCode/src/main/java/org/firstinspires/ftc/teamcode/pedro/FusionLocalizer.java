//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by Fernflower decompiler)
//

package org.firstinspires.ftc.teamcode.pedro;

import static org.firstinspires.ftc.teamcode.util.math.MathUtils.EPSILON;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.math.Vector;
import com.pedropathing.math.Velocity;
import com.pedropathing.utils.Angle;
import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

public class FusionLocalizer implements Localizer {
    private final Localizer deadReckoning;
    private Pose currentRawPose;
    private MotionState motionState;
    private Pose currentRelativeTransform;
    private Matrix P;
    private final Matrix Q;
    private final Matrix R;
    private long lastUpdateTime = -1L;
    private final NavigableMap<Long, KalmanState> history = new TreeMap();
    private final int bufferSize;

    public FusionLocalizer(Localizer deadReckoning, Pose initialCovariance, Pose processVariance, Pose measurementVariance, int bufferSize) {
        this.deadReckoning = deadReckoning;
        this.motionState = MotionState.zero();
        this.currentRawPose = Pose.zero();
        this.P = Matrix.diag(new double[]{Math.max(initialCovariance.x(), EPSILON), Math.max(initialCovariance.y(), EPSILON), Math.max(initialCovariance.heading(), EPSILON)});
        this.Q = Matrix.diag(new double[]{Math.max(processVariance.x(), EPSILON), Math.max(processVariance.y(), EPSILON), Math.max(processVariance.heading(), EPSILON)});
        this.R = Matrix.diag(new double[]{Math.max(measurementVariance.x(), EPSILON), Math.max(measurementVariance.y(), EPSILON), Math.max(measurementVariance.heading(), EPSILON)});
        this.bufferSize = bufferSize;
        this.history.put(0L, new KalmanState(Pose.zero(), Velocity.zero(), this.currentRawPose, this.P));
    }

    public MotionState state() {
        return this.motionState;
    }

    public void update() {
        this.deadReckoning.update();
        long now = System.nanoTime();
        double dt = this.lastUpdateTime < 0L ? (double)0.0F : (double)(now - this.lastUpdateTime) / (double)1.0E9F;
        this.lastUpdateTime = now;
        Velocity currentVelocity = this.deadReckoning.velocity();
        Pose rawPose = this.deadReckoning.pose();
        this.currentRelativeTransform = this.currentRawPose.invert().compose(rawPose);
        this.P = this.updateCovariance(this.P, this.motionState.pose(), currentVelocity, dt);
        Pose currentPosition = this.motionState.pose().compose(this.currentRelativeTransform);
        this.currentRawPose = rawPose;
        this.motionState = MotionState.ofVelocity(currentPosition, currentVelocity);
        this.history.put(now, new KalmanState(currentPosition, currentVelocity, this.currentRelativeTransform, this.P));
        if (this.history.size() > this.bufferSize) {
            this.history.pollFirstEntry();
        }

    }

    public void reset() {
        this.deadReckoning.reset();
    }

    private Matrix updateCovariance(Matrix P, Pose pose, Velocity velocity, double dt) {
        Twist twist = velocity.toTwist(pose.heading());
        Vector dist = twist.toVector().abs().times(dt);
        Vector Q_diag = new Vector(this.Q.getDiagonal());
        Matrix bodyQ = Matrix.diag(dist.hadamard(Q_diag));
        Matrix rotation = Matrix.rotationTransform(pose.heading());
        Matrix worldQ = rotation.times(bodyQ).times(rotation.transpose());
        P = P.plus(worldQ).clampDiagonals(EPSILON);
        return P;
    }

    private KalmanState getKalmanState() {
        return new KalmanState(this.motionState.pose(), this.motionState.velocity(), this.currentRelativeTransform, this.P);
    }

    public void addMeasurement(Pose measuredPose, long timestamp) {
        this.addMeasurement(measuredPose, timestamp, (Pose)null);
    }

    public void addMeasurement(Pose measuredPose, long timestamp, Pose measurementVariance) {
        Matrix measurementR = measurementVariance == null ? this.R : Matrix.diag(new double[]{measurementVariance.x(), measurementVariance.y(), measurementVariance.heading()}).clampDiagonals(EPSILON);
        if (!this.history.isEmpty() && timestamp >= (Long)this.history.firstKey() && timestamp <= (Long)this.history.lastKey()) {
            KalmanState interpolatedData = this.interpolate(timestamp);
            if (interpolatedData == null) {
                interpolatedData = this.getKalmanState();
            }

            Pose pastPose = interpolatedData.pose;
            if (pastPose == null) {
                pastPose = this.pose();
            }

            boolean measX = !Double.isNaN(measuredPose.x());
            boolean measY = !Double.isNaN(measuredPose.y());
            boolean measH = !Double.isNaN(measuredPose.heading());
            Vector innovation = new Vector(new double[]{measX ? measuredPose.x() - pastPose.x() : (double)0.0F, measY ? measuredPose.y() - pastPose.y() : (double)0.0F, measH ? Angle.normalizeSigned(measuredPose.heading() - pastPose.heading()) : (double)0.0F});
            Matrix M = Matrix.diag(new double[]{measX ? (double)1.0F : (double)0.0F, measY ? (double)1.0F : (double)0.0F, measH ? (double)1.0F : (double)0.0F});
            Matrix Pm = interpolatedData.covariance;
            Matrix S = Pm.plus(measurementR);
            Matrix S_inv = S.invert();
            if (S_inv != null) {
                Matrix K = Pm.times(S_inv);
                if (interpolatedData.upperKey != null) {
                    KalmanState upperEntry = (KalmanState)this.history.get(interpolatedData.upperKey);
                    this.history.put(interpolatedData.upperKey, new KalmanState(upperEntry.pose, upperEntry.velocity, interpolatedData.remainderTransform, upperEntry.covariance));
                }

                K = M.times(K);
                innovation = M.times(innovation);
                Vector Ky = K.times(innovation);
                Pose updatedPast = new Pose(pastPose.x() + Ky.get(0), pastPose.y() + Ky.get(1), Angle.normalize(pastPose.heading() + Ky.get(2)));
                Matrix I = Matrix.identity(3);
                Matrix IK = I.minus(K);
                Matrix cov = IK.times(Pm).times(IK.transpose()).plus(K.times(measurementR).times(K.transpose())).clampDiagonals(EPSILON);
                this.history.put(timestamp, new KalmanState(updatedPast, interpolatedData.velocity, interpolatedData.relativeTransform, cov));
                long prevTime = timestamp;
                Pose prevPose = updatedPast;

                for(Map.Entry<Long, KalmanState> entry : this.history.tailMap(timestamp, false).entrySet()) {
                    long t = (Long)entry.getKey();
                    Velocity velocity = ((KalmanState)entry.getValue()).velocity;
                    if (velocity == null) {
                        velocity = this.velocity();
                    }

                    double dt = (double)(t - prevTime) / (double)1.0E9F;
                    Pose relativeTransform = ((KalmanState)entry.getValue()).relativeTransform;
                    cov = this.updateCovariance(cov, prevPose, velocity, dt);
                    prevPose = prevPose.compose(relativeTransform);
                    this.history.put(t, new KalmanState(prevPose, velocity, ((KalmanState)entry.getValue()).relativeTransform, cov));
                    prevTime = t;
                }

                this.motionState = MotionState.ofVelocity(((KalmanState)this.history.lastEntry().getValue()).pose, this.motionState.velocity());
                this.P = ((KalmanState)this.history.lastEntry().getValue()).covariance;
            }
        }
    }

    private KalmanState interpolate(long timestamp) {
        Long lowerKey = (Long)this.history.floorKey(timestamp);
        Long upperKey = (Long)this.history.ceilingKey(timestamp);
        if (lowerKey != null && upperKey != null) {
            if (lowerKey.equals(upperKey)) {
                return (KalmanState)this.history.get(lowerKey);
            } else {
                KalmanState lower = (KalmanState)this.history.get(lowerKey);
                KalmanState upper = (KalmanState)this.history.get(upperKey);
                double ratio = (double)(timestamp - lowerKey) / (double)(upperKey - lowerKey);
                Velocity lowerVel = lower.velocity;
                Velocity upperVel = upper.velocity;
                Velocity interpolVel = lowerVel.plus(upperVel.minus(lowerVel).scale(ratio));
                Pose interpolPose = Pose.interpolate(lower.pose, upper.pose, ratio);
                Pose toMeasurement = interpolateTransform(Pose.zero(), upper.relativeTransform, ratio);
                Pose remainder = toMeasurement.invert().compose(upper.relativeTransform);
                KalmanState result = new KalmanState(interpolPose, interpolVel, toMeasurement, lower.covariance);
                result.remainderTransform = remainder;
                result.upperKey = upperKey;
                return result;
            }
        } else {
            return null;
        }
    }

    public static Pose interpolateTransform(Pose a, Pose b, double ratio) {
        Twist delta = Twist.riemannianLog(a, b).times(ratio);
        return a.compose(Pose.zero().exp(delta));
    }

    public void setStartPose(Pose setStart) {
        this.deadReckoning.setPose(setStart);
        this.history.put(0L, new KalmanState(setStart, Velocity.zero(), setStart, this.P));
        this.motionState = MotionState.ofVelocity(setStart, this.motionState.velocity());
        this.currentRawPose = setStart;
    }

    public void setPose(Pose setPose) {
        this.motionState = MotionState.ofVelocity(setPose, this.motionState.velocity());
        this.deadReckoning.setPose(setPose);
        this.currentRawPose = setPose;
        if (!this.history.isEmpty()) {
            ((KalmanState)this.history.lastEntry().getValue()).pose = setPose;
        } else {
            this.setStartPose(setPose);
        }

    }

    private static class KalmanState {
        Pose pose;
        Velocity velocity;
        Pose relativeTransform;
        Matrix covariance;
        Pose remainderTransform;
        Long upperKey;

        public KalmanState(Pose pose, Velocity velocity, Pose relativeTransform, Matrix covariance) {
            this.pose = pose;
            this.velocity = velocity;
            this.relativeTransform = relativeTransform;
            this.covariance = covariance;
        }
    }
}
