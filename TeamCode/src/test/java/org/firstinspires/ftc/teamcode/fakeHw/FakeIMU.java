package org.firstinspires.ftc.teamcode.fakeHw;

import com.qualcomm.robotcore.hardware.IMU;
import java.util.Objects;
import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.navigation.*;

/** Stimuli are already in robot coordinates; mounting parameters do not rotate them. */
public final class FakeIMU extends FakeSensorDevice implements IMU {
    private double yaw, pitch, roll, yawOffset;
    private float xRate, yRate, zRate;
    private long orientationTime, rateTime;
    private Parameters parameters;
    private int initializeCount, resetYawCount;

    @Override public boolean initialize(Parameters parameters) {
        requireOpen();
        Objects.requireNonNull(parameters, "parameters");
        Objects.requireNonNull(parameters.imuOrientationOnRobot, "imuOrientationOnRobot");
        this.parameters = parameters.copy();
        initializeCount++;
        return true;
    }
    public boolean isInitialized() { return parameters != null; }
    public Parameters getParameters() {
        requireOpen();
        if (parameters == null) throw new IllegalStateException("IMU not initialized");
        return parameters.copy();
    }
    public int getInitializeCount() { return initializeCount; }
    public int getResetYawCount() { return resetYawCount; }
    public void setOrientation(AngleUnit unit, double yaw, double pitch, double roll, long acquisitionTime) {
        requireOpen();
        Objects.requireNonNull(unit, "unit");
        finite(yaw, "yaw"); finite(pitch, "pitch"); finite(roll, "roll");
        timestamp(acquisitionTime);
        this.yaw = radians(unit, yaw);
        this.pitch = radians(unit, pitch);
        this.roll = radians(unit, roll);
        orientationTime = acquisitionTime;
    }
    public void setAngularVelocity(AngleUnit unit, double x, double y, double z, long acquisitionTime) {
        requireOpen();
        Objects.requireNonNull(unit, "unit");
        finite(x, "x rate"); finite(y, "y rate"); finite(z, "z rate");
        timestamp(acquisitionTime);
        float newX = rate(unit, x), newY = rate(unit, y), newZ = rate(unit, z);
        xRate = newX; yRate = newY; zRate = newZ;
        rateTime = acquisitionTime;
    }
    private static float rate(AngleUnit unit, double value) {
        float radians = (float) unit.getUnnormalized().toRadians(value);
        finite(radians, "rate in radians");
        finite((float) Math.toDegrees(radians), "rate in degrees");
        return radians;
    }
    private static double radians(AngleUnit unit, double value) {
        double period = unit == AngleUnit.DEGREES ? 360 : 2 * Math.PI;
        return AngleUnit.RADIANS.normalize(unit.toRadians(value % period));
    }
    private static void timestamp(long time) {
        if (time < 0) throw new IllegalArgumentException("acquisitionTime must be nonnegative");
    }
    @Override public void resetYaw() { requireOpen(); yawOffset = yaw; resetYawCount++; }
    @Override public YawPitchRollAngles getRobotYawPitchRollAngles() {
        requireOpen();
        return new YawPitchRollAngles(AngleUnit.RADIANS, AngleUnit.RADIANS.normalize(yaw - yawOffset), pitch, roll, orientationTime);
    }
    @Override public Quaternion getRobotOrientationAsQuaternion() {
        requireOpen();
        double cy = Math.cos((yaw - yawOffset) / 2), sy = Math.sin((yaw - yawOffset) / 2);
        double cp = Math.cos(pitch / 2), sp = Math.sin(pitch / 2);
        double cr = Math.cos(roll / 2), sr = Math.sin(roll / 2);
        // FTC yaw/pitch/roll is intrinsic Z-X-Y, not aerospace Z-Y-X.
        return new Quaternion((float) (cy * cp * cr - sy * sp * sr),
                (float) (cy * sp * cr - sy * cp * sr),
                (float) (cy * cp * sr + sy * sp * cr),
                (float) (cy * sp * sr + sy * cp * cr), orientationTime);
    }
    @Override public Orientation getRobotOrientation(AxesReference reference, AxesOrder order, AngleUnit unit) {
        requireOpen();
        Objects.requireNonNull(reference, "reference");
        Objects.requireNonNull(order, "order");
        Objects.requireNonNull(unit, "unit");
        Quaternion q = getRobotOrientationAsQuaternion();
        // GeneralMatrixF avoids Quaternion.toMatrix's Android OpenGL dependency on a plain JVM.
        float w = q.w, x = q.x, y = q.y, z = q.z;
        float[] data = new float[] {
                1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w),
                2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w),
                2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)
        };
        // Roundoff at gimbal lock must not send the SDK's asin/acos outside [-1,1].
        for (int i = 0; i < data.length; i++) data[i] = (float) clip(data[i], -1, 1);
        GeneralMatrixF matrix = new GeneralMatrixF(3, 3, data);
        Orientation result = Orientation.getOrientation(matrix, reference, order, unit);
        result.acquisitionTime = orientationTime;
        return result;
    }
    @Override public AngularVelocity getRobotAngularVelocity(AngleUnit unit) {
        requireOpen();
        Objects.requireNonNull(unit, "unit");
        return new AngularVelocity(AngleUnit.RADIANS, xRate, yRate, zRate, rateTime).toAngleUnit(unit);
    }
    @Override public void resetDeviceConfigurationForOpMode() {
        super.resetDeviceConfigurationForOpMode();
        parameters = null;
    }
}
