package org.firstinspires.ftc.teamcode.library;

/** Utility Drive vector class with forward, strafe, rotate */
public class DriveVector {
    public static DriveVector IDLE = new DriveVector(0, 0, 0, 0);
    double forward, strafe, rotate, robotHeading;

    public DriveVector(double forward, double strafe, double rotate, double robotHeading) {
        this.forward = forward;
        this.strafe = strafe;
        this.rotate = rotate;
        this.robotHeading = robotHeading;

    }

    public DriveVector(double forward, double strafe, double rotate) {
        this(forward, strafe, rotate, 0); // Robot Centric
    }

    public DriveVector add(DriveVector other) {
        return new DriveVector(
                this.forward + other.forward,
                this.strafe + other.strafe,
                this.rotate + other.rotate,
                this.robotHeading // Keep the same robot heading
        );
    }
    public DriveVector scale(double scalar) {
        return new DriveVector(
                this.forward * scalar,
                this.strafe * scalar,
                this.rotate * scalar,
                this.robotHeading // Keep the same robot heading
        );
    }

    public DriveVector() {
        this(0, 0, 0, 0);
    }
    public DriveVector rotateFieldCentric() {
        // IF FIELD CENTRIC BREAKS, REPLACE ROBOT HEADING WITH NEGATIVE ROBOT HEADING
        double cosA = Math.cos(robotHeading);
        double sinA = Math.sin(robotHeading);
        double newForward = forward * cosA + strafe * sinA;
        double newStrafe = -forward * sinA + strafe * cosA;
        return new DriveVector(newForward, newStrafe, rotate, 0);
    }

    public void setRobotHeading(double heading) {this.robotHeading = heading;}

    public double[] rotateDrivePowers() {
        double[] motorPowers = new double[4];
        motorPowers[0] = forward + strafe + rotate; // Front Left
        motorPowers[1] = forward - strafe - rotate; // Front Right
        motorPowers[2] = forward - strafe + rotate; // Back Left
        motorPowers[3] = forward + strafe - rotate; // Back Right

        // Normalize motor powers if any exceed 1.0
        double maxPower = 0;
        for (double power : motorPowers) {
            maxPower = Math.max(maxPower, Math.abs(power));
        }
        if (maxPower > 1.0) {
            for (int i = 0; i < motorPowers.length; i++) {
                motorPowers[i] /= maxPower;
            }
        }

        return motorPowers;
    }

    public DriveVector copy() {
        return new DriveVector(forward, strafe, rotate, robotHeading);
    }
}
