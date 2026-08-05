package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.library.DriveVector;
import org.firstinspires.ftc.teamcode.library.robot.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.library.hardware.actuators.Motor;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.Button;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.GpBindings;
import org.firstinspires.ftc.teamcode.library.robot.RobotConstants;

import java.util.function.DoubleUnaryOperator;
import java.util.function.Supplier;

public class RobotDrive extends Mechanism {
    public static class ROBOT_DRIVE_PARAMS implements Mechanism.Params<ROBOT_DRIVE_PARAMS> {
        public DoubleUnaryOperator DEFAULT_STICK_CURVE = DoubleUnaryOperator.identity();
        public boolean START_TELEOP_DRIVE_ON_INIT = true;
        public boolean RESET_STICK_CURVE_ON_INIT = true;
        public boolean UPDATE_FOLLOWER_ON_INIT = true;
        public double INITIAL_HEADING_OFFSET = 0;
        public Button ZERO_HEADING_BUTTON = Button.BACK;

        public ROBOT_DRIVE_PARAMS() {}

        private ROBOT_DRIVE_PARAMS(ROBOT_DRIVE_PARAMS other) {
            DEFAULT_STICK_CURVE = other.DEFAULT_STICK_CURVE;
            START_TELEOP_DRIVE_ON_INIT = other.START_TELEOP_DRIVE_ON_INIT;
            RESET_STICK_CURVE_ON_INIT = other.RESET_STICK_CURVE_ON_INIT;
            UPDATE_FOLLOWER_ON_INIT = other.UPDATE_FOLLOWER_ON_INIT;
            INITIAL_HEADING_OFFSET = other.INITIAL_HEADING_OFFSET;
            ZERO_HEADING_BUTTON = other.ZERO_HEADING_BUTTON;
        }

        public ROBOT_DRIVE_PARAMS defaultStickCurve(DoubleUnaryOperator defaultStickCurve) {
            this.DEFAULT_STICK_CURVE = defaultStickCurve == null ? DoubleUnaryOperator.identity() : defaultStickCurve;
            return this;
        }

        public ROBOT_DRIVE_PARAMS startTeleOpDriveOnInit(boolean startTeleOpDriveOnInit) {
            this.START_TELEOP_DRIVE_ON_INIT = startTeleOpDriveOnInit;
            return this;
        }

        public ROBOT_DRIVE_PARAMS resetStickCurveOnInit(boolean resetStickCurveOnInit) {
            this.RESET_STICK_CURVE_ON_INIT = resetStickCurveOnInit;
            return this;
        }

        public ROBOT_DRIVE_PARAMS updateFollowerOnInit(boolean updateFollowerOnInit) {
            this.UPDATE_FOLLOWER_ON_INIT = updateFollowerOnInit;
            return this;
        }

        public ROBOT_DRIVE_PARAMS initialHeadingOffset(double initialHeadingOffset) {
            this.INITIAL_HEADING_OFFSET = initialHeadingOffset;
            return this;
        }

        public ROBOT_DRIVE_PARAMS zeroHeadingButton(Button zeroHeadingButton) {
            this.ZERO_HEADING_BUTTON = zeroHeadingButton;
            return this;
        }

        @Override
        public ROBOT_DRIVE_PARAMS copy() {
            return new ROBOT_DRIVE_PARAMS(this);
        }

        @Override
        public ROBOT_DRIVE_PARAMS updateFromBot(RobotConstants bot) {
            return this;
        }
    }

    public final ROBOT_DRIVE_PARAMS params;
    public final MecanumDrive mecanumDrive;

    public RobotDrive(HardwareMap map, RobotConstants bot) {
        super(map, bot);
        params = resolveParams(RobotDrive.class, new ROBOT_DRIVE_PARAMS());
        mecanumDrive = new MecanumDrive(map, bot, params);
    }

    public void setStartingPose(Pose pose) {
        mecanumDrive.setStartingPose(pose);
    }

    public void setTeleOpDrive(DriveVector vector) {
        mecanumDrive.setTeleOpDrive(vector);
    }

    public void setTeleOpDrive(DriveVector vector, boolean fieldCentric) {
        mecanumDrive.setTeleOpDrive(vector, fieldCentric);
    }

    public void startTeleOpDrive() {
        mecanumDrive.startTeleOpDrive();
    }

    public Pose getPose() {
        return mecanumDrive.getPose();
    }

    public Vector getVelocity() {
        return mecanumDrive.getVelocity();
    }

    public double getAngularVelocity() {
        return mecanumDrive.getAngularVelocity();
    }

    public Vector getAcceleration() {
        return mecanumDrive.getAcceleration();
    }

    public void zeroHeadingOffset() {
        mecanumDrive.zeroHeadingOffset();
    }

    public void setStickCurve(DoubleUnaryOperator curve) {
        mecanumDrive.setStickCurve(curve);
    }

    public Motor[] getDriveMotors() {
        return mecanumDrive.getDriveMotors();
    }

    public boolean isUsingManualDrive() {
        return mecanumDrive.useManualDrive;
    }

    @Override
    public void periodic(GpBindings GpBindings) {
        mecanumDrive.periodic();
        if (GpBindings != null && GpBindings.isPressed(params.ZERO_HEADING_BUTTON)) {
            zeroHeadingOffset();
            setTeleOpDrive(new DriveVector(-GpBindings.leftStick.getY(), GpBindings.leftStick.getX(), GpBindings.rightStick.getX()));
        } else if (GpBindings != null) {
            setTeleOpDrive(new DriveVector(-GpBindings.leftStick.getY(), GpBindings.leftStick.getX(), GpBindings.rightStick.getX()));
        }
    }

    public Command teleOp(Supplier<GpBindings> GpBindingsSupplier) {
        return Commands.infinite(() -> periodic(GpBindingsSupplier.get()));
    }
}
