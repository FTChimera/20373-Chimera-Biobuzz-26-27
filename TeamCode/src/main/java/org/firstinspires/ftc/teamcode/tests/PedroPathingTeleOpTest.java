package org.firstinspires.ftc.teamcode.tests;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.Button;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.GamepadEx;

import java.util.function.Supplier;

@SuppressWarnings("SpellCheckingInspection")
@Configurable
@Utility(name = "Pedro Pathing TeleOp Test")
public class PedroPathingTeleOpTest extends OpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    boolean isRobotCentric = false;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    public static double DegreesOffset = 90;
    private GamepadEx driver;

    @Override
    public void init() {
        driver = new GamepadEx(gamepad1);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? Constants.RED_DEFAULT_START_POSE: startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, Constants.BLUE_DEFAULT_START_POSE)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(0), 0.8))
                .build();
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        driver.update();
        if (!automatedDrive) {
            //Make the last parameter false for field-centric

            //This is the normal version to use in the TeleOp
            follower.setTeleOpDrive(
                    driver.leftStickY(),
                    -driver.leftStickX(),
                    -driver.rightStickX(),
                    isRobotCentric,
                    isRobotCentric ? 0 : Math.toRadians(DegreesOffset)
            );

        }
        if (driver.wasJustPressed(Button.BACK)) {
            DegreesOffset = Math.toDegrees(follower.getPose().getHeading());
        }

        //Automated PathFollowing
        if (driver.wasJustPressed(Button.A)) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (driver.wasJustPressed(Button.B)) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        // Call this once per loop after any teleop/path command updates.
        follower.update();
        telemetryM.update();

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        telemetryM.debug("Drive Mode", isRobotCentric?"Robot Centric":"Field Centric");
    }
}
