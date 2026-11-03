package org.firstinspires.ftc.teamcode.opmodes.auto;


import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.api.Paths;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.config.OpModeStorage;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.pedro.commands.Scheduler;
import org.firstinspires.ftc.teamcode.planners.CommonKt;
import org.firstinspires.ftc.teamcode.planners.ShootPoses;
import org.firstinspires.ftc.teamcode.planners.StartPoses;
import org.firstinspires.ftc.teamcode.planners.commands.DrivetrainCommands;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.RobotMode;

@Autonomous
public class BasicAutonPreloads  extends LinearOpMode  {
    /**
     * Override this method and place your code here.
     * <p>
     * Please do not catch {@link InterruptedException}s that are thrown in your OpMode
     * unless you are doing it to perform some brief cleanup, in which case you must exit
     * immediately afterward. Once the OpMode has been told to stop, your ability to
     * control hardware will be limited.
     *
     * @throws InterruptedException When the OpMode is stopped while calling a method
     *                              that can throw {@link InterruptedException}
     */
    @Override
    public void runOpMode() throws InterruptedException {
        Scheduler scheduler = new Scheduler();
        OpModeStorage.pose = StartPoses.gardenArea;
        OpModeStorage.allianceColor = AllianceColor.RED;
        CommonKt.init(OpModeStorage.allianceColor);
        Robot robot = new Robot(telemetry, hardwareMap, OpModeStorage.allianceColor, RobotMode.AUTONOMOUS, OpModeStorage.pose);
        robot.matchMode = OpModeStorage.matchMode;
        robot.debug = true;

        scheduler.schedule(
                sequential(
                        DrivetrainCommands.follow(robot.follower, toShootPreloads())
                )
        );

        waitForStart();
        if (isStopRequested()) {
            robot.stopEverything();
            return;
        }
        robot.start();
        try {
            while (opModeIsActive()) {
                scheduler.execute();
                robot.update();
            }
        } finally {
            scheduler.reset();
            robot.stopEverything();
        }
    }

    private Path toShootPreloads() {
        return Paths.line(StartPoses.gardenArea, ShootPoses.garden).linear(StartPoses.gardenArea, ShootPoses.garden);
    }
}
