package org.firstinspires.ftc.teamcode.library.planners.paths;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.library.AllianceColor;
import org.firstinspires.ftc.teamcode.library.planners.Common;
import org.firstinspires.ftc.teamcode.library.planners.PathPlanner;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro;


public class FromTestPose {
    public static class ToIntakeThreeTest implements PathPlanner {
        volatile Robot robot;
        Pose startPose;
        PathChain intakePath;
        PathChain returnPath;
        volatile Intake intake;
        public ToIntakeThreeTest(Robot robot, PathPlanner prevPlanner) {
            this.robot = robot;
            this.intake = robot.getIntake();
            fromPrev(prevPlanner);
        }
        @Override
        public Pose getEndPose() {
            return Common.score.pose();
        }

        @Override
        public Command run() {
            return sequential(
                    instant(() -> {if (intake!=null) intake.FullPower();}),
                    follow(robot.requireDrive().getFollower(), intakePath),
                    follow(robot.requireDrive().getFollower(), returnPath),
                    instant(() -> {if (intake!=null) intake.Stop();})
            );
        }

        @Override
        public void setStartPose(Pose pose) {
            startPose=pose;
        }

        @Override
        public void buildPaths() {
            intakePath = new PathChain(Pedro.linear(startPose, Common.intakeTest.pose()));
            returnPath = new PathChain(Pedro.linear(Common.intakeTest.pose(), Common.score.pose()));
        }
        @NonNull public String toString(){return"ToIntake3Test";}
    }
}
