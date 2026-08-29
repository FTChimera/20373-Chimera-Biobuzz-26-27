package org.firstinspires.ftc.teamcode.library.planners.paths;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.library.planners.PathPlanner;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;

import java.nio.file.Path;


public class FromTestPose {
    public static class ToIntakeThreeTest implements PathPlanner {
        Robot robot;
        Pose startPose;
        public ToIntakeThreeTest(Robot robot, PathPlanner prevPlanner) {
            this.robot = robot;
            fromPrev(prevPlanner);
        }
        @Override
        public Pose getEndPose() {
            return null;
        }

        @Override
        public Command run() {
            return null;
        }

        @Override
        public void setStartPose(Pose pose) {
            startPose=pose;
        }

        @Override
        public void buildPaths() {

        }
        @NonNull public String toString(){return"ToIntake3Test";}
    }
}
