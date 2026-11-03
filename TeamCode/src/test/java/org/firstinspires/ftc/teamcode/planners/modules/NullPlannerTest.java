package org.firstinspires.ftc.teamcode.planners.modules;

import static org.junit.Assert.assertTrue;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;

import org.firstinspires.ftc.teamcode.util.poses.Pose2d;
import org.junit.Test;

import java.util.concurrent.atomic.AtomicBoolean;

public class NullPlannerTest {
    @Test
    public void completesImmediatelySoFollowingPlannersCanStart() {
        AtomicBoolean followingPlannerStarted = new AtomicBoolean(false);
        Command command = Groups.sequential(
                new Utils.NullPlanner(new Pose2d(0, 0, 0)).run(),
                Commands.instant(() -> followingPlannerStarted.set(true))
        );

        command.start();
        command.execute();

        assertTrue(followingPlannerStarted.get());
    }
}
