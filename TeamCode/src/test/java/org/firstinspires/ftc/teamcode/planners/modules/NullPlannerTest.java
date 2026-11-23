package org.firstinspires.ftc.teamcode.planners.modules;

import static org.junit.Assert.assertTrue;

import com.rmsftc.cmdlib.cmd.Command;
import com.rmsftc.cmdlib.cmd.Commands;

import com.rmsftc.utils.math.Pose;
import org.junit.Test;

import java.util.concurrent.atomic.AtomicBoolean;

public class NullPlannerTest {
    @Test
    public void completesImmediatelySoFollowingPlannersCanStart() {
        AtomicBoolean followingPlannerStarted = new AtomicBoolean(false);
        Command command = Commands.sequence(
                new Utils.NullPlanner(new Pose(0, 0, 0)),
                Commands.runOnce(() -> followingPlannerStarted.set(true))
        );

        command.execute();

        assertTrue(followingPlannerStarted.get());
    }
}
