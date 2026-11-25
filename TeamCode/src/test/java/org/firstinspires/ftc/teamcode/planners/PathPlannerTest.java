package org.firstinspires.ftc.teamcode.planners;

import static org.junit.Assert.assertEquals;

import com.rmsftc.cmdlib.Status;
import com.rmsftc.cmdlib.cmd.Command;
import com.rmsftc.cmdlib.cmd.Commands;
import com.rmsftc.utils.math.Pose;
import org.firstinspires.ftc.teamcode.util.game.Field;
import org.junit.Test;

/**
 * A leg whose paths cannot be built must degrade (end at once) instead of
 * throwing out of start() and killing the OpMode — BA does not catch
 * exceptions from start(), so a throw there would skip onStop.
 */
public class PathPlannerTest {
    private static final class ThrowingPlanner extends PathPlanner {
        private final boolean throwInBuildPaths;

        ThrowingPlanner(boolean throwInBuildPaths) {
            super(null);
            this.throwInBuildPaths = throwInBuildPaths;
        }

        @Override
        public Pose getEndPose() {
            return new Pose(0, 0, 0);
        }

        @Override
        public Position from() {
            return Position.ANY;
        }

        @Override
        public Position to() {
            return Position.ANY;
        }

        @Override
        protected void buildPaths() {
            if (throwInBuildPaths) {
                throw new Field.NoRouteException("no route for test");
            }
        }

        @Override
        protected Command buildCommand() {
            if (!throwInBuildPaths) {
                throw new IllegalStateException("no command for test");
            }
            return Commands.waitMs(0);
        }

        @Override
        public String toString() {
            return "ThrowingPlanner";
        }
    }

    @Test
    public void buildPathsThrowDegradesToDoneInsteadOfThrowing() {
        PathPlanner planner = new ThrowingPlanner(true);
        assertEquals(Status.FINISHED, planner.execute());
    }

    @Test
    public void buildCommandThrowDegradesToDoneInsteadOfThrowing() {
        PathPlanner planner = new ThrowingPlanner(false);
        assertEquals(Status.FINISHED, planner.execute());
    }
}
