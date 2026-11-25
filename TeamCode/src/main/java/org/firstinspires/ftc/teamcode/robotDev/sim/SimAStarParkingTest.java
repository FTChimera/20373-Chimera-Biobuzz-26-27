package org.firstinspires.ftc.teamcode.robotDev.sim;

import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rmsftc.utils.math.Pose;

import org.firstinspires.ftc.teamcode.planners.Constants;

@Autonomous(name = "Sim #5 A* Parking (AFTER #1 PASS)", group = "Sim diagnostics")
public class SimAStarParkingTest extends SimPathPinpointTest {
    @Override
    protected Path buildPath(Pose start) {
        if (Constants.Field.AStar.CELL_SIZE_IN != 6.0) {
            throw new IllegalStateException("Diagnostic requires production A* 6-in cells");
        }
        // Same Field/AStarPathfinder and 3-in inward target as Robot.moveToParkZone.
        // Follow this very Path, not moveToParkZone(), which would generate a second route.
        return robot.field.generatePathToLoadingZoneShortest(start, 3);
    }

    @Override
    protected double timeoutSeconds() {
        return 40;
    }

    @Override
    protected String routeDescription() {
        return "Production Field -> AStarPathfinder; cell size=" + Constants.Field.AStar.CELL_SIZE_IN
                + " in; EXACT piecewise-linear Pedro route below; run only after #1 PASS";
    }

    @Override
    protected boolean atDestination(Pose measured) {
        // Production park definition is BODY OVERLAP; use measured pose, not velocity compensation.
        return robot.field.isRobotOverlappingLoadingZone(measured);
    }
}
