package org.firstinspires.ftc.teamcode.util.game;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.rmsftc.utils.math.Pose;
import org.junit.Test;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.Polygon;

import java.util.List;

public class AStarPathfinderTest {
    private static final GeometryFactory FACTORY = Field.geometryFactory;
    private static final double EPSILON = 1e-9;

    @Test
    public void openFieldPathKeepsExactEndpointsAndAvoidsSmallBox() {
        Polygon box = box(60, 66, 66, 78);
        Pose start = new Pose(20, 72, 0);
        Pose goal = new Pose(124, 72, 0);

        List<Pose> path = new AStarPathfinder().findPath(start, goal, box);

        assertFalse(path.isEmpty());
        assertEndpoints(path, start, goal);
        assertNoWaypointInside(path, box);
        double straight = start.distance(goal);
        assertTrue("Path should detour around the box, length " + pathLength(path),
                pathLength(path) > straight);
        assertTrue("Path length " + pathLength(path) + " is wildly longer than straight " + straight,
                pathLength(path) < 1.5 * straight);
    }

    @Test
    public void hiveLikeRectForcesRouteAroundIt() {
        Polygon hive = box(60, 48, 84, 96);
        Pose start = new Pose(30, 72, 0);
        Pose goal = new Pose(114, 72, 0);

        List<Pose> path = new AStarPathfinder().findPath(start, goal, hive);

        assertFalse(path.isEmpty());
        assertEndpoints(path, start, goal);
        assertNoWaypointInside(path, hive);
        double straight = start.distance(goal);
        assertTrue("Path should detour around the hive, length " + pathLength(path),
                pathLength(path) > straight);
        assertTrue("Path length " + pathLength(path) + " is wildly longer than straight " + straight,
                pathLength(path) < 1.5 * straight);
    }

    @Test
    public void returnsEmptyWhenStartCellBlocked() {
        Polygon box = box(60, 60, 66, 66);
        List<Pose> path = new AStarPathfinder().findPath(
                new Pose(63, 63), new Pose(120, 100), box);
        assertTrue(path.isEmpty());
    }

    @Test
    public void returnsEmptyWhenGoalCellBlocked() {
        Polygon box = box(60, 60, 66, 66);
        List<Pose> path = new AStarPathfinder().findPath(
                new Pose(20, 20), new Pose(63, 63), box);
        assertTrue(path.isEmpty());
    }

    @Test
    public void returnsEmptyWhenWallSpansField() {
        Polygon wall = box(60, 0, 66, 144);
        List<Pose> path = new AStarPathfinder().findPath(
                new Pose(20, 72), new Pose(120, 72), wall);
        assertTrue(path.isEmpty());
    }

    @Test
    public void neverCutsDiagonalThroughSharedCornerOfBlockedCells() {
        // Two diagonally adjacent blocked cells share the corner (12, 12).
        // Without the no-corner-cutting rule the cheapest route steps
        // diagonally from cell (1, 1) to cell (2, 2) straight through it.
        Geometry forbidden = box(12, 6, 18, 12).union(box(6, 12, 12, 18));
        Pose start = new Pose(3, 9, 0);
        Pose goal = new Pose(21, 21, 0);

        List<Pose> path = new AStarPathfinder().findPath(start, goal, forbidden);

        assertFalse(path.isEmpty());
        assertEndpoints(path, start, goal);
        assertNoWaypointInside(path, forbidden);
        for (int i = 0; i < path.size() - 1; i++) {
            Pose a = path.get(i);
            Pose b = path.get(i + 1);
            assertTrue("Segment " + a + " -> " + b + " cuts through the shared blocked corner (12, 12)",
                    pointSegmentDistance(12, 12, a.x(), a.y(), b.x(), b.y()) > 1e-6);
        }
    }

    @Test
    public void openFieldPathIsDirect() {
        Pose start = new Pose(10, 10, 0);
        Pose goal = new Pose(130, 130, 0);

        List<Pose> path = new AStarPathfinder().findPath(start, goal, null);

        assertEquals(2, path.size());
        assertEndpoints(path, start, goal);
        assertEquals(start.distance(goal), pathLength(path), 1e-9);
    }

    private static Polygon box(double x1, double y1, double x2, double y2) {
        return FACTORY.createPolygon(new Coordinate[]{
                new Coordinate(x1, y1), new Coordinate(x2, y1), new Coordinate(x2, y2),
                new Coordinate(x1, y2), new Coordinate(x1, y1)});
    }

    private static void assertEndpoints(List<Pose> path, Pose start, Pose goal) {
        Pose first = path.get(0);
        assertEquals(start.x(), first.x(), EPSILON);
        assertEquals(start.y(), first.y(), EPSILON);
        assertEquals(start.heading(), first.heading(), EPSILON);
        Pose last = path.get(path.size() - 1);
        assertEquals(goal.x(), last.x(), EPSILON);
        assertEquals(goal.y(), last.y(), EPSILON);
        assertEquals(goal.heading(), last.heading(), EPSILON);
    }

    private static void assertNoWaypointInside(List<Pose> path, Geometry forbidden) {
        for (Pose waypoint : path) {
            assertFalse("Waypoint " + waypoint + " entered forbidden space",
                    forbidden.contains(FACTORY.createPoint(
                            new Coordinate(waypoint.x(), waypoint.y()))));
        }
    }

    private static double pathLength(List<Pose> path) {
        double length = 0.0;
        for (int i = 0; i < path.size() - 1; i++) {
            length += path.get(i).distance(path.get(i + 1));
        }
        return length;
    }

    private static double pointSegmentDistance(double px, double py,
                                               double x1, double y1, double x2, double y2) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        double lengthSquared = dx * dx + dy * dy;
        double t = lengthSquared == 0 ? 0
                : Math.max(0, Math.min(1, ((px - x1) * dx + (py - y1) * dy) / lengthSquared));
        double closestX = x1 + t * dx;
        double closestY = y1 + t * dy;
        return Math.hypot(px - closestX, py - closestY);
    }
}
