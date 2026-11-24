package org.firstinspires.ftc.teamcode.util.game;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.rmsftc.utils.math.Pose;
import org.junit.Test;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;
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
        assertTrue(path.size() > 2);
        assertEndpoints(path, start, goal);
        assertNoWaypointInside(path, box);
        assertNoSegmentEnters(path, box);
        assertHeadingsPointTowardNext(path);
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
        assertTrue(path.size() > 2);
        assertEndpoints(path, start, goal);
        assertNoWaypointInside(path, hive);
        assertNoSegmentEnters(path, hive);
        assertHeadingsPointTowardNext(path);
        double straight = start.distance(goal);
        assertTrue("Path should detour around the hive, length " + pathLength(path),
                pathLength(path) > straight);
        assertTrue("Path length " + pathLength(path) + " is wildly longer than straight " + straight,
                pathLength(path) < 1.5 * straight);
    }

    @Test
    public void startCellBlockedButExactStartLegalRoutes() {
        // The exact start (60.5, 60.5) is outside the box, but its cell center
        // (63, 63) is inside it. The old code returned empty here even though
        // the pose is legal; the search must start from the nearest free cell
        // visible from the exact start instead.
        Polygon box = box(61, 61, 67, 67);
        Pose start = new Pose(60.5, 60.5, 0);
        Pose goal = new Pose(120, 100, 0);

        List<Pose> path = new AStarPathfinder().findPath(start, goal, box);

        assertFalse(path.isEmpty());
        assertEndpoints(path, start, goal);
        assertNoWaypointInside(path, box);
        assertNoSegmentEnters(path, box);
        assertHeadingsPointTowardNext(path);
        assertTrue("Path length " + pathLength(path) + " is wildly longer than straight "
                        + start.distance(goal),
                pathLength(path) < 1.5 * start.distance(goal));
    }

    @Test
    public void goalCellBlockedButExactGoalLegalRoutes() {
        // Mirror of the start case: the exact goal (60.5, 60.5) is outside the
        // box, but its cell center (63, 63) is inside it. The search must end
        // at the nearest free cell visible from the exact goal instead.
        Polygon box = box(61, 61, 67, 67);
        Pose start = new Pose(20, 20, 0);
        Pose goal = new Pose(60.5, 60.5, 0);

        List<Pose> path = new AStarPathfinder().findPath(start, goal, box);

        assertFalse(path.isEmpty());
        assertEndpoints(path, start, goal);
        assertNoWaypointInside(path, box);
        assertNoSegmentEnters(path, box);
        assertHeadingsPointTowardNext(path);
        assertTrue("Path length " + pathLength(path) + " is wildly longer than straight "
                        + start.distance(goal),
                pathLength(path) < 1.5 * start.distance(goal));
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
        assertNoSegmentEnters(path, forbidden);
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

    @Test
    public void startEqualsGoalReturnsSinglePose() {
        Pose pose = new Pose(50, 50, 0.5);
        Polygon farAway = box(100, 100, 110, 110);

        List<Pose> path = new AStarPathfinder().findPath(pose, pose, farAway);

        assertEquals(1, path.size());
        assertEquals(pose.x(), path.get(0).x(), EPSILON);
        assertEquals(pose.y(), path.get(0).y(), EPSILON);
        assertEquals(pose.heading(), path.get(0).heading(), EPSILON);
    }

    @Test
    public void sameCellDirectShot() {
        // Both poses snap to cell (8, 8); the direct segment is clear.
        Pose start = new Pose(50, 50, 0.5);
        Pose goal = new Pose(53, 52, -1.0);

        List<Pose> path = new AStarPathfinder().findPath(start, goal, null);

        assertEquals(2, path.size());
        assertEndpoints(path, start, goal);
    }

    @Test
    public void sameCellBlockedLineOfSightDetoursThroughNeighbor() {
        // Start (49, 49) and goal (53, 51) share cell (8, 8), whose center
        // (51, 51) is free, but the box (51.5, 49)-(53.5, 50.5) sits on the
        // direct segment. The path must detour through a neighboring cell
        // visible from both endpoints instead of giving up.
        Polygon box = box(51.5, 49, 53.5, 50.5);
        Pose start = new Pose(49, 49, 0.5);
        Pose goal = new Pose(53, 51, -1.0);

        List<Pose> path = new AStarPathfinder().findPath(start, goal, box);

        assertEquals(3, path.size());
        assertEndpoints(path, start, goal);
        assertNoWaypointInside(path, box);
        assertNoSegmentEnters(path, box);
        assertHeadingsPointTowardNext(path);
    }

    @Test
    public void sameCellNoVisibleDetourNeighborReturnsEmpty() {
        // Start (49, 49) and goal (53, 51) share cell (8, 8); the small box
        // blocks the direct segment, and the frame walls around the cell
        // block the line of sight to every neighboring cell center, so no
        // detour cell is visible from both endpoints.
        Geometry forbidden = frameWalls().union(box(51.5, 49, 53.5, 50.5));

        List<Pose> path = new AStarPathfinder().findPath(
                new Pose(49, 49, 0), new Pose(53, 51, 0), forbidden);

        assertTrue(path.isEmpty());
    }

    @Test
    public void legalPoseWithNoVisibleFreeCellReturnsEmpty() {
        // The start (49, 49) is legal, but its own cell center (51, 51) is
        // inside the box and the frame walls block the line of sight to
        // every other cell center, so no search cell is reachable.
        Geometry forbidden = frameWalls().union(box(50, 50, 53, 52));

        List<Pose> path = new AStarPathfinder().findPath(
                new Pose(49, 49, 0), new Pose(120, 120, 0), forbidden);

        assertTrue(path.isEmpty());
    }

    /**
     * Four thin walls forming a closed frame around cell (8, 8) (x 48-54,
     * y 48-54). The interior is open, but any line of sight from inside the
     * frame to a cell center outside it crosses a wall.
     */
    private static Geometry frameWalls() {
        return box(46, 46, 48, 56)
                .union(box(54, 46, 56, 56))
                .union(box(48, 46, 54, 48))
                .union(box(48, 54, 54, 56));
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

    /** No segment between consecutive waypoints may enter the forbidden interior. */
    private static void assertNoSegmentEnters(List<Pose> path, Geometry forbidden) {
        for (int i = 0; i < path.size() - 1; i++) {
            Pose a = path.get(i);
            Pose b = path.get(i + 1);
            LineString segment = FACTORY.createLineString(new Coordinate[]{
                    new Coordinate(a.x(), a.y()), new Coordinate(b.x(), b.y())});
            assertTrue("Segment " + a + " -> " + b + " enters forbidden space",
                    !segment.intersects(forbidden) || segment.touches(forbidden));
        }
    }

    /** Every intermediate waypoint's heading points toward the next waypoint. */
    private static void assertHeadingsPointTowardNext(List<Pose> path) {
        for (int i = 1; i < path.size() - 1; i++) {
            Pose current = path.get(i);
            Pose next = path.get(i + 1);
            double expected = Math.atan2(next.y() - current.y(), next.x() - current.x());
            assertHeadingEquals("Waypoint " + current + " heading should point toward " + next,
                    expected, current.heading());
        }
    }

    /** Headings are angles: compare modulo 2*PI (FTCUtils Pose normalizes to [0, 2*PI)). */
    private static void assertHeadingEquals(String message, double expected, double actual) {
        double diff = expected - actual;
        while (diff > Math.PI) diff -= 2.0 * Math.PI;
        while (diff <= -Math.PI) diff += 2.0 * Math.PI;
        assertEquals(message, 0.0, diff, 1e-9);
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
