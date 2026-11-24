package org.firstinspires.ftc.teamcode.util.game;

import com.rmsftc.utils.math.Pose;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.PriorityQueue;

/**
 * A* pathfinder over a uniform cell grid of the whole 144x144 in field.
 *
 * <p>Replaces {@code DijkstraPathfinder} (visibility graph over a 3x3 lattice).
 * The grid covers the field in {@link #CELL_SIZE_IN} cells; a cell is blocked
 * iff its center point lies inside the forbidden geometry. Movement is
 * 8-directional with the standard octile heuristic, and a diagonal step is
 * allowed only when both orthogonal neighbor cells are free, so the path
 * never cuts the corner of a blocked cell.</p>
 *
 * <p>Why center-testing is safe here: every caller passes a forbidden space
 * already inflated by the robot's half-diagonal plus clearance (about 13 in
 * total in {@link Field#transitWaypoints}), while a 6 in cell's corner is at
 * most 6/sqrt(2) ~ 4.3 in from its center. A cell whose center is free can
 * therefore clip the forbidden region by at most ~4.3 in at a corner, leaving
 * roughly 8.5 in of the inflation margin intact, and the returned waypoints
 * are cell centers (or the exact start/end poses), never cell corners.</p>
 *
 * <p>Contract (same as the old pathfinder): {@link #findPath} returns a list
 * from the exact start pose to the exact end pose, with intermediate
 * waypoints at cell centers and intermediate headings pointing toward the
 * next point. It returns an empty list when no route exists (start or goal
 * cell blocked, or the search cannot reach the goal).</p>
 */
public class AStarPathfinder {

    /**
     * Cell edge length in inches. One-line tunable: the grid is
     * 144 / CELL_SIZE_IN cells per side (24x24 = 576 cells at 6 in).
     */
    public static final double CELL_SIZE_IN = 6.0;

    private static final double FIELD_SIZE_IN = 144.0;
    private static final int CELLS_PER_SIDE = (int) Math.round(FIELD_SIZE_IN / CELL_SIZE_IN);
    private static final double DIAGONAL_COST = CELL_SIZE_IN * Math.sqrt(2.0);
    private static final double SQRT2_MINUS_1 = Math.sqrt(2.0) - 1.0;
    private static final double EPSILON = 1e-9;

    private static final GeometryFactory GEOMETRY_FACTORY = new GeometryFactory();

    /** One entry in the A* open set. */
    private static final class SearchNode {
        final int col;
        final int row;
        final double g;
        final double f;

        SearchNode(int col, int row, double g, double f) {
            this.col = col;
            this.row = row;
            this.g = g;
            this.f = f;
        }
    }

    private static final Comparator<SearchNode> BY_F_THEN_HEURISTIC = new Comparator<SearchNode>() {
        @Override
        public int compare(SearchNode a, SearchNode b) {
            int byF = Double.compare(a.f, b.f);
            if (byF != 0) return byF;
            // Prefer the node closer to the goal so fewer cells are expanded.
            return Double.compare(a.f - a.g, b.f - b.g);
        }
    };

    /**
     * Finds the shortest path from start to end while dodging forbidden spaces.
     *
     * @param start          The current robot pose.
     * @param end            The target destination pose.
     * @param forbiddenSpace The JTS geometry containing all inflated field obstacles.
     * @return A sequential list of Pose objects from start to end. Returns an empty list if trapped.
     */
    public List<Pose> findPath(Pose start, Pose end, Geometry forbiddenSpace) {
        Coordinate startCoord = new Coordinate(start.x(), start.y());
        Coordinate endCoord = new Coordinate(end.x(), end.y());
        if (forbiddenSpace != null && !forbiddenSpace.isEmpty()) {
            if (forbiddenSpace.contains(GEOMETRY_FACTORY.createPoint(startCoord))
                    || forbiddenSpace.contains(GEOMETRY_FACTORY.createPoint(endCoord))) {
                return Collections.emptyList();
            }
        }

        int startCol = snapCell(start.x());
        int startRow = snapCell(start.y());
        int goalCol = snapCell(end.x());
        int goalRow = snapCell(end.y());

        boolean[][] blocked = computeBlockedCells(forbiddenSpace);
        if (blocked[startCol][startRow] || blocked[goalCol][goalRow]) {
            return Collections.emptyList();
        }

        // Start and goal in the same cell: no grid search is possible.
        // Mirror the old pathfinder's single-pose path when the poses coincide;
        // otherwise take the direct shot when it is clear, and give up when it is not.
        if (startCol == goalCol && startRow == goalRow) {
            if (startCoord.distance(endCoord) < EPSILON) {
                return Collections.singletonList(new Pose(start.x(), start.y(), start.heading()));
            }
            if (hasLineOfSight(startCoord, endCoord, forbiddenSpace)) {
                List<Pose> direct = new ArrayList<>(2);
                direct.add(new Pose(start.x(), start.y(), start.heading()));
                direct.add(new Pose(end.x(), end.y(), end.heading()));
                return direct;
            }
            return Collections.emptyList();
        }

        List<Coordinate> cellPath = search(startCol, startRow, goalCol, goalRow, blocked);
        if (cellPath.isEmpty()) {
            return Collections.emptyList();
        }

        // Exact start/end poses bookend the cell centers.
        List<Coordinate> rawWaypoints = new ArrayList<>(cellPath.size() + 1);
        rawWaypoints.add(startCoord);
        for (int i = 1; i < cellPath.size() - 1; i++) {
            rawWaypoints.add(cellPath.get(i));
        }
        rawWaypoints.add(endCoord);

        List<Coordinate> waypoints = smooth(rawWaypoints, forbiddenSpace);

        List<Pose> path = new ArrayList<>(waypoints.size());
        for (int i = 0; i < waypoints.size(); i++) {
            Coordinate waypoint = waypoints.get(i);
            // Keep start and target headings intact; intermediate headings point towards the next point.
            double heading;
            if (i == 0) {
                heading = start.heading();
            } else if (i == waypoints.size() - 1) {
                heading = end.heading();
            } else {
                Coordinate next = waypoints.get(i + 1);
                heading = Math.atan2(next.y - waypoint.y, next.x - waypoint.x);
            }
            path.add(new Pose(waypoint.x, waypoint.y, heading));
        }
        return path;
    }

    /** Standard A* over the cell grid; returns the center coordinates from start cell to goal cell. */
    private List<Coordinate> search(int startCol, int startRow, int goalCol, int goalRow, boolean[][] blocked) {
        double[][] gScore = new double[CELLS_PER_SIDE][CELLS_PER_SIDE];
        int[][] cameFrom = new int[CELLS_PER_SIDE][CELLS_PER_SIDE];
        boolean[][] closed = new boolean[CELLS_PER_SIDE][CELLS_PER_SIDE];
        for (int col = 0; col < CELLS_PER_SIDE; col++) {
            for (int row = 0; row < CELLS_PER_SIDE; row++) {
                gScore[col][row] = Double.POSITIVE_INFINITY;
                cameFrom[col][row] = -1;
            }
        }

        gScore[startCol][startRow] = 0.0;
        PriorityQueue<SearchNode> open = new PriorityQueue<>(64, BY_F_THEN_HEURISTIC);
        open.add(new SearchNode(startCol, startRow, 0.0,
                heuristic(startCol, startRow, goalCol, goalRow)));

        while (!open.isEmpty()) {
            SearchNode current = open.poll();
            if (closed[current.col][current.row]) continue;
            // Stale queue entry: a cheaper route to this cell was found after it was pushed.
            if (current.g > gScore[current.col][current.row] + EPSILON) continue;
            closed[current.col][current.row] = true;

            if (current.col == goalCol && current.row == goalRow) {
                return reconstructPath(cameFrom, goalCol, goalRow);
            }

            for (int dc = -1; dc <= 1; dc++) {
                for (int dr = -1; dr <= 1; dr++) {
                    if (dc == 0 && dr == 0) continue;
                    int nextCol = current.col + dc;
                    int nextRow = current.row + dr;
                    if (nextCol < 0 || nextCol >= CELLS_PER_SIDE
                            || nextRow < 0 || nextRow >= CELLS_PER_SIDE) continue;
                    if (blocked[nextCol][nextRow] || closed[nextCol][nextRow]) continue;
                    if (dc != 0 && dr != 0) {
                        // No corner-cutting: a diagonal step is allowed only when
                        // both orthogonal neighbors are free.
                        if (blocked[current.col + dc][current.row]
                                || blocked[current.col][current.row + dr]) continue;
                    }
                    double stepCost = (dc != 0 && dr != 0) ? DIAGONAL_COST : CELL_SIZE_IN;
                    double tentativeG = gScore[current.col][current.row] + stepCost;
                    if (tentativeG + EPSILON < gScore[nextCol][nextRow]) {
                        gScore[nextCol][nextRow] = tentativeG;
                        cameFrom[nextCol][nextRow] = current.col * CELLS_PER_SIDE + current.row;
                        open.add(new SearchNode(nextCol, nextRow, tentativeG,
                                tentativeG + heuristic(nextCol, nextRow, goalCol, goalRow)));
                    }
                }
            }
        }
        return Collections.emptyList();
    }

    private List<Coordinate> reconstructPath(int[][] cameFrom, int goalCol, int goalRow) {
        List<Coordinate> path = new ArrayList<>();
        int index = goalCol * CELLS_PER_SIDE + goalRow;
        while (index != -1) {
            path.add(cellCenter(index / CELLS_PER_SIDE, index % CELLS_PER_SIDE));
            index = cameFrom[index / CELLS_PER_SIDE][index % CELLS_PER_SIDE];
        }
        Collections.reverse(path);
        return path;
    }

    /**
     * Greedy string-pulling: from each kept waypoint, jump to the farthest
     * later waypoint with a clear line of sight. This collapses the grid
     * staircase into straight legs comparable to the old visibility-graph
     * paths, while every kept segment is verified clear of forbidden space.
     */
    private List<Coordinate> smooth(List<Coordinate> rawWaypoints, Geometry forbiddenSpace) {
        List<Coordinate> smoothed = new ArrayList<>();
        int anchor = 0;
        smoothed.add(rawWaypoints.get(anchor));
        while (anchor < rawWaypoints.size() - 1) {
            int next = anchor + 1;
            for (int j = anchor + 2; j < rawWaypoints.size(); j++) {
                if (hasLineOfSight(rawWaypoints.get(anchor), rawWaypoints.get(j), forbiddenSpace)) {
                    next = j;
                }
            }
            smoothed.add(rawWaypoints.get(next));
            anchor = next;
        }
        return smoothed;
    }

    private boolean[][] computeBlockedCells(Geometry forbiddenSpace) {
        boolean[][] blocked = new boolean[CELLS_PER_SIDE][CELLS_PER_SIDE];
        if (forbiddenSpace == null || forbiddenSpace.isEmpty()) {
            return blocked;
        }
        for (int col = 0; col < CELLS_PER_SIDE; col++) {
            for (int row = 0; row < CELLS_PER_SIDE; row++) {
                Coordinate center = cellCenter(col, row);
                blocked[col][row] = forbiddenSpace.contains(GEOMETRY_FACTORY.createPoint(center));
            }
        }
        return blocked;
    }

    /** Octile distance: admissible (and consistent) for 8-direction movement. */
    private static double heuristic(int col, int row, int goalCol, int goalRow) {
        int dx = Math.abs(col - goalCol);
        int dy = Math.abs(row - goalRow);
        return CELL_SIZE_IN * (Math.max(dx, dy) + SQRT2_MINUS_1 * Math.min(dx, dy));
    }

    private static Coordinate cellCenter(int col, int row) {
        return new Coordinate(CELL_SIZE_IN * (col + 0.5), CELL_SIZE_IN * (row + 0.5));
    }

    /** Nearest cell index for a field coordinate, clamped to the grid. */
    private static int snapCell(double coordinate) {
        int cell = (int) Math.round((coordinate - CELL_SIZE_IN / 2.0) / CELL_SIZE_IN);
        return Math.max(0, Math.min(CELLS_PER_SIDE - 1, cell));
    }

    /**
     * Checks if a straight path exists between two coordinates without clipping obstacles.
     */
    private boolean hasLineOfSight(Coordinate c1, Coordinate c2, Geometry forbiddenSpace) {
        if (forbiddenSpace == null || forbiddenSpace.isEmpty()) {
            return true;
        }

        LineString line = GEOMETRY_FACTORY.createLineString(new Coordinate[]{c1, c2});

        // Touching obstacle boundaries at waypoint endpoints is allowed; entering obstacle interiors is not.
        return !line.intersects(forbiddenSpace) || line.touches(forbiddenSpace);
    }
}
