package org.firstinspires.ftc.teamcode.util.game;

import com.rmsftc.utils.math.Pose;
import org.locationtech.jts.geom.Coordinate;

import org.firstinspires.ftc.teamcode.planners.Constants;
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
 * The grid covers the field in {@code Constants.Field.AStar.CELL_SIZE_IN} cells; a cell is blocked
 * iff its center point lies inside the forbidden geometry. Movement is
 * 8-directional with the standard octile heuristic, and a diagonal step is
 * allowed only when both orthogonal neighbor cells are free, so the path
 * never cuts the corner of a blocked cell.</p>
 *
 * <p>Guarantee: every segment of the returned path is verified by
 * line-of-sight against the forbidden geometry. Neighbor steps are checked
 * during the search, the exact start/end legs are checked when their search
 * cells are chosen, and the smoothing pass only keeps verified segments. The
 * robot center therefore never enters the forbidden interior, and the body
 * keeps whatever clearance the caller built into the forbidden space. In
 * {@code Field.transitWaypoints} that inflation is the body's half-diagonal,
 * hypot(17, 17) / 2 ~ 12.02 in, plus TRANSIT_CLEARANCE_INCHES = 1.0 in of
 * real clearance (~13.02 in total); only that 1.0 in is clearance, the rest
 * is the body's own extent.</p>
 *
 * <p>When the start or goal cell center is blocked but the exact pose is
 * legal (outside the forbidden space), the search starts from the nearest
 * free cell visible from the exact pose instead of giving up, so poses that
 * sit just outside an inflated obstacle still route.</p>
 *
 * <p>Contract (same as the old pathfinder): {@link #findPath} returns a list
 * from the exact start pose to the exact end pose, with intermediate
 * waypoints at cell centers and intermediate headings pointing toward the
 * next point. It returns an empty list when no route exists (start or goal
 * pose inside the forbidden space, no reachable search cell, or the search
 * cannot reach the goal). Start and goal in the same cell take the direct
 * shot when it is clear, detour through a neighboring cell when it is not,
 * and give up when no neighbor is visible from both endpoints.</p>
 */
public class AStarPathfinder {

    // Cell geometry lives in Constants.Field.AStar (cell edge 6 in, so the
    // grid is Constants.Field.SIZE_IN / Constants.Field.AStar.CELL_SIZE_IN = 24 cells per side, 24x24 = 576 cells).

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

        boolean[][] blocked = computeBlockedCells(forbiddenSpace);

        int[] startCell = nearestReachableCell(startCoord, snapCell(start.x()), snapCell(start.y()),
                blocked, forbiddenSpace);
        int[] goalCell = nearestReachableCell(endCoord, snapCell(end.x()), snapCell(end.y()),
                blocked, forbiddenSpace);
        if (startCell == null || goalCell == null) {
            return Collections.emptyList();
        }
        int startCol = startCell[0];
        int startRow = startCell[1];
        int goalCol = goalCell[0];
        int goalRow = goalCell[1];

        // Start and goal search from the same cell: no grid search is possible.
        // Mirror the old pathfinder's single-pose path when the poses coincide;
        // otherwise take the direct shot when it is clear, detour through a
        // neighboring cell when it is not, and give up when no neighbor is
        // visible from both endpoints.
        if (startCol == goalCol && startRow == goalRow) {
            if (startCoord.distance(endCoord) < Constants.Field.AStar.EPSILON) {
                return Collections.singletonList(new Pose(start.x(), start.y(), start.heading()));
            }
            if (hasLineOfSight(startCoord, endCoord, forbiddenSpace)) {
                List<Pose> direct = new ArrayList<>(2);
                direct.add(new Pose(start.x(), start.y(), start.heading()));
                direct.add(new Pose(end.x(), end.y(), end.heading()));
                return direct;
            }
            Coordinate detour = nearestDetourCell(startCoord, endCoord, startCol, startRow,
                    blocked, forbiddenSpace);
            if (detour == null) {
                return Collections.emptyList();
            }
            List<Pose> detourPath = new ArrayList<>(3);
            detourPath.add(new Pose(start.x(), start.y(), start.heading()));
            detourPath.add(new Pose(detour.x, detour.y,
                    Math.atan2(end.y() - detour.y, end.x() - detour.x)));
            detourPath.add(new Pose(end.x(), end.y(), end.heading()));
            return detourPath;
        }

        List<Coordinate> cellPath = search(startCol, startRow, goalCol, goalRow, blocked, forbiddenSpace);
        if (cellPath.isEmpty()) {
            return Collections.emptyList();
        }

        // Exact start/end poses bookend the cell centers, which are kept (not
        // dropped) so the exact-start and exact-goal legs are part of the
        // waypoint list. Every consecutive pair is line-of-sight verified:
        // neighbor steps were checked during the search, and the exact
        // endpoints were checked against their search cells when those cells
        // were chosen, so smoothing can only replace verified segments with
        // verified segments. Consecutive duplicates (an exact pose that is
        // already a cell center) are dropped so no zero-length leg is emitted.
        List<Coordinate> rawWaypoints = new ArrayList<>(cellPath.size() + 2);
        rawWaypoints.add(startCoord);
        for (Coordinate center : cellPath) {
            if (rawWaypoints.get(rawWaypoints.size() - 1).distance(center) >= Constants.Field.AStar.EPSILON) {
                rawWaypoints.add(center);
            }
        }
        if (rawWaypoints.get(rawWaypoints.size() - 1).distance(endCoord) >= Constants.Field.AStar.EPSILON) {
            rawWaypoints.add(endCoord);
        }

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
    private List<Coordinate> search(int startCol, int startRow, int goalCol, int goalRow,
                                    boolean[][] blocked, Geometry forbiddenSpace) {
        double[][] gScore = new double[Constants.Field.AStar.CELLS_PER_SIDE][Constants.Field.AStar.CELLS_PER_SIDE];
        int[][] cameFrom = new int[Constants.Field.AStar.CELLS_PER_SIDE][Constants.Field.AStar.CELLS_PER_SIDE];
        boolean[][] closed = new boolean[Constants.Field.AStar.CELLS_PER_SIDE][Constants.Field.AStar.CELLS_PER_SIDE];
        for (int col = 0; col < Constants.Field.AStar.CELLS_PER_SIDE; col++) {
            for (int row = 0; row < Constants.Field.AStar.CELLS_PER_SIDE; row++) {
                gScore[col][row] = Double.POSITIVE_INFINITY;
                cameFrom[col][row] = -1;
            }
        }

        gScore[startCol][startRow] = 0.0;
        PriorityQueue<SearchNode> open = new PriorityQueue<>(Constants.Field.AStar.OPEN_SET_INITIAL_CAPACITY, BY_F_THEN_HEURISTIC);
        open.add(new SearchNode(startCol, startRow, 0.0,
                heuristic(startCol, startRow, goalCol, goalRow)));

        while (!open.isEmpty()) {
            SearchNode current = open.poll();
            if (closed[current.col][current.row]) continue;
            // Stale queue entry: a cheaper route to this cell was found after it was pushed.
            if (current.g > gScore[current.col][current.row] + Constants.Field.AStar.EPSILON) continue;
            closed[current.col][current.row] = true;

            if (current.col == goalCol && current.row == goalRow) {
                return reconstructPath(cameFrom, goalCol, goalRow);
            }

            for (int dc = -1; dc <= 1; dc++) {
                for (int dr = -1; dr <= 1; dr++) {
                    if (dc == 0 && dr == 0) continue;
                    int nextCol = current.col + dc;
                    int nextRow = current.row + dr;
                    if (nextCol < 0 || nextCol >= Constants.Field.AStar.CELLS_PER_SIDE
                            || nextRow < 0 || nextRow >= Constants.Field.AStar.CELLS_PER_SIDE) continue;
                    if (blocked[nextCol][nextRow] || closed[nextCol][nextRow]) continue;
                    if (dc != 0 && dr != 0) {
                        // No corner-cutting: a diagonal step is allowed only when
                        // both orthogonal neighbors are free.
                        if (blocked[current.col + dc][current.row]
                                || blocked[current.col][current.row + dr]) continue;
                    }
                    // A step between cell centers is only usable when the robot
                    // center can actually drive it: both centers may be free
                    // while the segment between them clips a forbidden corner.
                    if (!hasLineOfSight(cellCenter(current.col, current.row),
                            cellCenter(nextCol, nextRow), forbiddenSpace)) continue;
                    double stepCost = (dc != 0 && dr != 0) ? Constants.Field.AStar.DIAGONAL_COST : Constants.Field.AStar.CELL_SIZE_IN;
                    double tentativeG = gScore[current.col][current.row] + stepCost;
                    if (tentativeG + Constants.Field.AStar.EPSILON < gScore[nextCol][nextRow]) {
                        gScore[nextCol][nextRow] = tentativeG;
                        cameFrom[nextCol][nextRow] = current.col * Constants.Field.AStar.CELLS_PER_SIDE + current.row;
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
        int index = goalCol * Constants.Field.AStar.CELLS_PER_SIDE + goalRow;
        while (index != -1) {
            path.add(cellCenter(index / Constants.Field.AStar.CELLS_PER_SIDE, index % Constants.Field.AStar.CELLS_PER_SIDE));
            index = cameFrom[index / Constants.Field.AStar.CELLS_PER_SIDE][index % Constants.Field.AStar.CELLS_PER_SIDE];
        }
        Collections.reverse(path);
        return path;
    }

    /**
     * Nearest cell (by ring, then by distance from the exact point) whose
     * center is free and visible from the exact point. The snapped cell
     * itself is ring 0, so a legal pose in a free cell keeps its own cell.
     * Returns null when no cell on the grid qualifies.
     */
    private int[] nearestReachableCell(Coordinate from, int col, int row, boolean[][] blocked,
                                       Geometry forbiddenSpace) {
        for (int ring = 0; ring < Constants.Field.AStar.CELLS_PER_SIDE; ring++) {
            List<int[]> candidates = new ArrayList<>();
            for (int[] cell : ringCells(col, row, ring)) {
                if (!blocked[cell[0]][cell[1]]) {
                    candidates.add(cell);
                }
            }
            candidates.sort(Comparator.comparingDouble(
                    cell -> from.distance(cellCenter(cell[0], cell[1]))));
            for (int[] cell : candidates) {
                if (hasLineOfSight(from, cellCenter(cell[0], cell[1]), forbiddenSpace)) {
                    return cell;
                }
            }
        }
        return null;
    }

    /**
     * Nearest neighboring cell (by ring, then by detour length) whose center
     * is visible from both exact endpoints. Ring 0 is the shared cell itself,
     * which cannot detour anywhere, so the search starts at ring 1. Returns
     * null when no neighbor is visible from both endpoints.
     */
    private Coordinate nearestDetourCell(Coordinate start, Coordinate end, int col, int row,
                                         boolean[][] blocked, Geometry forbiddenSpace) {
        for (int ring = 1; ring < Constants.Field.AStar.CELLS_PER_SIDE; ring++) {
            List<int[]> candidates = new ArrayList<>();
            for (int[] cell : ringCells(col, row, ring)) {
                if (!blocked[cell[0]][cell[1]]) {
                    candidates.add(cell);
                }
            }
            candidates.sort(Comparator.comparingDouble(cell -> {
                Coordinate center = cellCenter(cell[0], cell[1]);
                return start.distance(center) + center.distance(end);
            }));
            for (int[] cell : candidates) {
                Coordinate center = cellCenter(cell[0], cell[1]);
                if (hasLineOfSight(start, center, forbiddenSpace)
                        && hasLineOfSight(center, end, forbiddenSpace)) {
                    return center;
                }
            }
        }
        return null;
    }

    /** Cells at exactly Chebyshev distance {@code ring} from ({@code col}, {@code row}), clipped to the grid. */
    private List<int[]> ringCells(int col, int row, int ring) {
        List<int[]> cells = new ArrayList<>();
        for (int c = Math.max(0, col - ring); c <= Math.min(Constants.Field.AStar.CELLS_PER_SIDE - 1, col + ring); c++) {
            for (int r = Math.max(0, row - ring); r <= Math.min(Constants.Field.AStar.CELLS_PER_SIDE - 1, row + ring); r++) {
                if (Math.max(Math.abs(c - col), Math.abs(r - row)) == ring) {
                    cells.add(new int[]{c, r});
                }
            }
        }
        return cells;
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
        boolean[][] blocked = new boolean[Constants.Field.AStar.CELLS_PER_SIDE][Constants.Field.AStar.CELLS_PER_SIDE];
        if (forbiddenSpace == null || forbiddenSpace.isEmpty()) {
            return blocked;
        }
        for (int col = 0; col < Constants.Field.AStar.CELLS_PER_SIDE; col++) {
            for (int row = 0; row < Constants.Field.AStar.CELLS_PER_SIDE; row++) {
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
        return Constants.Field.AStar.CELL_SIZE_IN * (Math.max(dx, dy) + Constants.Field.AStar.SQRT2_MINUS_1 * Math.min(dx, dy));
    }

    private static Coordinate cellCenter(int col, int row) {
        return new Coordinate(Constants.Field.AStar.CELL_SIZE_IN * (col + 0.5), Constants.Field.AStar.CELL_SIZE_IN * (row + 0.5));
    }

    /** Nearest cell index for a field coordinate, clamped to the grid. */
    private static int snapCell(double coordinate) {
        int cell = (int) Math.round((coordinate - Constants.Field.AStar.CELL_SIZE_IN / 2.0) / Constants.Field.AStar.CELL_SIZE_IN);
        return Math.max(0, Math.min(Constants.Field.AStar.CELLS_PER_SIDE - 1, cell));
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
