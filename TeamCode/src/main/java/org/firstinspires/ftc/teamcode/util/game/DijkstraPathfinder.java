package org.firstinspires.ftc.teamcode.util.game;

import org.firstinspires.ftc.teamcode.util.poses.Pose2d;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;
import java.util.*;

public class DijkstraPathfinder {

    /**
     * Inner class representing a node inside the Visibility Graph.
     */
    private static class Node implements Comparable<Node> {
        Coordinate coordinate;
        double minDistance = Double.POSITIVE_INFINITY;
        Node previous;
        List<Edge> edges = new ArrayList<>();

        Node(Coordinate coordinate) {
            this.coordinate = coordinate;
        }

        @Override
        public int compareTo(Node other) {
            return Double.compare(this.minDistance, other.minDistance);
        }
    }

    private static class Edge {
        Node target;
        double weight;

        Edge(Node target, double weight) {
            this.target = target;
            this.weight = weight;
        }
    }

    /**
     * Finds the shortest path from start to end while dodging forbidden spaces.
     *
     * @param start          The current robot pose.
     * @param end            The target destination pose.
     * @param forbiddenSpace The JTS geometry containing all inflated field obstacles.
     * @return A sequential list of Pose2d objects from start to end. Returns an empty list if trapped.
     */
    public List<Pose2d> findPath(Pose2d start, Pose2d end, Geometry forbiddenSpace) {
        Coordinate startCoord = start.toCoordinate();
        Coordinate endCoord = end.toCoordinate();
        if (forbiddenSpace != null && !forbiddenSpace.isEmpty()) {
            if (forbiddenSpace.contains(forbiddenSpace.getFactory().createPoint(startCoord))
                    || forbiddenSpace.contains(forbiddenSpace.getFactory().createPoint(endCoord))) {
                return Collections.emptyList();
            }
        }

        // 1. Build a unique list of all potential waypoints (Start, Goal, and Obstacle Corners)
        List<Coordinate> uniquePoints = new ArrayList<>();
        uniquePoints.add(startCoord);
        uniquePoints.add(endCoord);

        // Add coordinates from the boundaries of all inflated obstacles
        if (forbiddenSpace != null) {
            for (Coordinate c : forbiddenSpace.getCoordinates()) {
                if (!uniquePoints.contains(c)) {
                    uniquePoints.add(c);
                }
            }
        }

        // Map raw JTS coordinates to Node structures
        Map<Coordinate, Node> nodeMap = new HashMap<>();
        List<Node> graphNodes = new ArrayList<>();
        for (Coordinate coord : uniquePoints) {
            Node node = new Node(coord);
            nodeMap.put(coord, node);
            graphNodes.add(node);
        }

        // 2. Build the Visibility Graph: Connect nodes that have a clear line of sight
        for (int i = 0; i < graphNodes.size(); i++) {
            for (int j = i + 1; j < graphNodes.size(); j++) {
                Node n1 = graphNodes.get(i);
                Node n2 = graphNodes.get(j);

                if (hasLineOfSight(n1.coordinate, n2.coordinate, forbiddenSpace)) {
                    double distance = n1.coordinate.distance(n2.coordinate);
                    n1.edges.add(new Edge(n2, distance));
                    n2.edges.add(new Edge(n1, distance));
                }
            }
        }

        // 3. Execute Dijkstra's Search Algorithm
        Node startNode = nodeMap.get(startCoord);
        Node endNode = nodeMap.get(endCoord);
        startNode.minDistance = 0.0;

        PriorityQueue<Node> vertexQueue = new PriorityQueue<>();
        vertexQueue.add(startNode);

        while (!vertexQueue.isEmpty()) {
            Node u = vertexQueue.poll();

            if (u == endNode) break; // Optimization: Stop early once we reach the goal

            for (Edge e : u.edges) {
                Node v = e.target;
                double weight = e.weight;
                double distanceThroughU = u.minDistance + weight;

                if (distanceThroughU < v.minDistance) {
                    vertexQueue.remove(v);
                    v.minDistance = distanceThroughU;
                    v.previous = u;
                    vertexQueue.add(v);
                }
            }
        }

        // 4. Reconstruct the resulting shortest path sequence
        List<Pose2d> path = new ArrayList<>();
        if (endNode.previous == null && startNode != endNode) {
            return Collections.emptyList(); // No valid path found (trapped by obstacles)
        }

        for (Node node = endNode; node != null; node = node.previous) {
            // Keep start and target headings intact, intermediate corner headings point towards next point
            double heading = (node == endNode) ? end.heading : 0.0;
            if (node == startNode) heading = start.heading;

            path.add(new Pose2d(node.coordinate.x, node.coordinate.y, heading));
        }

        Collections.reverse(path);

        // Fill intermediate corner headings so the robot dynamically tracks its vector direction
        for (int i = 1; i < path.size() - 1; i++) {
            Pose2d current = path.get(i);
            Pose2d next = path.get(i + 1);
            current.heading = Math.atan2(next.y - current.y, next.x - current.x);
        }

        return path;
    }

    /**
     * Checks if a straight path exists between two coordinates without clipping obstacles.
     */
    private boolean hasLineOfSight(Coordinate c1, Coordinate c2, Geometry forbiddenSpace) {
        if (forbiddenSpace == null || forbiddenSpace.isEmpty()) {
            return true;
        }

        LineString line = Field.geometryFactory.createLineString(new Coordinate[]{c1, c2});

        // Touching obstacle boundaries at waypoint endpoints is allowed; entering obstacle interiors is not.
        return !line.intersects(forbiddenSpace) || line.touches(forbiddenSpace);
    }
}