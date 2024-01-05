package com.igniterobotics.jvisibility;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import com.igniterobotics.jvisibility.geometry.Edge;
import com.igniterobotics.jvisibility.geometry.GeometryMath;
import com.igniterobotics.jvisibility.geometry.Point;
import com.igniterobotics.jvisibility.model.BSTOpenEdges;
import com.igniterobotics.jvisibility.model.Environment;
import com.igniterobotics.jvisibility.model.OpenEdges;
import com.igniterobotics.jvisibility.model.VisibilityGraph;

public class VisibilityGraphGenerator {
    private static Comparator<Point> getAngleComparator(Point p) {
        return new Comparator<Point>() {
            @Override
            public int compare(Point v1, Point v2) {
                double theta1 = GeometryMath.negativePiTo2Pi(Math.atan2(v1.y - p.y, v1.x - p.x));
                double theta2 = GeometryMath.negativePiTo2Pi(Math.atan2(v2.y - p.y, v2.x - p.x));

                if (theta1 < theta2)
                    return -1;
                else if (theta1 > theta2)
                    return 1;
                else {
                    if (Math.pow(p.x - v1.x, 2) + Math.pow(p.y - v1.y, 2) < Math.pow(p.x - v2.x, 2)
                            + Math.pow(p.y - v2.y, 2))
                        return -1;
                    else
                        return 1;
                }
            }
        };
    }

    public static List<Point> visibleVertices(Point start, Point end, Environment environment) {
        List<Edge> edges = environment.getEdges();
        List<Point> points = environment.getPoints();

        if (start != null && !points.contains(start)) points.add(start);
        if (end != null && !points.contains(end)) points.add(end);

        points.sort(getAngleComparator(start));

        OpenEdges openEdges = new BSTOpenEdges();

        Point pointInf = new Point(Double.MAX_VALUE, start.y);
        for (Edge edge : edges) {
            if (edge.containsEndpoint(start))
                continue;
            if (GeometryMath.edgeIntersect(start, pointInf, edge)) {
                if (GeometryMath.onSegment(start, edge.p1, pointInf))
                    continue;
                if (GeometryMath.onSegment(start, edge.p2, pointInf))
                    continue;
                openEdges.insert(start, pointInf, edge);
            }
        }

        List<Point> visible = new ArrayList<>();

        // begin the clockwise scan
        Point prev = null;
        boolean prevVisible = false;

        for (Point p : points) {
            if (p.equals(start))
                continue;

            for (Edge edge : environment.getIncidentEdges(p)) {
                if (GeometryMath.ccw(start, p, edge.getOtherVertex(p)) == -1) {
                    openEdges.delete(start, p, edge);
                }
            }

            boolean isVisible = false;
            if (prev == null || GeometryMath.ccw(start, prev, p) != 0 || !GeometryMath.onSegment(start, prev, p)) {
                if (openEdges.size() == 0) {
                    isVisible = true;
                } else if (!GeometryMath.edgeIntersect(start, p, openEdges.smallest())) {
                    isVisible = true;
                }
            } else if (!prevVisible) {
                isVisible = false;
            } else {
                isVisible = true;
                for (Edge edge : openEdges) {
                    if (!edge.containsEndpoint(prev) && GeometryMath.edgeIntersect(prev, p, edge)) {
                        isVisible = false;
                        break;
                    }
                }

                if (isVisible && GeometryMath.edgeInPolygon(prev, p, environment)) {
                    isVisible = false;
                }
            }

            if (isVisible && !environment.getAdjacentPoints(start).contains(p)) {
                isVisible = !GeometryMath.edgeInPolygon(start, p, environment);
            }

            if (isVisible) {
                visible.add(p);
            }

            for (Edge edge : environment.getIncidentEdges(p)) {
                if (!edge.containsEndpoint(start) && GeometryMath.ccw(start, p, edge.getOtherVertex(p)) == 1) {
                    openEdges.insert(start, p, edge);
                }
            }

            prev = p;
            prevVisible = isVisible;
        }

        return visible;
    }
}
