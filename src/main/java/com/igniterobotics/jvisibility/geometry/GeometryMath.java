package com.igniterobotics.jvisibility.geometry;

import com.igniterobotics.jvisibility.model.Environment;

public class GeometryMath {
    private static double toleranceExp = 25;
    private static int tInt = (int) Math.pow(10, toleranceExp); 
    private static double tDouble = Math.pow(10, toleranceExp);

    /**
     * Returns the intersection point of line P1 and P2 with Edge edge
     * 
     * IMPORTANT: This method makes the assumption that the intersection point lies
     * on edge
     * 
     * @param p1   Start point of line
     * @param p2   End point of line
     * @param edge Intersection edge
     * @return
     */
    public static Point intersectPoint(Point p1, Point p2, Edge edge) {
        if (edge.containsEndpoint(p1))
            return p1;
        if (edge.containsEndpoint(p2))
            return p2;

        double pslope;
        double eslope;

        // if edge is parallel to the y axis
        if (edge.p1.x == edge.p2.x) {
            // if half line is also parallel to the y axis, there is no intersection. we do
            // not consider overlapping lines
            if (p1.x == p2.x)
                return null;
            pslope = (p1.y - p2.y) / (p1.x - p2.x);
            double intersectX = edge.p1.x;
            double intersectY = pslope * (intersectX - p1.x) + p1.y;
            return new Point(intersectX, intersectY);
        } else if (p1.x == p2.x) {
            // if the line is vertical, we need to compute the edge's intersection with the
            // line. (slope will be zero)
            eslope = (edge.p1.y - edge.p2.y) / (edge.p1.x - edge.p2.x);
            double intersectX = p1.x;
            double intersectY = eslope * (intersectX - edge.p1.x) + edge.p1.y;
            return new Point(intersectX, intersectY);
        }

        // otherwise, slopes are safe to compute
        pslope = (p1.y - p2.y) / (p1.x - p2.x);
        eslope = (edge.p1.y - edge.p2.y) / (edge.p1.x - edge.p2.x);
        if (pslope == eslope)
            return null; // parallel lines do not intersect lol
        double intersectX = (eslope * edge.p1.x - pslope * p1.x + p1.y - edge.p1.y) / (eslope - pslope);
        double intersectY = eslope * (intersectX - edge.p1.x) + edge.p1.y;

        return new Point(intersectX, intersectY);
    }

    public static double angle2(Point a, Point b, Point c) {
        double A = Math.pow(c.x - b.x, 2) + Math.pow(c.y - b.y, 2);
        double B = Math.pow(c.x - a.x, 2) + Math.pow(c.y - a.y, 2);
        double C = Math.pow(b.x - a.x, 2) + Math.pow(b.y - a.y, 2);
        double cos = (A + C - B) / (2 * Math.sqrt(A) * Math.sqrt(C));
        return Math.acos((int) (cos*tInt)/tDouble);  
    }

    /**
     * Returns the euclidian distance from p1 to the intersect point of line
     * (p1->p2) with edge
     * 
     * @param p1   Start point of line
     * @param p2   End point of line
     * @param edge Intersection edge
     * @return distance
     */
    public static double pointEdgeDistance(Point p1, Point p2, Edge edge) {
        Point intersection = intersectPoint(p1, p2, edge);
        if (intersection != null)
            return Point.dist(p1, intersection);
        return 0;
    }

    /**
     * Given colinear points p, q, r, checks if q lies on line segment pr
     * 
     * @param p
     * @param q
     * @param r
     * @return
     */
    public static boolean onSegment(Point p, Point q, Point r) {
        return q.x <= Math.max(p.x, r.x) && q.x >= Math.min(p.x, r.x) && q.y <= Math.max(p.y, r.y)
                && q.y >= Math.min(p.y, r.y);
    }

    /**
     * Converts an angle that is in the range [-PI, PI] to one that is in the range
     * [0, 2PI]
     * 
     * @param angle Angle in the range [-PI, PI]
     * @return Angle in the range [0, 2PI]
     */
    public static double negativePiTo2Pi(double angle) {
        if (angle < 0) {
            return angle + Math.PI * 2;
        } else {
            return angle;
        }
    }

    /**
     * Return whether line CA is counter-clockwise of line BA
     * 
     * @param A
     * @param B
     * @param C
     * @return
     */
    public static int ccw(Point A, Point B, Point C) {
        // we find the determinant
        double det = (int) (((B.x - A.x) * (C.y - A.y) - (B.y - A.y) * (C.x - A.x)) * tInt) / tDouble;
        if (det > 0)
            return 1;
        if (det < 0)
            return -1;
        return 0;
    }

    /**
     * Returns whether the edge formed by p1 and q1 intersects edge
     * 
     * @param p1
     * @param q1
     * @param edge
     * @return
     */
    public static boolean edgeIntersect(Point p1, Point q1, Edge edge) {
        Point p2 = edge.p1;
        Point q2 = edge.p2;
        int o1 = ccw(p1, q1, p2);
        int o2 = ccw(p1, q1, q2);
        int o3 = ccw(p2, q2, p1);
        int o4 = ccw(p2, q2, q1);

        if (o1 != o2 && o3 != o4) {
            return true;
        }
        if (o1 == 0 && onSegment(p1, p2, q1)) {
            return true;
        }
        if (o2 == 0 && onSegment(p1, q2, q1)) {
            return true;
        }
        if (o3 == 0 && onSegment(p2, p1, q2)) {
            return true;
        }
        if (o4 == 0 && onSegment(p2, q1, q2)) {
            return true;
        }

        return false;
    }

    /**
     * Checks whether the given point p is within a polygon
     * @param p1
     * @param polygon
     * @return true if p is within polygon, false otherwise
     */
    public static boolean isPointInPolygon(Point p1, Polygon polygon) {
        Point p2 = new Point(Double.MAX_VALUE, p1.y);
        int intersectCount = 0;
        for(Edge edge : polygon.getEdges()) {
            if(p1.y < edge.p1.y && p1.y < edge.p2.y) continue;
            if(p1.y > edge.p1.y && p1.y > edge.p2.y) continue;
            if(p1.x > edge.p1.x && p1.x > edge.p2.x) continue;
            boolean isEdgeP1Colinear = ccw(p1, edge.p1, p2) == 0;
            boolean isEdgeP2Colinear = ccw(p1, edge.p2, p2) == 0;
            if(isEdgeP1Colinear && isEdgeP2Colinear) continue;
            if(isEdgeP1Colinear || isEdgeP2Colinear) {
                Point colinearPoint = isEdgeP1Colinear ? edge.p1 : edge.p2;
                if(edge.getOtherVertex(colinearPoint).y > p1.y) {
                    intersectCount += 1;
                }
            } else if(edgeIntersect(p1, p2, edge)) {
                intersectCount += 1;
            }
        }

        if(intersectCount % 2 == 0) return false;
        return true;
    }

    public static boolean edgeInPolygon(Point p1, Point p2, Environment env) {
        if(p1.getPolygonId() != p2.getPolygonId()) return false;
        if(p1.getPolygonId() == -1 || p2.getPolygonId() == -1) return false;
    
        Point midPoint = new Point((p1.x + p2.x) / 2.0, (p1.y + p2.y) / 2.0);
        return isPointInPolygon(midPoint, env.getPolygonById(p1.getPolygonId()));
    }
}
