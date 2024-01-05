package com.igniterobotics.jvisibility.geometry;

import java.util.Objects;

public class Point {
    private int polygonId = -1;
    public double x;
    public double y;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public static double dist2(Point p1, Point p2) {
        return Math.pow(p1.x - p2.x, 2) + Math.pow(p1.y - p2.y, 2);
    }

    public static double dist(Point p1, Point p2) {
        return Math.sqrt(dist2(p1, p2));
    }

    public int getPolygonId() {
        return polygonId;
    }

    public void setPolygonId(int polygonId) {
        this.polygonId = polygonId;
    }

    @Override
    public boolean equals(Object obj) {
        if(!(obj instanceof Point)) return false;
        Point other = (Point) obj;
        return other.x == this.x && other.y == this.y;
    }

    @Override
    public int hashCode() {
        return Objects.hash(this.x, this.y);
    }
}
