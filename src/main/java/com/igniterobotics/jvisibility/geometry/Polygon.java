package com.igniterobotics.jvisibility.geometry;

import java.util.ArrayList;
import java.util.List;

import com.igniterobotics.jvisibility.geometry.Edge;
import com.igniterobotics.jvisibility.geometry.Point;

public class Polygon {
    private int id;
    private List<Edge> edges = new ArrayList<>();
    private final Point[] points;

    public Polygon(Point[] points) {
        this(01, points);
    }

    public Polygon(double[] pointCoords) {
        this.points = new Point[pointCoords.length / 2];
        for(int i = 0; i < pointCoords.length / 2; i++) {
            this.points[i] = new Point(pointCoords[2 * i], pointCoords[2 * i + 1]);
        }

        for(int i = 0; i < points.length - 1; i++) {
            points[i].setPolygonId(id);
            points[i + 1].setPolygonId(id);
            Edge e = new Edge(points[i], points[i + 1]);
            edges.add(e);
        }
        edges.add(new Edge(points[points.length - 1], points[0]));
    }

    public Polygon(int id, Point[] points) {
        this.points = points;
        for(int i = 0; i < points.length - 1; i++) {
            points[i].setPolygonId(id);
            points[i + 1].setPolygonId(id);
            Edge e = new Edge(points[i], points[i + 1]);
            edges.add(e);
        }
        edges.add(new Edge(points[points.length - 1], points[0]));
    }

    public List<Edge> getEdges() {
        return edges;
    }

    public void setId(int id) {
        this.id = id;
        for(Point p : points) {
            p.setPolygonId(id);
        }
    }

    public int getId() {
        return id;
    }
}