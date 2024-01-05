package com.igniterobotics.jvisibility.geometry;

public class Edge {
    public final Point p1;
    public final Point p2;

    public Edge(Point p1, Point p2) {
        this.p1 = p1;
        this.p2 = p2;
    }

    public boolean containsEndpoint(Point p) {
        return p.equals(p1) || p.equals(p2);
    }

    public Point getOtherVertex(Point p) {
        if(p.equals(p1)) {
            return p2;
        } else {
            return p1;
        }
    }

    @Override
    public boolean equals(Object obj) {
        if(!(obj instanceof Edge)) return false;
        Edge other = (Edge) obj;

        return (other.p1.equals(this.p1) && other.p2.equals(this.p2)) || (other.p1.equals(this.p2) && other.p2.equals(this.p1));
    }
}
