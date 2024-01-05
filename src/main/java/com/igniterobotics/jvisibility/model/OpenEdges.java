package com.igniterobotics.jvisibility.model;

import com.igniterobotics.jvisibility.geometry.Edge;
import com.igniterobotics.jvisibility.geometry.Point;

public interface OpenEdges extends Iterable<Edge> {
    /**
     * Insert an edge into the OpenEdges structure. Edges are ordered by order of intersection in the half line formed by p1 to p2.
     * @param p1 Starting point of half line
     * @param p2 Ending point of half line
     * @param edge
     */
    void insert(Point p1, Point p2, Edge edge);

    /**
     * Deletes the specified edge from the OpenEdges structure
     * @param edge Edge to remove from the structure
     */
    void delete(Point p1, Point p2, Edge edge);

    /**
     * Returns the smallest edge in this structure
     * @return The smallest edge
     */
    Edge smallest();

    /**
     * Returns the size of the structure
     * @return The size of the structure
     */
    int size();
}
