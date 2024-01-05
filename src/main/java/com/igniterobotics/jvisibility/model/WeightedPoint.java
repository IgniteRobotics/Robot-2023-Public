package com.igniterobotics.jvisibility.model;

import com.igniterobotics.jvisibility.geometry.Point;

public class WeightedPoint extends Point {
    public final double weight;

    public WeightedPoint(double x, double y, double weight) {
        super(x, y);
        this.weight = weight;
    }
}
