package com.igniterobotics.jvisibility.model;

import java.util.*;
import java.util.stream.Collectors;

import com.igniterobotics.jvisibility.VisibilityGraphGenerator;
import com.igniterobotics.jvisibility.geometry.Point;

public class VisibilityGraph {
    private Environment environment;
    private Map<Point, List<Point>> map = new HashMap<>();

    private VisibilityGraph(Environment environment) {
        this.environment = environment;
    }

    public static VisibilityGraph  generate(Environment environment, Point start, Point end) {
        VisibilityGraph graph = new VisibilityGraph(environment);

        for (Point point : environment.getPoints()) {
            List<Point> visible = VisibilityGraphGenerator.visibleVertices(point, end, environment);
            graph.addVisible(point, visible);
        }

        if(start != null) graph.addVisible(start, VisibilityGraphGenerator.visibleVertices(start, end, environment));
        if(end != null) graph.addVisible(end, VisibilityGraphGenerator.visibleVertices(end, end, environment));

        return graph;
    }

    public void addVisible(Point point, List<Point> visiblePoints) {
        map.put(point, visiblePoints);
    }

    public List<Point> getVisible(Point p) {
        if(map.get(p) == null) return new ArrayList<>();
        return map.get(p);
    }

    public boolean isVisible(Point from, Point to) {
        if(map.get(from) == null) return false;
        return map.get(from).contains(to);
    }

    public Map<WeightedPoint, WeightedPoint> dijkstras(Point startNode) {
        Map<Point, Double> vectorWeights = new HashMap<>();
        Queue<WeightedPoint> open = new PriorityQueue<>(new Comparator<WeightedPoint>() {
            public int compare(WeightedPoint arg0, WeightedPoint arg1) {
                if(arg0.weight < arg1.weight) return -1;
                else if(arg0.weight > arg1.weight) return 1;
                else return 0;
            };
        });
        Set<WeightedPoint> closed = new HashSet<>();
        Map<WeightedPoint, WeightedPoint> prev = new HashMap<>();

        open.add(new WeightedPoint(startNode.x, startNode.y, 0));
        while(!open.isEmpty()) {
            // THE WEIGHT HERE IS FROM startNode to V!!!
            WeightedPoint v = open.remove();
            closed.add(v);

            List<WeightedPoint> adjacentVertices = getVisible(v).stream().map(point -> new WeightedPoint(point.x, point.y, Point.dist(v, point))).collect(Collectors.toList());
            for(WeightedPoint adj : adjacentVertices) {
                if(closed.contains(adj)) continue;
                // REMEMBER: the weight here is from v to adj, where v is a node in the graph, not necessarily the startNode!!!
                double d = v.weight + adj.weight;
                if(!vectorWeights.containsKey(adj) || d < vectorWeights.get(adj)) {
                    vectorWeights.put(adj, d);
                    prev.put(adj, v);
                }

                WeightedPoint adjStartWeight = new WeightedPoint(adj.x, adj.y, d);
                open.add(adjStartWeight);
            }
        }

        return prev;
    }

    public List<Point> shortestPath(Point start, Point end) {
        Map<WeightedPoint, WeightedPoint> dijkstrasResult = dijkstras(start);
        Deque<WeightedPoint> stack = new LinkedList<>();

        WeightedPoint current = new WeightedPoint(end.x, end.y, 0);
        while(current != null) {
            stack.push(current);
            current = dijkstrasResult.get(current);
        }

        List<Point> result = new ArrayList<>();
        while(!stack.isEmpty()) {
            result.add(stack.remove());
        }

        return result;
    }

    private void addVisible(Point p) {
        addVisible(p, VisibilityGraphGenerator.visibleVertices(p, null, environment));
    }

    private void remove(Point p) {
        map.remove(p);
    }

    public Collection<Point> getPoints() {
        return map.keySet();
    }
}
