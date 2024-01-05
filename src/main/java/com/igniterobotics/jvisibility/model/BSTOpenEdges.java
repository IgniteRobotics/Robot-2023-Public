package com.igniterobotics.jvisibility.model;

import java.util.Deque;
import java.util.Iterator;
import java.util.LinkedList;

import com.igniterobotics.jvisibility.geometry.Edge;
import com.igniterobotics.jvisibility.geometry.GeometryMath;
import com.igniterobotics.jvisibility.geometry.Point;

public class BSTOpenEdges implements OpenEdges {
    public static class Node {
        public Edge data;
        public Node left;
        public Node right;

        public Node(Edge edge) {
            this.data = edge;
        }
    }

    private Node root;
    private int size = 0;

    public void insert(Point p1, Point p2, Edge edge) {
        double dist = GeometryMath.pointEdgeDistance(p1, p2, edge);
        root = insert(p1, p2, edge, dist, root);
        size += 1;
    }

    private boolean lessThan(Point p1, Point p2, Edge edge1, Edge edge2) {
        if(edge1.equals(edge2)) return false;
        if(!GeometryMath.edgeIntersect(p1, p2, edge2)) return true;
        double edge1Dist = GeometryMath.pointEdgeDistance(p1, p2, edge1);
        double edge2Dist = GeometryMath.pointEdgeDistance(p1, p2, edge2);
        if(edge1Dist > edge2Dist) return false;
        if(edge1Dist < edge2Dist) return true;
        
        Point samePoint;
        if(edge2.containsEndpoint(edge1.p1)) {
            samePoint = edge1.p1;
        } else {
            samePoint = edge1.p2;
        }

        double angleEdge1 = GeometryMath.angle2(p1, p2, edge1.getOtherVertex(samePoint));
        double angleEdge2 = GeometryMath.angle2(p1, p2, edge2.getOtherVertex(samePoint));
        if(angleEdge1 < angleEdge2) return true;
        return false;
    }

    private Node insert(Point p1, Point p2, Edge edge, double dist, Node n) {
        if(n == null) {
            return new Node(edge);
        } else {
            double cmpDist = GeometryMath.pointEdgeDistance(p1, p2, n.data);
            if(lessThan(p1, p2, n.data, edge)) {
                n.right = insert(p1, p2, edge, dist, n.right);
            } else {
                n.left = insert(p1, p2, edge, dist, n.left);
            }
        }

        return n;
    }

    @Override
    public void delete(Point p1, Point p2, Edge edge) {
        this.root = delete(p1, p2, edge, root);
    }

    @Override
    public Edge smallest() {
        return smallest(root).data;
    }

    private Node smallest(Node n) {
        while(n.left != null) {
            n = n.left;
        }
        return n;
    }

    private Node delete(Point p1, Point p2, Edge edge, Node node) {
        if(node == null || edge == null) return null;
        double dist = GeometryMath.pointEdgeDistance(p1, p2, edge);

        double cmpDist = GeometryMath.pointEdgeDistance(p1, p2, node.data);
        if(node.data.equals(edge)) {
            size -= 1;
            if(node.left == null) {
                return node.right;
            } else if(node.right == null) {
                return node.left;
            } else {
                node.data = smallest(node.right).data;
                node.right = delete(p1, p2, node.data, node.right);
                size += 1;
            }
        } else if(lessThan(p1, p2, node.data, edge)) {
            node.right = delete(p1, p2, edge, node.right);
        } else {
            node.left = delete(p1, p2, edge, node.left);
        }

        return node;
    }

    public int size() {
        return size;
    }

    @Override
    public Iterator<Edge> iterator() {
        return new BSTIterator();
    }

    private class BSTIterator implements Iterator<Edge> {
        private Deque<Node> nodeStack = new LinkedList<>();

        public BSTIterator() {
            Node n = BSTOpenEdges.this.root;
            while(n != null) {
                nodeStack.push(n);
                n = n.left;
            }
        }

        @Override
        public boolean hasNext() {
            return !nodeStack.isEmpty();
        }

        @Override
        public Edge next() {
            Node n = nodeStack.pop();
            Edge item = n.data;
            if(n.right != null) {
                Node o = n.right;
                while(o != null) {
                    nodeStack.push(o);
                    o = o.left;
                }
            }

            return item;
        }
    }

    public Node getRoot() {
        return root;
    }
}