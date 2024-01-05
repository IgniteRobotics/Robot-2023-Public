package frc.robot.trajectory;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.StringJoiner;
import java.util.stream.Collectors;

import org.json.JSONArray;
import org.json.JSONObject;

import com.igniterobotics.jvisibility.geometry.Point;
import com.igniterobotics.jvisibility.geometry.Polygon;
import com.igniterobotics.jvisibility.model.Environment;
import com.igniterobotics.jvisibility.model.VisibilityGraph;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.RawPublisher;
import edu.wpi.first.networktables.RawTopic;
import edu.wpi.first.wpilibj.DriverStation;

public class PathGenerator {
    private Environment environment;
    private NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private NetworkTable trajectoryTable = ntInstance.getTable("trajectory");

    private DoubleArrayTopic visualizerPointTopic = trajectoryTable.getDoubleArrayTopic("visualizerPoint");
    private DoubleArraySubscriber visualizerPointTopicSubscriber = visualizerPointTopic.subscribe(new double[2]);


    private static PathGenerator instance;

    public static PathGenerator getInstance() {
        if(instance == null) {
            instance = new PathGenerator();
        }

        return instance;
    }

    public void initialize() {
        try {
            loadEnvironment();
        } catch(IOException e) {
            DriverStation.reportError("Failed to initialize PathGenerator environment", e.getStackTrace());
        }
    }

    private void loadEnvironment() throws IOException {
        environment = new Environment();

        BufferedReader fieldDataStream = new BufferedReader(
                new InputStreamReader(PathGenerator.class.getResourceAsStream("/2023-field.json")));
        StringJoiner jsonBuilder = new StringJoiner("\n");
        String line;
        while ((line = fieldDataStream.readLine()) != null) {
            jsonBuilder.add(line);
        }

        JSONObject fieldData = new JSONObject(jsonBuilder.toString());
        JSONArray fieldObstacles = fieldData.getJSONArray("obstacles");
        for (int i = 0; i < fieldObstacles.length(); ++i) {
            JSONObject obstacle = fieldObstacles.getJSONObject(i);
            JSONArray obstaclePoints = obstacle.getJSONArray("points");
            double buffer = 0; // no buffer, define buffers manually

            double[] polygonPoints = new double[obstaclePoints.length()];
            double xCenter = 0;
            double yCenter = 0;

            for (int j = 0; j < obstaclePoints.length(); j += 2) {
                xCenter += obstaclePoints.getDouble(j);
                yCenter += obstaclePoints.getDouble(j + 1);
            }

            xCenter /= obstaclePoints.length() / 2;
            yCenter /= obstaclePoints.length() / 2;

            for (int j = 0; j < obstaclePoints.length(); j += 2) {
                double x = obstaclePoints.getDouble(j);
                double y = obstaclePoints.getDouble(j + 1);

                polygonPoints[j] = x + buffer * Math.signum(x - xCenter);
                polygonPoints[j + 1] = y + buffer * Math.signum(y - yCenter);
            }

            Polygon polygon = new Polygon(polygonPoints);
            environment.addPolygon(polygon);
        }
    }

    private void publishGraph(VisibilityGraph graph) {
        // compress data
        List<Float> serialized = new ArrayList<>();
        List<Point> points = new ArrayList<>(graph.getPoints());
        serialized.add(Float.valueOf(graph.getPoints().size()));

        for(Point p : points) {
            serialized.add((float) p.x);
            serialized.add((float) p.y);
        }

        for(Point p : points) {
            List<Point> visible = graph.getVisible(p);
            serialized.add(Float.valueOf(visible.size()));
            for(Point v : visible) {
                serialized.add(Float.valueOf(points.indexOf(v)));
            }
        }

        ByteBuffer buf = ByteBuffer.allocate(serialized.size() * 4);
        for(Float d : serialized) {
            buf.putFloat(d);
        }
    }

    public void publishTrajectory(Trajectory trajectory) {
        if (trajectory == null)
            return;

        List<State> states = trajectory.getStates();
        List<State> filteredStates = new ArrayList<>();
        int modulo = Math.max(1, states.size() / 70);
        for(int i = 0; i < states.size(); ++i) {
            if(i % modulo == 0) {
                filteredStates.add(states.get(i));
            }
        }

        double[] stateXY = new double[filteredStates.size() * 2];
        for (int i = 0; i < filteredStates.size(); i += 1) {
            if(i * 2 + 1 >= stateXY.length) break;
            State s = filteredStates.get(i);
            stateXY[i * 2] = s.poseMeters.getX();
            stateXY[i * 2 + 1] = s.poseMeters.getY();
        }
    }

    private PathPoint mapPoint(List<Translation2d> points, int index) {
        Translation2d point = points.get(index);

        if(index == 0) {
            Translation2d next = points.get(index + 1);
            double dX = next.getX() - points.get(index).getX();
            double dY = next.getY() - points.get(index).getY();

            Translation2d imPrev = new Translation2d(point.getX() - dX, point.getY() - dY);
            return new PathPoint(point, new Rotation2d(Math.atan2(next.getY() - imPrev.getY(), next.getX() - imPrev.getX())));
        } else if(index == points.size() - 1) {
            Translation2d prev = points.get(index - 1);
            double dX = point.getX() - prev.getX();
            double dY = point.getY() - prev.getY();

            Translation2d imNext = new Translation2d(point.getX() + dX, point.getY() + dY);
            return new PathPoint(point, new Rotation2d(Math.atan2(imNext.getY() - prev.getY(), imNext.getX() - prev.getX())));
        } else {
            Translation2d prev = points.get(index - 1);
            Translation2d next = points.get(index + 1);
            return new PathPoint(point, new Rotation2d(Math.atan2(next.getY() - prev.getY(), next.getX() - prev.getX())));
        }
    }

    private PathPlannerTrajectory generateTrajectoryFromPoints(List<Translation2d> points) {
        List<PathPoint> pathPoints = new ArrayList<>();

        for (int i = 0; i < points.size(); ++i) {
            pathPoints.add(mapPoint(points, i));
        }

        PathPoint lastPoint = pathPoints.get(pathPoints.size() - 1);
        // pathPoints.set(pathPoints.size() - 1, new PathPoint(lastPoint.position, lastPoint.heading, new Rotation2d(Math.PI)));
        // pathPoints.get(pathPoints.size() - 1).holonomicRotation = new Rotation2d(Math.PI); 

        PathPlannerTrajectory trajectory = PathPlanner.generatePath(new PathConstraints(2, 2), pathPoints);

        return trajectory;
    }

    private List<Translation2d> generateAvoidancePoints(Translation2d start, Translation2d end) {
        Point startPoint = new Point(start.getX(), start.getY());
        Point endPoint = new Point(end.getX(), end.getY());

        System.out.println("Generating Dynamic Trajectory");
        VisibilityGraph graph = VisibilityGraph.generate(environment, startPoint, endPoint);
        List<Point> shortestPath = graph.shortestPath(startPoint, endPoint);
        // shortestPath.add(0, startPoint);

        return shortestPath.stream().map((point) -> new Translation2d(point.x, point.y)).collect(Collectors.toList());
    }

    public PathPlannerTrajectory shortestAvoidancePath(Translation2d start, Translation2d end) {
        List<Translation2d> points = List.of(start, end);
        List<PathPoint> pathPoints = new ArrayList<>();

        for (int i = 0; i < points.size(); ++i) {
            pathPoints.add(mapPoint(points, i));
        }

        return PathPlanner.generatePath(new PathConstraints(2, 2), pathPoints.get(0), new PathPoint(pathPoints.get(1).position, pathPoints.get(1).heading, new Rotation2d()));
    }

    public PathPlannerTrajectory toDestination(Pose2d robotCurrentPose, Translation2d location) {
        List<Translation2d> points = List.of(robotCurrentPose.getTranslation(), location);
        List<PathPoint> pathPoints = new ArrayList<>();

        for (int i = 0; i < points.size(); ++i) {
            pathPoints.add(mapPoint(points, i));
        }

        return PathPlanner.generatePath(new PathConstraints(2, 2), pathPoints.get(0), new PathPoint(pathPoints.get(1).position, pathPoints.get(1).heading, new Rotation2d()));
    }

    public PathPlannerTrajectory toDestination(Pose2d robotCurrentPose, Translation2d location, Rotation2d endHeading) {
        List<Translation2d> points = List.of(robotCurrentPose.getTranslation(), location);
        List<PathPoint> pathPoints = new ArrayList<>();

        for (int i = 0; i < points.size(); ++i) {
            pathPoints.add(mapPoint(points, i));
        }

        return PathPlanner.generatePath(new PathConstraints(2, 2), new PathPoint(pathPoints.get(0).position, pathPoints.get(0).heading, robotCurrentPose.getRotation()), new PathPoint(pathPoints.get(1).position, pathPoints.get(1).heading, endHeading));
    }

    public Translation2d getFieldVisualizerPoint() {
        double[] d = visualizerPointTopicSubscriber.get();
        return new Translation2d(d[0], d[1]);
    }
}
