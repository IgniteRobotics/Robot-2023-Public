package frc.robot.trajectory;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.PoseWithCurvature;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer;
import edu.wpi.first.math.trajectory.Trajectory.State;
import frc.robot.model.SwerveTrajectory;

/**
 * @deprecated don't use this, math is suspicious
 */
public class CubicPathInterpolator {
    public static final double epsilon = 0.1;
    public static final double derivativeClamp = 0.6;

    /**
     * For x in the interval [x0, xF], return the corresponding y coordinate of the
     * cubic interpolated curve.
     * 
     * @param p An array of 4 points p1, p2, p3, p4. Interpolation is done between
     *          p2 and p3.
     * @param t Position to interpolate at
     * @return
     */
    private static double getValue(double[] p, double t) {
        /*
         * From f(x) and f'(x), where f is a cubic polynomial
         * 
         * a = 2f(0) - 2f(1) + f'(0) + f'(1)
         * b = -3f(0) + 3f(1) - 2f'(0) - f'(1)
         * c = f'(0)
         * d = f(0)
         * 
         * let f'(0) = (p_2 - p_0) / 2 and f'(1) = (p_3 - p_1) / 2 for smoother curves.
         */

        double df_dt_0 = MathUtil.clamp((p[2] - p[0]) / 2, -derivativeClamp, derivativeClamp);
        double df_dt_1 = MathUtil.clamp((p[3] - p[1]) / 2, -derivativeClamp, derivativeClamp);

        double a = 2 * p[1] - 2 * p[2] + df_dt_0 + df_dt_1;
        double b = -3 * p[1] + 3 * p[2] - 2 * df_dt_0 - df_dt_1;
        double c = df_dt_0;
        double d = p[1];

        return a * Math.pow(t, 3) + b * Math.pow(t, 2) + c * t + d;
    }

    /**
     * Computes values necessary for path interpolation
     * Places values into double[] results, where results[0] is curvature, results[1] is heading, and results[2], results[3] is the parameterized value
     */
    private static void computeValues(double[] x, double[] y, double t, double[] results) {
        /*
         * f(0) = p_1
         * f(1) = p_2
         */
        double df_dt_0_x = MathUtil.clamp((x[2] - x[0]) / 2, -derivativeClamp, derivativeClamp);
        double df_dt_1_x = MathUtil.clamp((x[3] - x[1]) / 2, -derivativeClamp, derivativeClamp);

        double a = 2 * x[1] - 2 * x[2] + df_dt_0_x + df_dt_1_x;
        double b = -3 * x[1] + 3 * x[2] - 2 * df_dt_0_x - df_dt_1_x;
        double c = df_dt_0_x;
        double d = x[1];

        double df_dt_0_y = MathUtil.clamp((y[2] - y[0]) / 2, -1, 1);
        double df_dt_1_y = MathUtil.clamp((y[3] - y[1]) / 2, -1, 1);

        double e = 2 * y[1] - 2 * y[2] + df_dt_0_y + df_dt_1_y;
        double f = -3 * y[1] + 3 * y[2] - 2 * df_dt_0_y - df_dt_1_y;
        double g = df_dt_0_y;
        double h = y[1];

        double A = (3 * a * t * t + 2 * b * t + c);
        double B = (3 * e * t * t + 2 * f * t + g);
        double C = (6 * a * t + 2 * b);
        double D = (6 * e * t + 2 * f);

        results[0] = Math.abs(A * D - B * C) / Math.pow(Math.sqrt(A * A + B * B), 3);
        results[1] = Math.atan2(B, A);
        results[2] = a * Math.pow(t, 3) + b * Math.pow(t, 2) + c * t + d;
        results[3] = e * Math.pow(t, 3) + f * Math.pow(t, 2) + g * t + h;
    }

    private static List<PoseWithCurvature> generatePosesWithCurvatures(List<Pose2d> waypoints, List<Rotation2d> interpolatedHeadings) {
        waypoints = new LinkedList<>(waypoints);
        int size = waypoints.size();
        if (size < 2)
            return Collections.emptyList();

        // Generate "imaginary" beginning and end points for interpolating first and
        // last interval
        // Let the endpoint be in the middle of a line between the imaginary point and
        // the point next to the endpoint
        Pose2d imStart = new Pose2d(new Translation2d(2 * waypoints.get(0).getX() - waypoints.get(1).getX(),
                2 * waypoints.get(0).getY() - waypoints.get(1).getY()), new Rotation2d(2 * waypoints.get(0).getRotation().getRadians() - waypoints.get(1).getRotation().getRadians()));
                Pose2d imEnd = new Pose2d(new Translation2d(2 * waypoints.get(size - 2).getX() - waypoints.get(size - 1).getX(),
                2 * waypoints.get(size - 2).getY() - waypoints.get(size - 1).getY()), new Rotation2d(2 * waypoints.get(size - 2).getRotation().getRadians() - waypoints.get(size - 1).getRotation().getRadians()));

        waypoints.add(0, imStart);
        waypoints.add(imEnd);

        List<PoseWithCurvature> poseWithCurvature = new ArrayList<>();

        for (int i = 1; i < waypoints.size() - 2; ++i) {
            Pose2d prev = waypoints.get(i - 1);
            Pose2d start = waypoints.get(i);
            Pose2d end = waypoints.get(i + 1);
            Pose2d next = waypoints.get(i + 2);

            double t = 0;

            double[] dataValues = new double[4];
            while (t < 1) {
                // perform interpolation between points
                double[] xPoints = new double[] { prev.getX(), start.getX(), end.getX(), next.getX() };
                double[] yPoints = new double[] { prev.getY(), start.getY(), end.getY(), next.getY() };
                double[] thetaPoints = new double[] { prev.getRotation().getRadians(), start.getRotation().getRadians(), end.getRotation().getRadians(), next.getRotation().getRadians() };
                
                computeValues(xPoints, yPoints, t, dataValues);
                double x = dataValues[2];
                double y = dataValues[3];

                poseWithCurvature.add(new PoseWithCurvature(new Pose2d(x, y, new Rotation2d(dataValues[1])), dataValues[0]));
                interpolatedHeadings.add(new Rotation2d(getValue(thetaPoints, t)));

                t += epsilon;
            }
        }

        return poseWithCurvature;
    }

    public static SwerveTrajectory generateCubicTrajectory(List<Pose2d> waypoints, TrajectoryConfig config) {
        List<Rotation2d> interpolatedHeadings = new ArrayList<>();

        Trajectory trajectory = TrajectoryParameterizer.timeParameterizeTrajectory(
                generatePosesWithCurvatures(waypoints, interpolatedHeadings),
                config.getConstraints(),
                config.getStartVelocity(),
                config.getEndVelocity(),
                config.getMaxVelocity(),
                config.getMaxAcceleration(),
                config.isReversed());

        return new SwerveTrajectory(trajectory.getStates(), interpolatedHeadings);
    }

    /*
     * Determining curvature from the cubic path
     * 
     * Let r(t) = (at^3 + bt^2 + ct + d)i + (et^3 + ft^2 + gt + h)j be the vector
     * representing our interpolated path between waypoints w_n and w_(n+1).
     * Then, r'(t) = (3at^2 + 2bt + c)i + (3et^2 + 2ft + g)j, r''(t) = (6at + 2b)i +
     * (6et + 2f)j, and |r'(t)| = sqrt((3at^2 + 2bt + c)^2 + (3et^2 + 2ft + g)^2)
     * let A = (3at^2 + 2bt + c), B = (3et^2 + 2ft + g), C = (6at + 2b), D = (6et +
     * 2f)
     * 
     * Then |r'(t) x r''(t)| = AD - BC, and curvature k = (AD - BC) / |r'(t)|^3
     * 
     * I worry that this may be too computationally expensive.
     */
}
