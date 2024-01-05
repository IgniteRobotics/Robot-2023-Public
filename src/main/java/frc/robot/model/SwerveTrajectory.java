package frc.robot.model;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;

public class SwerveTrajectory extends Trajectory {
    public final List<Rotation2d> interpolatedRotation;

    public SwerveTrajectory(List<State> states, List<Rotation2d> interpolatedRotation) {
        super(states);
        this.interpolatedRotation = interpolatedRotation;
    }

    public Rotation2d getTimeParameterizedRotation(double seconds) {
        double d = (seconds / this.getTotalTimeSeconds()) * interpolatedRotation.size();
        int i = Math.max(Math.min((int) Math.round(d), interpolatedRotation.size() - 1), 0);

        return interpolatedRotation.get(i);
    }

    public List<Rotation2d> getInterpolatedRotation() {
        return interpolatedRotation;
    }
}
