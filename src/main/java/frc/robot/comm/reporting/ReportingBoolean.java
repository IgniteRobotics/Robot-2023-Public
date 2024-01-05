package frc.robot.comm.reporting;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ReportingBoolean extends ReportingType<Boolean> {
    public ReportingBoolean(String name, ReportingLevel level) {
        super(name, level);
    }

    @Override
    public void put() {
        SmartDashboard.putBoolean(name, value);
    }
}
