package frc.robot.comm.reporting;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ReportingString extends ReportingType<String> {
    public ReportingString(String name, ReportingLevel level) {
        super(name, level);
    }

    @Override
    public void put() {
        SmartDashboard.putString(name, value);
    }
}
