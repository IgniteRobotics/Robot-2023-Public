package frc.robot.comm.reporting;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ReportingNumber extends ReportingType<Double> {
    public ReportingNumber(String name, ReportingLevel level) {
        super(name, level);
    }

    @Override
    public void put() {
        SmartDashboard.putNumber(name, value);
    }
}
