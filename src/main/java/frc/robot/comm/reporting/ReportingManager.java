package frc.robot.comm.reporting;

public class ReportingManager {
    private static ReportingLevel reportingLevel = ReportingLevel.DEBUG;

    public static void setReportingLevel(ReportingLevel reportingLevel) {
        ReportingManager.reportingLevel = reportingLevel;
    }

    public static ReportingLevel getReportingLevel() {
        return reportingLevel;
    }

    public static boolean isReportable(ReportingType<?> reportingType) {
        return reportingType.getLevel().level >= reportingLevel.level;
    }
}
