package frc.robot.comm.reporting;

/**
 * Reporting level for ReportingType. "Finer" levels should have smaller numbers
 */
public enum ReportingLevel {
    DEBUG(0), TEST(1), COMPETITON(2);

    public int level;

    private ReportingLevel(int level) {
        this.level = level;
    }
}
