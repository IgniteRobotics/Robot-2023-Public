package frc.robot.comm.reporting;

public abstract class ReportingType<T> {
    protected T value;
    protected String name;
    private ReportingLevel level;

    public ReportingType(String name, ReportingLevel level) {
        this.name = name;
        this.level = level;
    }

    public abstract void put();

    public final void publish() {
        if(ReportingManager.isReportable(this)) {
            put();
        }
    }

    public final void set(T value) {
        this.value = value;
        publish();
    }

    public final T get() {
        return value;
    }

    public ReportingLevel getLevel() {
        return this.level;
    }
}
