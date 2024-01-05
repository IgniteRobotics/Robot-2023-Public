package frc.robot.comm;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

public class SendableRelativeEncoder implements RelativeEncoder, Sendable, AutoCloseable {
    private final RelativeEncoder encoder;

    public SendableRelativeEncoder(RelativeEncoder encoder) {
        this.encoder = encoder;
    }

    public double getPosition() {
        return this.encoder.getPosition();
    }

    public double getVelocity() {
        return this.encoder.getVelocity();
    }

    public REVLibError setPosition(double position) {
        return this.encoder.setPosition(position);
    }

    public REVLibError setPositionConversionFactor(double factor) {
        return this.encoder.setPositionConversionFactor(factor);
    }

    public REVLibError setVelocityConversionFactor(double factor) {
        return this.encoder.setVelocityConversionFactor(factor);
    }

    public double getPositionConversionFactor() {
        return this.encoder.getPositionConversionFactor();
    }

    public double getVelocityConversionFactor() {
        return this.encoder.getVelocityConversionFactor();
    }

    public REVLibError setAverageDepth(int depth) {
        return this.encoder.setAverageDepth(depth);
    }

    public int getAverageDepth() {
        return this.encoder.getAverageDepth();
    }

    public REVLibError setMeasurementPeriod(int period_ms) {
        return this.encoder.setMeasurementPeriod(period_ms);
    }

    public int getMeasurementPeriod() {
        return this.encoder.getMeasurementPeriod();
    }

    public int getCountsPerRevolution() {
        return this.encoder.getCountsPerRevolution();
    }

    public REVLibError setInverted(boolean inverted) {
        return this.encoder.setInverted(inverted);
    }

    public boolean getInverted() {
        return this.encoder.getInverted();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Encoder");
        builder.addDoubleProperty("Speed", this::getVelocity, null);
        builder.addDoubleProperty("Distance", this::getPosition, null);
    }

    @Override
    public void close() throws Exception {
        SendableRegistry.remove(this);
    }
}
