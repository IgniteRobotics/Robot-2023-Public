package frc.robot.comm;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Encoder;

public class SendableTalonEncoder implements Sendable, AutoCloseable {
    private final TalonFX talonFX;

    public SendableTalonEncoder(TalonFX talonFX) {
        this.talonFX = talonFX;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Encoder");
        builder.addDoubleProperty("Speed", talonFX::getSelectedSensorVelocity, null);
        builder.addDoubleProperty("Distance", talonFX::getSelectedSensorPosition, null);
    }

    @Override
    public void close() throws Exception {
        SendableRegistry.remove(this);
    }
}
