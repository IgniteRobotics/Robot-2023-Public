package frc.robot.comm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

public class SendableAbsoluteEncoder implements Sendable, AutoCloseable {
    private final AbsoluteEncoder encoder;

    public SendableAbsoluteEncoder(AbsoluteEncoder encoder) {
        this.encoder = encoder;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Encoder");
        builder.addDoubleProperty("Speed", encoder::getVelocity, null);
        builder.addDoubleProperty("Distance", encoder::getPosition, null);
    }

    @Override
    public void close() throws Exception {
        SendableRegistry.remove(this);
    }
}
