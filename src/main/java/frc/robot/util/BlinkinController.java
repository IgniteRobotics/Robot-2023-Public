package frc.robot.util;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class BlinkinController {
    private Spark pwm;

    public BlinkinController(int pwmPort) {
        pwm = new Spark(pwmPort);
    }

    public void setPWM(double pwmValue) {
        this.pwm.set(pwmValue);
    }
}
