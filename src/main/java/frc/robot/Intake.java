package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake {
    private CANSparkMax motor = new CANSparkMax(9, MotorType.kBrushless);
    public void move (double speed) {
        speed = speed * 0.75;
        motor.set(speed);
    }
}
