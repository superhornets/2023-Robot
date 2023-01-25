package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
public class PickerUpper {

    private final CANSparkMax m_arm = new CANSparkMax(5, MotorType.kBrushless);
    private final DigitalInput m_armLimitDown = new DigitalInput(0);
    private final DigitalInput m_armLimitUp = new DigitalInput(1);

    public void moveArm(double speed) {
        speed = speed * 0.1;
        m_arm.set(speed);
        System.out.println(speed);
        /* if(speed > 0) {
            if(m_armLimitDown.get()) {
                m_arm.set(0);
            } else {
                m_arm.set(speed);
            }
        } else{
            if(m_armLimitUp.get()) {
                m_arm.set(0);
            } else {
                m_arm.set(speed);
            }
        }*/

    }

    public void left(double angle) {}
    public void right(double angle) {}
    
    public void extend(double speed) {}
    public void retract(double speed) {}

    public void open(double angle) {}
    public void closeCone() {}
    public void closeCube() {}
}

