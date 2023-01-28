package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
public class PickerUpper {

    private final CANSparkMax m_arm = new CANSparkMax(5, MotorType.kBrushless);
    private final DigitalInput m_armLimitDown = new DigitalInput(0);
    private final DigitalInput m_armLimitUp = new DigitalInput(1);

    private final CANSparkMax m_tower = new CANSparkMax(6, MotorType.kBrushless);
    private final DigitalInput m_towerLimitRight = new DigitalInput(2);
    private final DigitalInput m_towerLimitLeft = new DigitalInput(3);

    public void moveArm(double speed) {
        speed = speed * 0.1;
        m_arm.set(speed);
        System.out.println(speed);
        if(speed > 0) {
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
        }

    }

    public void moveTower(double speed) {
        speed = speed * 0.2;
        m_tower.set(speed);
        if(speed > 0) {
            /*if(m_towerLimitRight.get()) {
                m_tower.set(0);
            } else {
                m_tower.set(speed);
            
            }
        } else{
            if(m_towerLimitLeft.get()) {
                m_tower.set(0);
            } else {
                m_tower.set(speed);
            }*/
 

        }
        m_tower.set(speed);
    }
    
    
    public void extend(double speed) {}
    public void retract(double speed) {}

    public void open(double angle) {}
    public void closeCone() {}
    public void closeCube() {}
}

