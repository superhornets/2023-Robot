package frc.robot;

import java.net.ConnectException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class PickerUpper {

    private final CANSparkMax m_arm = new CANSparkMax(5, MotorType.kBrushless);
    private final DigitalInput m_armLimitDown = new DigitalInput(0);
    private final DigitalInput m_armLimitUp = new DigitalInput(1);

    private final CANSparkMax m_tower = new CANSparkMax(6, MotorType.kBrushless);
    private final DigitalInput m_towerLimitRight = new DigitalInput(2);
    private final DigitalInput m_towerLimitLeft = new DigitalInput(3);

    private final CANSparkMax m_grabber = new CANSparkMax(7, MotorType.kBrushless);
    private final DigitalInput m_grabberLimitOpen = new DigitalInput(4);
    private final DigitalInput m_grabberLimitClosed = new DigitalInput(5);

    public void moveArm(double speed) {
        speed = speed * 0.1;
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

    public void open() {
        if(m_grabberLimitOpen.get()) {
            m_grabber.set(0);
        } else {
            m_grabber.set(0.5);
        }

    }
    public void closeCone() {
        if(m_grabberLimitClosed.get()) {
            m_grabber.set(0);
        } else{
            if(m_grabber.getOutputCurrent() > 40) {
                m_grabber.set(0);
            } else{
                m_grabber.set(0.5);
            }
        }
    }
    public void closeCube() {}

    public void periodic() {
        double grabberCurrent = m_grabber.getOutputCurrent();
        SmartDashboard.putNumber("Grabber Current", grabberCurrent);
    }
}