package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Grabber {

    private final CANSparkMax m_grabber = new CANSparkMax(7, MotorType.kBrushless);
    private final DigitalInput m_grabberLimitOpen = new DigitalInput(4);
    private final DigitalInput m_grabberLimitClosed = new DigitalInput(5);

    public boolean extend(double speed) {return false;}
    public void retract(double speed) {}

 
    public void open() {
        /*if(m_grabberLimitOpen.get()) {
            m_grabber.set(0);
        } else {*/
            if(m_grabber.getOutputCurrent() > 5) {
                m_grabber.set(0);
            } else{
            m_grabber.set(0.1);}
        //}

    }
    public void closeCone() {
        /*if(m_grabberLimitClosed.get()) {
            m_grabber.set(0);
        } else{*/
            if(m_grabber.getOutputCurrent() > 5) {
                m_grabber.set(0);
            } else{
                m_grabber.set(-0.1);
            //}
        }
    }
    public void closeCube() {}

    public void periodic() {
        double grabberCurrent = m_grabber.getOutputCurrent();
        SmartDashboard.putNumber("Grabber Current", grabberCurrent);
    }
}