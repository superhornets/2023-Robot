package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Grabber {

    private final CANSparkMax m_grabber = new CANSparkMax(7, MotorType.kBrushless);
    private final CANSparkMax extender = new CANSparkMax(8, MotorType.kBrushless);
    private final DigitalInput m_grabberLimitOpen = new DigitalInput(4);
    private final DigitalInput m_grabberLimitClosed = new DigitalInput(5);
    private double grabberSpeed = .4;

    public void grabberSmartDashboard() {

        extenderSpeed = SmartDashboard.getNumber("grabber speed", grabberSpeed);
        //SmartDashboard.putNumber("grabber speed", grabberSpeed);

    }

    public void extend(double speed) {
        extender.set(speed);
    }

    public void retract(double speed) {
        extender.set(-speed);
    }

    public boolean extendToPos(double Pos) {
        return false;
    }

    public boolean retractToPos(double Pos) {
        return false;
    }

    public void open() {
        /*
         * if(m_grabberLimitOpen.get()) {
         * m_grabber.set(0);
         * } else {
         */
        if (m_grabber.getOutputCurrent() > 5) {
            m_grabber.set(0);
        } else {
            m_grabber.set(grabberSpeed);
        }
        // }

    }

    public void closeCone() {
        /*
         * if(m_grabberLimitClosed.get()) {
         * m_grabber.set(0);
         * } else{
         */
        if (m_grabber.getOutputCurrent() > 5) {
            m_grabber.set(0);
        } else {
            m_grabber.set(-grabberSpeed);
            // }
        }
    }

    public void closeCube() {
    }

    public void periodic() {
        double grabberCurrent = m_grabber.getOutputCurrent();
        SmartDashboard.putNumber("Grabber Current", grabberCurrent);
    }
}