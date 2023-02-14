package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Tower {
   
    public void SmartDashboardPrintout(){
        SmartDashboard.putNumber("tower position", m_towerEncoder.getPosition());
    }

    private final CANSparkMax m_tower = new CANSparkMax(6, MotorType.kBrushless);
    private final DigitalInput m_towerLimitRight = new DigitalInput(2);
    private final DigitalInput m_towerLimitLeft = new DigitalInput(3);
    private final RelativeEncoder m_towerEncoder = m_tower.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);


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
}
