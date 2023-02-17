package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

public class Tower {
    
    private final CANSparkMax m_tower = new CANSparkMax(6, MotorType.kBrushless);
    private final DigitalInput m_towerLimitRight = new DigitalInput(2);
    private final DigitalInput m_towerLimitLeft = new DigitalInput(3);
    private final RelativeEncoder m_towerEncoder = m_tower.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    
    private SparkMaxPIDController m_pidController;
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, maxAcc;
    private double currentPos = 0;
    private RelativeEncoder m_encoder;
    private int driveSpeed = 4800;

    public Tower(){
        m_pidController = m_tower.getPIDController();
        m_encoder = m_tower.getEncoder();
        kP = 5e-5; 
        kI = 8e-7;
        kD = 0; 
        kIz = 0; 
        kFF = 0;
        maxVel = 2000; // rpm
        maxAcc = 1500;
        kMaxOutput = 1; 
        kMinOutput = -1;
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);
        m_pidController.setSmartMotionMaxAccel(maxAcc, 0);
        m_pidController.setSmartMotionMaxVelocity(maxVel, 0);
        currentPos = m_encoder.getPosition();
    }

    
    
     public void SmartDashboardPrintout(){
         SmartDashboard.putNumber("tower position", m_towerEncoder.getPosition());
     }

    public void moveTower(double speed) {
        speed = speed * 0.2;
       
        if(speed > 0) {
            m_pidController.setReference(speed*driveSpeed, ControlType.kSmartVelocity);
                currentPos = m_encoder.getPosition();
        }

        else if (speed < 0) {
            m_pidController.setReference(speed*driveSpeed, ControlType.kSmartVelocity);
            currentPos = m_encoder.getPosition();
        }
        else if (speed == 0) {
            m_pidController.setReference(currentPos, ControlType.kSmartMotion);  
        }
    }  
    public boolean setTurret(double angle) {
        return false;
    }
    public boolean setArm(double position) {
        return false;
        
    }
}