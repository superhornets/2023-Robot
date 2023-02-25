package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Grabber {

    private final CANSparkMax m_grabber = new CANSparkMax(7, MotorType.kBrushless);
    private final CANSparkMax extender = new CANSparkMax(8, MotorType.kBrushless);
    private final DigitalInput m_grabberLimitOpen = new DigitalInput(4);
    private final DigitalInput m_grabberLimitClosed = new DigitalInput(5);
    private final RelativeEncoder m_towerEncoder = extender.getEncoder();
    
    private SparkMaxPIDController m_pidController;
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, maxAcc;
    private double currentPos = 0;
    private RelativeEncoder m_encoder;
    private int driveSpeed = 5000;
    private final double GEAR_RATIO = 38.1;

    public Grabber(){
        m_pidController = extender.getPIDController();
        m_encoder = extender.getEncoder();
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

    public void extend(double speed) {
        m_pidController.setReference(speed*driveSpeed, ControlType.kSmartVelocity);
    }

    public boolean extendToPos(double Pos){
        double target = (Pos*GEAR_RATIO) + currentPos;
        m_pidController.setReference(target, ControlType.kSmartMotion);
        if(m_encoder.getPosition() > Pos*GEAR_RATIO+currentPos-20 && m_encoder.getPosition() < Pos*GEAR_RATIO+currentPos+20 ){
            return false;
        }
        else{
            return true;
        }

    }
    public boolean retractToPos(double Pos){
        return false;
    }
 
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
    public double returnExtension(){
        return m_encoder.getPosition()*GEAR_RATIO;
    }
}