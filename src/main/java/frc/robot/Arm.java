package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

public class Arm {

    private SparkMaxPIDController m_pidController;
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, maxAcc;
    private double currentPos = 0;
    private RelativeEncoder m_encoder;
    private int driveSpeed = 4800;
    private final double STARTING_ANGLE = 0;
    private final double flatAngle = 90-STARTING_ANGLE;

    private final CANSparkMax m_arm = new CANSparkMax(5, MotorType.kBrushless);
    private final DigitalInput m_armLimitDown = new DigitalInput(0);
    private final DigitalInput m_armLimitUp = new DigitalInput(1);
    private Tower tower;
    private Grabber grabber;

    private double currentAngle = STARTING_ANGLE;
    private final double ARM_LENGTH = 0;
    private final double EXTENDER_GEAR_RATIO = 150;
    private final double TOWER_HEIGHT = 0;

    public Arm(){
        m_pidController=m_arm.getPIDController();
        m_encoder = m_arm.getEncoder();
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
    public void updatePosition(){
    }
    public void setPickerUpper(Tower tower, Grabber grabber){
        this.tower = tower;
        this.grabber = grabber;
    }
    public void moveArm(double speed) {
        speed = speed * 0.2;
        if(speed == 0) {
            /*if(m_armLimitDown.get()) {
                m_arm.set(0);
            } else {*/
                m_pidController.setReference(currentPos, ControlType.kSmartMotion);            //}
        } else if(speed >0){
            /*if(m_armLimitUp.get()) {
                m_arm.set(0);
            } else {*/
                m_pidController.setReference(speed*driveSpeed, ControlType.kSmartVelocity);
                currentPos = m_encoder.getPosition();
            //}
        }
        else if(speed <0){
            /*if(m_armLimitUp.get()) {
                m_arm.set(0);
            } else {*/
                m_pidController.setReference(speed*driveSpeed, ControlType.kSmartVelocity);
                currentPos = m_encoder.getPosition();
    
            //}
        }
    
    }
    public boolean isAtHeightLimit(){
        currentAngle = m_encoder.getPosition()*2;
        if(currentAngle < flatAngle){
            return false;
        }
        else if(armYDistance() < 78 && armYDistance() > 76){
            return true;
        }
        else{
            return false;
        }
        

    }

    public double armXDistance(){
        double armX = (grabber.returnExtension()+ARM_LENGTH) * Math.sin(currentAngle+STARTING_ANGLE);
        return armX;
    }

    public double armYDistance(){
        double armY = ((grabber.returnExtension()+ARM_LENGTH) * Math.cos(currentAngle+STARTING_ANGLE-90))+TOWER_HEIGHT;
        return armY;
    }
}

