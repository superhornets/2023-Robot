package frc.robot;

import java.net.ConnectException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class PickerUpper {

    private final CANSparkMax m_arm = new CANSparkMax(5, MotorType.kBrushless);
    private final DigitalInput m_armLimitDown = new DigitalInput(0);
    private final DigitalInput m_armLimitUp = new DigitalInput(1);

    private final CANSparkMax m_tower = new CANSparkMax(6, MotorType.kBrushless);
    private final DigitalInput m_towerLimitRight = new DigitalInput(2);
    private final DigitalInput m_towerLimitLeft = new DigitalInput(3);
    private final RelativeEncoder m_towerEncoder = m_tower.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);



    private final CANSparkMax m_grabber = new CANSparkMax(7, MotorType.kBrushless);
    private final DigitalInput m_grabberLimitOpen = new DigitalInput(4);
    private final DigitalInput m_grabberLimitClosed = new DigitalInput(5);



    private SparkMaxPIDController m_pidController;
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    private double currentPos = 0;
    private RelativeEncoder m_encoder;
    private int driveSpeed = 4800;




    public void armInit(){
        m_pidController=m_arm.getPIDController();
        m_encoder = m_arm.getEncoder();
        kP = 5e-5; 
        kI = 8e-7;
        kD = 0; 
        kIz = 0; 
        kFF = 0;
        maxRPM = 5700;
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
    public boolean setTurret(Double angle) {
        return false;
    }
    public boolean setArm(Double angle){
        return false;
    }
    
    public boolean extend(double speed) { return false; }
    public void retract(double speed) {}


    public void SmartDashboardPrintout(){
        SmartDashboard.putNumber("tower posison", m_towerEncoder.getPosition());
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

    
}