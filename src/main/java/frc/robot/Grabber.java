package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grabber extends SubsystemBase{

    private final CANSparkMax m_grabber = new CANSparkMax(7, MotorType.kBrushless);
    private final CANSparkMax extender = new CANSparkMax(8, MotorType.kBrushless);
    private final DigitalInput m_grabberLimitOpen = new DigitalInput(4);
    private final DigitalInput m_grabberLimitClosed = new DigitalInput(5);
    private final RelativeEncoder m_towerEncoder = extender.getEncoder();
    
    private SparkMaxPIDController m_pidController;
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, maxAcc;
    private double currentPos = 0;
    private RelativeEncoder m_encoder;
    private RelativeEncoder m_grabberEncoder;

    private int driveSpeed = 5000;
    private final double GEAR_RATIO = 38.3;
    private double grabberSpeed = 1;
    private double grabberZero = 0;
    private double grabberMax = 180/4;

    GenericEntry extendP;
    


    public Grabber(){

        m_grabber.setInverted(false);
        m_pidController = extender.getPIDController();
        m_encoder = extender.getEncoder();
        m_grabberEncoder = m_grabber.getEncoder();
        extender.setInverted(true);
        kP =  5e-5; 
        kI = 8e-7;
        kD = 0; 
        kIz = 0; 
        kFF = 0;
        maxVel = 3000; // rpm
        maxAcc = 2000;
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

        extendP = Shuffleboard.getTab("alsoyay").add("extendp", 0).getEntry();

    }
    public void grabberSmartDashboard() {

        //grabberSpeed = SmartDashboard.getNumber("grabber speed", grabberSpeed);
        //SmartDashboard.putNumber("grabber speed", grabberSpeed);

    }

    public void extend(double speed) {
        extender.set(speed);
        System.out.print(speed);
        //m_pidController.setReference(-speed*driveSpeed, ControlType.kSmartVelocity);
    }

    public boolean extendToPos(double Pos){
        double target = (Pos*GEAR_RATIO);
        m_pidController.setReference(target, ControlType.kSmartMotion);
        return Math.abs(Pos-returnExtension()) < 1;

    }

    public boolean retractToPos(double Pos) {
        return false;
    }
    public void resetExtenderEncoder() {
        m_encoder.setPosition(0);   
    }
    public void open() {
        /*
         * if(m_grabberLimitOpen.get()) {
         * m_grabber.set(0);
         * } else {
         */
        if (m_grabber.getOutputCurrent() > 8 /*|| Math.abs(m_encoder.getPosition()) > grabberMax*/) {
            m_grabber.set(0);
        } else {
            m_grabber.set(grabberSpeed);
        }
        // }

    }

    public void close() {
        /*
         * if(m_grabberLimitClosed.get()) {
         * m_grabber.set(0);
         * } else{
         */
        if (m_grabber.getOutputCurrent() > 8) {
            m_grabber.set(0);
        } else {
            m_grabber.set(-grabberSpeed);
            // }
        }
    }
    public void hold(){
        m_grabber.set(0);
    }
    public void closeCube() {
    }

    public void periodicGrabber() {
        double grabberCurrent = m_grabber.getOutputCurrent();
        SmartDashboard.putNumber("Grabber Current", grabberCurrent);
        SmartDashboard.putNumber("grabber encoder", m_grabberEncoder.getPosition());
    }
    public double returnExtension(){

        return m_encoder.getPosition()/GEAR_RATIO;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("extender distance", returnExtension());
    }
}