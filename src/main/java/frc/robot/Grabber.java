package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;


public class Grabber extends SubsystemBase{

    private final CANSparkMax m_grabber = new CANSparkMax(7, MotorType.kBrushless);
    private final DigitalInput m_grabberLimitOpen = new DigitalInput(4);
    private final DigitalInput m_grabberLimitClosed = new DigitalInput(5);
 
    
    private SparkMaxPIDController m_pidController;
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, maxAcc;
    private RelativeEncoder m_encoder;
    private RelativeEncoder m_grabberEncoder;
    private double grabberSpeed = .7;

    GenericEntry extendP;
    PhotonCamera camera = new PhotonCamera("HD_USB_Camera");
    


    public Grabber(){

        m_grabber.setInverted(false);

        m_grabberEncoder = m_grabber.getEncoder();
        
        kP =  1e-5; 
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

        extendP = Shuffleboard.getTab("alsoyay").add("extendp", 0).getEntry();

    }
    public void grabberSmartDashboard() {

        //grabberSpeed = SmartDashboard.getNumber("grabber speed", grabberSpeed);
        //SmartDashboard.putNumber("grabber speed", grabberSpeed);

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
        if (false) {
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

    public boolean checkForTarget() {
        var result = camera.getLatestResult();
        boolean hasTargets = result.hasTargets();
        if(hasTargets){
            return true;
        }
        else{
            return false;
            }
    }
    public double targetYaw(){
        var result = camera.getLatestResult();
        boolean hasTargets = result.hasTargets();
        PhotonTrackedTarget target = result.getBestTarget();
        return target.getYaw();
    }
    public double targetPitch(){
        var result = camera.getLatestResult();
        boolean hasTargets = result.hasTargets();
        PhotonTrackedTarget target = result.getBestTarget();
        return target.getPitch();
    }
    public double targetSkew(){
        var result = camera.getLatestResult();
        boolean hasTargets = result.hasTargets();
        PhotonTrackedTarget target = result.getBestTarget();
        return target.getSkew();
    }
}