package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Drive {
    private final CANSparkMax leftFrontDrive = new CANSparkMax(1, MotorType.kBrushless);
    private final CANSparkMax leftRearDrive = new CANSparkMax(2, MotorType.kBrushless);
    private final CANSparkMax rightFrontDrive = new CANSparkMax(3, MotorType.kBrushless);
    private final CANSparkMax rightRearDrive = new CANSparkMax(4, MotorType.kBrushless);
    private SparkMaxPIDController m_pidController;
    private SparkMaxPIDController m_pidControllerR;
    private RelativeEncoder m_encoder;
    private RelativeEncoder m_encoderR;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    private /*final*/ double ROTATIONS_PER_INCH = .5694;
    private double distance = 24;
    public boolean isMoving = false;
    private double currentPos = 0;
    private double currentPosR = 0;
    private PickerUpper pickerUpper = new PickerUpper();
    private int driveSpeed = 4800;
    private boolean isAutoDriving = false;
    
    public void driveInit(){
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);
        m_pidController.setSmartMotionMaxAccel(maxAcc, 0);
        m_pidController.setSmartMotionMaxVelocity(maxVel, 0);
        m_pidControllerR.setSmartMotionMaxVelocity(maxVel, 0);
        m_pidControllerR.setSmartMotionMaxAccel(maxAcc, 0);
        m_pidControllerR.setP(kP);
        m_pidControllerR.setI(kI);
        m_pidControllerR.setD(kD);
        m_pidControllerR.setIZone(kIz);
        m_pidControllerR.setFF(kFF);
        m_pidControllerR.setOutputRange(kMinOutput, kMaxOutput);
        leftFrontDrive.setInverted(true);
        m_pidController = leftFrontDrive.getPIDController();
        m_pidControllerR = rightFrontDrive.getPIDController();
        // Encoder object created to display position values
        m_encoder = leftFrontDrive.getEncoder();
        m_encoderR = rightFrontDrive.getEncoder();
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
        leftRearDrive.follow(leftFrontDrive);
        rightRearDrive.follow(rightFrontDrive);
    }
    public void setPos(){
        currentPos=m_encoder.getPosition();
        currentPosR = m_encoderR.getPosition();
    }
    public void teleopInitDrive(){
        setPos();
        isMoving = false;
    }
    public boolean isDriving(){
        if((m_encoder.getPosition() <= (distance+.5)*ROTATIONS_PER_INCH+currentPos && m_encoder.getPosition() >= (distance-.5)*ROTATIONS_PER_INCH+currentPos) && isMoving && (m_encoderR.getPosition() <= (distance+.5)*ROTATIONS_PER_INCH+currentPosR && m_encoderR.getPosition() >= (distance-.5)*ROTATIONS_PER_INCH+currentPosR)){
            return true;
        }
        else{
            return false;
        }
    }
    public void driveTo(double distance) {
        if(!isMoving){
            isMoving=true;
            setPos();
        }
        if(isDriving()){
            isMoving = false;
            setPos();
        }
        m_pidController.setReference((currentPos+distance*ROTATIONS_PER_INCH),com.revrobotics.CANSparkMax.ControlType.kSmartMotion);
        m_pidControllerR.setReference((currentPosR+distance*ROTATIONS_PER_INCH),com.revrobotics.CANSparkMax.ControlType.kSmartMotion);
    }
    public void holdSpeed(double speed){
        m_pidController.setReference((speed),com.revrobotics.CANSparkMax.ControlType.kSmartVelocity);
        m_pidControllerR.setReference((speed),com.revrobotics.CANSparkMax.ControlType.kSmartVelocity);
    }
    public void turnTo(double angle) {}

    public void arcade(double forwardSpeed, double turnSpeed) {
        var speeds = DifferentialDrive.arcadeDriveIK(forwardSpeed, turnSpeed, true);
      m_pidController.setReference(speeds.left*driveSpeed, com.revrobotics.CANSparkMax.ControlType.kSmartVelocity);
      m_pidControllerR.setReference(speeds.right*driveSpeed, com.revrobotics.CANSparkMax.ControlType.kSmartVelocity);
      //SmartDashboard.putNumber("left", speeds.left);
      //SmartDashboard.putNumber("right", speeds.right);
      setPos();
    }

    public void level() {}
}
