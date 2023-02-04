package frc.robot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;




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
        private int driveSpeed = 4800;
    private boolean isAutoDriving = false;
    private boolean rotateToAngle = false;
    private double currentRotationRate = 0;
    AHRS ahrs;
    double rotateToAngleRate;
    PIDController turnController;
    final double kPT = 0.02;
    final double kIT = 0.00;
    final double kDT = 0.00;
    final double kFT = 0.00;
    
    public void driveInit(){
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
        m_pidController = leftFrontDrive.getPIDController();
        m_pidControllerR = rightFrontDrive.getPIDController();
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
        System.out.println("drive init");
        
        // Encoder object created to display position values
        m_encoder = leftFrontDrive.getEncoder();
        m_encoderR = rightFrontDrive.getEncoder();


        //turn to angle init

    }
    public void setPos(){
        currentPos=m_encoder.getPosition();
        currentPosR = m_encoderR.getPosition();
    }
    public void teleopInitDrive(){
        setPos();
        isMoving = false;
        System.out.println("teleop init");
    }
    public void NavXInit (){
        try {
            /***********************************************************************
             * navX-MXP: - Communication via RoboRIO MXP (SPI, I2C) and USB. - See
             * http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
             * 
             * navX-Micro: - Communication via I2C (RoboRIO MXP or Onboard) and USB. - See
             * http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
             * 
             * VMX-pi: - Communication via USB. - See
             * https://vmx-pi.kauailabs.com/installation/roborio-installation/
             * 
             * Multiple navX-model devices on a single robot are supported.
             ************************************************************************/
            ahrs = new AHRS(SPI.Port.kMXP);
          } catch (RuntimeException ex) {
            //DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
          }
          turnController = new PIDController(kPT, kIT, kDT);
          turnController.enableContinuousInput(-180.0f, 180.0f);
      
    }
    public void resetNavX(){
        ahrs.reset();
    }
    public double wrapAngle(double angle){
        if(angle >= -180 && angle <= 180){
            return angle;
        }
        else if(angle < -180){
            angle = (360 + angle);
            return wrapAngle(angle);
        }
        else{
            angle = (-360 + angle);
            return wrapAngle(angle);
        }
    }
    public boolean isDriving(double distance){
        if((m_encoder.getPosition() <= (distance+.5)*ROTATIONS_PER_INCH+currentPos && m_encoder.getPosition() >= (distance-.5)*ROTATIONS_PER_INCH+currentPos) && isMoving && (m_encoderR.getPosition() <= (distance+.5)*ROTATIONS_PER_INCH+currentPosR && m_encoderR.getPosition() >= (distance-.5)*ROTATIONS_PER_INCH+currentPosR)){
            //System.out.println("is driving: true");
            return true;
        }
        else{
            //System.out.println("is driving: false " + distance*ROTATIONS_PER_INCH+currentPos);
            return false;
        }
    }
    public boolean isTurning(double angle) {
        if(wrapAngle(ahrs.getAngle()) <= angle + 3 && wrapAngle(ahrs.getAngle()) >= angle - 3){
            return true;
        }
        else{return false;}
    }
    public boolean driveTo(double distance) {
        if(!isMoving){
            isMoving=true;
            //setPos();
            
        }
        m_pidController.setReference((currentPos+distance*ROTATIONS_PER_INCH),com.revrobotics.CANSparkMax.ControlType.kSmartMotion);
        m_pidControllerR.setReference((currentPosR+distance*ROTATIONS_PER_INCH),com.revrobotics.CANSparkMax.ControlType.kSmartMotion);
        if(isDriving(distance)){
            isMoving = false;
            return true;
            //System.out.println("is driving true");
        }
        else{return false;}
        }
    public void holdSpeed(double speed){
        setPos();
        m_pidController.setReference((speed),com.revrobotics.CANSparkMax.ControlType.kSmartVelocity);
        m_pidControllerR.setReference((speed),com.revrobotics.CANSparkMax.ControlType.kSmartVelocity);
    }
    public void holdPosition(){
        //setPos();
        m_pidController.setReference((currentPos),com.revrobotics.CANSparkMax.ControlType.kSmartMotion);
        m_pidControllerR.setReference((currentPosR),com.revrobotics.CANSparkMax.ControlType.kSmartMotion);
    }
    public boolean turnTo(double angle) {
        angle = wrapAngle(angle);
        if(!rotateToAngle){
            turnController.setSetpoint(angle);
            rotateToAngle = true;
        }
        currentRotationRate = MathUtil.clamp(turnController.calculate(wrapAngle(ahrs.getAngle())), -.5, .5);
        if(!isTurning(angle) /*&& !(currentRotationRate < .01 && currentRotationRate > -.01)*/){
            arcade(0, currentRotationRate);
            return false;
        }
        else if(!(currentRotationRate < .01 && currentRotationRate > -.01)){
            arcade(0, currentRotationRate);
            return false;
        }
        else{
            rotateToAngle = false;
            setPos();
            holdPosition();
            return true;
            }


    }
    public void stopRotation(){
        rotateToAngle = false;
    }
    public void arcade(double forwardSpeed, double turnSpeed) {
        var speeds = DifferentialDrive.arcadeDriveIK(forwardSpeed, turnSpeed, true);
        m_pidController.setReference(speeds.left*driveSpeed, com.revrobotics.CANSparkMax.ControlType.kSmartVelocity);
        m_pidControllerR.setReference(speeds.right*driveSpeed, com.revrobotics.CANSparkMax.ControlType.kSmartVelocity);
        //SmartDashboard.putNumber("left", speeds.left);
        //SmartDashboard.putNumber("right", speeds.right);
        setPos();
        //System.out.println("arcade");
    }

    public void level() {}

    public void SmartDashboardPrintout(double distance){
        SmartDashboard.putNumber("angle", ahrs.getAngle());
        SmartDashboard.putBoolean("isMoving", isMoving);
        SmartDashboard.putNumber("currentPos", currentPos);
        SmartDashboard.putNumber("currentPosR", currentPosR);
        SmartDashboard.putBoolean("isDrving", isDriving(distance));
    }
}
