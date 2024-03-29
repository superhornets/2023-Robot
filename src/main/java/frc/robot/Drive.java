package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Drive extends SubsystemBase {
  private final CANSparkMax leftFrontDrive = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax leftRearDrive = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax rightFrontDrive = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax rightRearDrive = new CANSparkMax(4, MotorType.kBrushless);
  private SparkMaxPIDController m_pidController;
  private SparkMaxPIDController m_pidControllerR;
  private RelativeEncoder m_encoder;
  private RelativeEncoder m_encoderR;
  public double kP,
      kI,
      kD,
      kIz,
      kFF,
      kMaxOutput,
      kMinOutput,
      maxRPM,
      maxVel,
      minVel,
      maxAcc,
      allowedErr;
  private /*final*/ double ROTATIONS_PER_INCH = .5694;
  public boolean isMoving = false;
  private double currentPos = 0;
  private double currentPosR = 0;
  private int driveSpeed = 4800;
  private boolean rotateToAngle = false;
  private double currentRotationRate = 0;
  private final double METERS_TO_INCHES = 39.37;
  private double autoTime = 0;
  AHRS ahrs;
  double rotateToAngleRate;
  PIDController turnController;
  // private double flatAngle = 0;
  final double kPT = 0.02;
  final double kIT = 0.00;
  final double kDT = 0.00;
  final double kFT = 0.00;
  int levelStage = 0;
  int driveStage = 0;
  GenericEntry yay;

  // vision
  PhotonCamera camera = new PhotonCamera("OV5647");

  public void driveInit() {
    kP = 5e-5; // was 5e-5
    kI = 8e-7; // was 8e-7
    kD = 0;
    kIz = 0;
    kFF = 0;
    yay = Shuffleboard.getTab("alsoyay")
        .add("dfsafkj", 0)
        .withWidget(BuiltInWidgets.kGraph)
        .getEntry();
    maxRPM = 5700;
    maxVel = 3000; // rpm
    maxAcc = 1000;
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

    // turn to angle init

  }

  public void setPos() {
    m_encoder.setPosition(0);
    m_encoderR.setPosition(0);
    currentPos = m_encoder.getPosition();
    currentPosR = m_encoderR.getPosition();
  }

  public void teleopInitDrive() {
    setPos();
    holdSpeed(0);
    isMoving = false;
    System.out.println("teleop init");
  }

  public double returnAngle() {
    return ahrs.getAngle();
  }

  public void NavXInit() {
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
      // DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }
    turnController = new PIDController(kPT, kIT, kDT);
    turnController.enableContinuousInput(-180.0f, 180.0f);
    ahrs.calibrate();
    // flatAngle = ahrs.getRoll();
  }

  public void resetNavX() {
    ahrs.reset();
    // flatAngle = ahrs.getRoll();
  }

  public double wrapAngle(double angle) {
    if (angle >= -180 && angle <= 180) {
      return angle;
    } else if (angle < -180) {
      angle = (360 + angle);
      return wrapAngle(angle);
    } else {
      angle = (-360 + angle);
      return wrapAngle(angle);
    }
  }

  public boolean isDriving(double distance) {
    if ((m_encoder.getPosition() <= (distance + .5) * ROTATIONS_PER_INCH + currentPos
        && m_encoder.getPosition() >= (distance - .5) * ROTATIONS_PER_INCH + currentPos)
        && isMoving
        && (m_encoderR.getPosition() <= (distance + .5) * ROTATIONS_PER_INCH + currentPosR
            && m_encoderR.getPosition() >= (distance - .5) * ROTATIONS_PER_INCH + currentPosR)) {
      // System.out.println("is driving: true");
      return true;
    } else {
      // System.out.println("is driving: false " + distance*ROTATIONS_PER_INCH+currentPos);
      return false;
    }
  }

  public boolean isTurning(double angle) {
    if (wrapAngle(ahrs.getAngle()) <= angle + 3 && wrapAngle(ahrs.getAngle()) >= angle - 3) {
      return true;
    } else {
      return false;
    }
  }

  public boolean driveTo(double distance) {
    if (!isMoving) {
      isMoving = true;
      // setPos();

    }
    m_pidController.setReference(
        (currentPos + distance * ROTATIONS_PER_INCH),
        com.revrobotics.CANSparkMax.ControlType.kSmartMotion);
    m_pidControllerR.setReference(
        (currentPosR + distance * ROTATIONS_PER_INCH),
        com.revrobotics.CANSparkMax.ControlType.kSmartMotion);
    if (isDriving(distance)) {
      isMoving = false;
      return true;
      // System.out.println("is driving true");
    } else {
      return false;
    }
  }

  public void holdSpeed(double speed) {
    setPos();
    m_pidController.setReference((speed), com.revrobotics.CANSparkMax.ControlType.kSmartVelocity);
    m_pidControllerR.setReference((speed), com.revrobotics.CANSparkMax.ControlType.kSmartVelocity);
  }

  public void holdPosition() {
    // setPos();
    m_pidController.setReference(
        (currentPos), com.revrobotics.CANSparkMax.ControlType.kSmartMotion);
    m_pidControllerR.setReference(
        (currentPosR), com.revrobotics.CANSparkMax.ControlType.kSmartMotion);
  }

  public boolean turnTo(double angle) {
    angle = wrapAngle(angle);
    if (!rotateToAngle) {
      turnController.setSetpoint(angle);
      rotateToAngle = true;
    }
    currentRotationRate = MathUtil.clamp(turnController.calculate(wrapAngle(ahrs.getAngle())), -.5, .5);
    if (!isTurning(angle) /*&& !(currentRotationRate < .01 && currentRotationRate > -.01)*/) {
      arcade(0, currentRotationRate);
      return false;
    } else if (!(currentRotationRate < .01 && currentRotationRate > -.01)) {
      arcade(0, currentRotationRate);
      return false;
    } else {
      rotateToAngle = false;
      setPos();
      holdPosition();
      return true;
    }
  }

  public void stopRotation() {
    rotateToAngle = false;
  }

  public void arcade(double forwardSpeed, double turnSpeed) {
    /*if(Math.abs(turnSpeed) > .1){
        m_pidController.setSmartMotionMaxAccel(100, 0);
        m_pidControllerR.setSmartMotionMaxAccel(100, 0);
    
    }
    else{
        m_pidController.setSmartMotionMaxAccel(1000, 0);
        m_pidControllerR.setSmartMotionMaxAccel(1000, 0);
    
    }*/
    double turning = 0;
    // if(Math.abs(turnSpeed) < .1){
    // turning = 0;
    //// }
    // else{

    if (turnSpeed < 0) {
      turning = (turnSpeed + .05) * .25;

    } else {
      turning = (turnSpeed - .05) * .25;
    }
    // }
    var speeds = DifferentialDrive.arcadeDriveIK(forwardSpeed, turning, false);
    m_pidController.setReference(
        speeds.left * driveSpeed, com.revrobotics.CANSparkMax.ControlType.kSmartVelocity);
    m_pidControllerR.setReference(
        speeds.right * driveSpeed, com.revrobotics.CANSparkMax.ControlType.kSmartVelocity);
    // SmartDashboard.putNumber("left", speeds.left);
    // SmartDashboard.putNumber("right", speeds.right);
    setPos();
    // System.out.println("arcade");
  }

  public void arcadeTeleop1(double forwardSpeed, double turnSpeed) {
    var speeds = DifferentialDrive.arcadeDriveIK(forwardSpeed, turnSpeed, false);
    leftFrontDrive.set(speeds.left);
    rightFrontDrive.set(speeds.right);
    setPos();
  }

  public void levelInit() {
    levelStage = 0;
  }

  public boolean level() {
    SmartDashboard.putNumber("Level stage", levelStage);
    SmartDashboard.putNumber("ahrs.getRoll", ahrs.getRoll());

    double time = 0;
    if (levelStage == 0) {
      arcadeTeleop1(-.3, 0);
      if (ahrs.getRoll() > 5) {
        levelStage = 1;
        System.out.println("stage 0");
      }
    } else if (levelStage == 1) {
      arcadeTeleop1(-.2, 0);
      if (ahrs.getRoll() > 0) {
        levelStage = 2;
        time = Timer.getFPGATimestamp();
        System.out.println("stage 1");
      }
    } else if (levelStage == 2) {
      setPos();
      holdPosition();
      if (Math.abs(Timer.getFPGATimestamp() - time) > .5) {
        if (ahrs.getRoll() >= -5 && ahrs.getRoll() <= 5) {
          levelStage = 4;
          setPos();
        } else if (ahrs.getRoll() > 0) {
          levelStage = 3;
        } else if (ahrs.getRoll() < -6) {
          levelStage = 1;
          System.out.println("Going to stage 3");
        }
        System.out.println("stage 2");
      }
    } else if (levelStage == 3) {
      arcadeTeleop1(.3, 0);
      if (ahrs.getRoll() > -6) {
        levelStage = 2;
        time = Timer.getFPGATimestamp();
        System.out.println("stage 3");
      }
    } else if (levelStage == 4) {
      holdPosition();
      System.out.println("stage 4");
    }

    if (levelStage == 4) {
      return true;
    } else {
      return false;
    }
  }

  public void driveOverInit() {
    driveStage = 0;
    autoTime = Timer.getFPGATimestamp();
  }

  public boolean driveOverChargingStation() {
    double distance = 0;
    var result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();
    if (hasTargets) {
      PhotonTrackedTarget target = result.getBestTarget();
      if (target.getFiducialId() == 4) {
        distance = target.getBestCameraToTarget().getX();
      }
    }
    if (driveStage == 0) {
      arcadeTeleop1(.5, 0);
      if (distance >= 2.8 || Math.abs(Timer.getFPGATimestamp() - autoTime) > 1) {
        driveStage = 1;
      }
      return false;
    } else if (driveStage == 1) {
      arcadeTeleop1(.2, 0);

      if (distance >= 5 || Math.abs(Timer.getFPGATimestamp() - autoTime) > 5) {
        driveStage = 2;
        arcadeTeleop1(0, 0);
      }
      return false;
    } else if (driveStage == 2) {
      holdPosition();
      return true;
    } else {
      return false;
    }
  }

  public void SmartDashboardPrintout(/*double distance*/ ) {
    // SmartDashboard.putNumber("angle", ahrs.getAngle());
    // SmartDashboard.putBoolean("isMoving", isMoving);
    // SmartDashboard.putNumber("currentPos", currentPos);
    // SmartDashboard.putNumber("currentPosR", currentPosR);
    // SmartDashboard.putBoolean("isDrving", isDriving(distance));
    // SmartDashboard.putNumber("roll", ahrs.getRoll()-flatAngle);
    // SmartDashboard.putNumber("yaw", ahrs.getYaw());

    // SmartDashboard.putNumber("pitch", ahrs.getPitch());
    var result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();
    if (hasTargets) {
      PhotonTrackedTarget target = result.getBestTarget();
      SmartDashboard.putNumber("aprilTag", target.getFiducialId());
      SmartDashboard.putNumber(
          "distance X", target.getBestCameraToTarget().getX() * METERS_TO_INCHES);
      SmartDashboard.putNumber(
          "distance Y", target.getBestCameraToTarget().getY() * METERS_TO_INCHES);
      SmartDashboard.putNumber("distance Z", target.getBestCameraToTarget().getZ());
      SmartDashboard.putNumber("navX angle", ahrs.getAngle());
    }
    SmartDashboard.putNumber("left drive motor", m_encoder.getVelocity());
    SmartDashboard.putNumber("right drive motor", m_encoderR.getVelocity());
    SmartDashboard.putNumber("drive angle", ahrs.getAngle());
    SmartDashboard.putNumber("navX yaw", ahrs.getRoll());
  }

  public boolean checkForTarget(int targetID) {
    var result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();
    if (hasTargets) {
      PhotonTrackedTarget target = result.getBestTarget();
      if (targetID == 0) {
        return true;
      } else {
        if (targetID == target.getFiducialId()) {
          return true;
        } else {
          return false;
        }
      }
    } else {
      return false;
    }
  }

  public int targetID() {
    var result = camera.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    int ID = target.getFiducialId();
    return ID;
  }

  public Transform3d targetValues() {
    var result = camera.getLatestResult();
    // boolean hasTargets = result.hasTargets();

    PhotonTrackedTarget target = result.getBestTarget();
    Transform3d targetVal = target.getBestCameraToTarget();
    return targetVal;
  }
}
