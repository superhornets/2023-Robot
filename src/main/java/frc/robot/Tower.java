package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Tower extends SubsystemBase {

  private final CANSparkMax m_tower = new CANSparkMax(6, MotorType.kBrushless);
  private final DigitalInput m_towerLimitRight = new DigitalInput(2);
  private final DigitalInput m_towerLimitLeft = new DigitalInput(3);
  private final int GEAR_RATIO = 60;

  GenericEntry fjasdkl;

  private SparkMaxPIDController m_pidController;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, maxAcc;
  private double currentPos = 0;
  private RelativeEncoder m_encoder;
  private int driveSpeed = 4800;
  private Arm arm;
  private Grabber grabber;
  private double cubeAngle = 0;
  private double startingAngle = 0;
  private boolean safety = false;

  public Tower() {
    fjasdkl = Shuffleboard.getTab("alsoyay").add("tower", 0).getEntry();
    m_pidController = m_tower.getPIDController();
    m_tower.setInverted(true);
    m_encoder = m_tower.getEncoder();
    kP = 5e-5;
    kI = 8e-7;
    kD = 0;
    kIz = 0;
    kFF = 0;
    maxVel = 1000;
    maxAcc = 500;
    kMaxOutput = 0;
    kMinOutput = 0;
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    m_pidController.setSmartMotionMaxAccel(maxAcc, 0);
    m_pidController.setSmartMotionMaxVelocity(maxVel, 0);
    currentPos = getPosition();
  }

  public void SmartDashboardPrintout() {

    SmartDashboard.putNumber("tower angle", returnAngle());
    SmartDashboard.putNumber("maxVel", maxVel);
  }

  public void safety(boolean override) {
    if (arm.returnAngle() < 32) {
      if (!override) {
        if (!safety) {
          safety = true;
          m_pidController.setSmartMotionMaxVelocity(maxVel, 0);
        }
      }
    } else {
      if (safety) {
        safety = true;
        m_pidController.setSmartMotionMaxVelocity(maxVel, 0);
      }
    }
  }

  public void pitSafety(boolean override) {
    if (!override) {
      if (maxVel != 0) {
        maxVel = 0;
        m_pidController.setOutputRange(0, 0);
      }
    } else {
      if (maxVel == 0) {
        maxVel = 1000;
        m_pidController.setOutputRange(-.2, .2);
      }
    }
  }

  public void setPickerUpper(Arm arm, Grabber grabber) {
    this.arm = arm;
    this.grabber = grabber;
  }

  public void moveTower(double speed, boolean override, boolean isSlowMode) {
    if (Math.abs(speed) < .04) {
      speed = 0;
    }
    speed = speed * 0.3;
    if (isSlowMode) {
      speed = speed / 4;
    }

    if (speed > 0 && (returnAngle() < 90 || override)) {
      m_pidController.setReference(speed * driveSpeed, ControlType.kSmartVelocity);
      currentPos = getPosition();
    } else if (speed < 0 && (returnAngle() > -90 || override)) {
      m_pidController.setReference(speed * driveSpeed, ControlType.kSmartVelocity);
      currentPos = getPosition();
    } else if (speed == 0) {
      m_pidController.setReference(0, ControlType.kSmartVelocity);
      currentPos = getPosition();
    }
  }

  public void holdTowerPos() {
    m_pidController.setReference(m_encoder.getPosition(), ControlType.kSmartMotion);
  }

  public boolean moveTowerTo(double angle) {
    m_pidController.setReference(degreesToRotations(angle), ControlType.kSmartMotion);

    double error = angle - getPosition();
    return Math.abs(error) < 3;
  }

  public boolean setArm(double position) {
    return false;
  }

  public void setZero() {
    m_encoder.setPosition(0);
    currentPos = 0;
    moveTowerTo(currentPos);
  }

  public void rotateToCubeInit() {
    cubeAngle = grabber.targetYaw();
    startingAngle = currentPos;
  }

  public boolean rotateToCube() {
    return moveTowerTo(startingAngle + cubeAngle);
  }

  public double getPosition() {
    return rotationsToDegrees(m_encoder.getPosition());
  }

  private double degreesToRotations(double degrees) {
    return degrees / 360 * GEAR_RATIO;
  }

  private double rotationsToDegrees(double rotations) {
    return rotations / GEAR_RATIO * 360;
  }

  public boolean moveTowerToQuadrant(String quadrant) {
    boolean done = false;
    if (quadrant == "a") {
      done = moveTowerTo(0);
    } else if (quadrant == "b") {
      done = moveTowerTo(90);
    } else if (quadrant == "c") {
      if (getPosition() > 0) {
        done = moveTowerTo(180);
      } else {
        done = moveTowerTo(-180);
      }
    } else if (quadrant == "d") {
      done = moveTowerTo(-90);
    }

    return done;
  }

  public double returnAngle() {
    return getPosition();
  }

  public void setP(double yay) {

    m_pidController.setP(yay);
  }

  @Override
  public void periodic() {}
}
