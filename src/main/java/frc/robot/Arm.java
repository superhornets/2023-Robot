package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  private SparkMaxPIDController m_pidController;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, maxAcc;
  private double currentPos = 0;
  private RelativeEncoder m_encoder;
  private int driveSpeed = 4800;
  private final double STARTING_ANGLE = 15;

  private final double EXTENSION_LIMIT = 48;
  private final double LEFT_FRONT_ANGLE = -27.84;
  private final double RIGHT_FRONT_ANGLE = 27.84;
  private final double LEFT_REAR_ANGLE = -131.78;
  private final double RIGHT_REAR_ANGLE = 131.78;
  private final double FRONT_LIMIT = 22.25 + EXTENSION_LIMIT;
  private final double LEFT_LIMIT = 11.75 + EXTENSION_LIMIT;
  private final double RIGHT_LIMIT = 11.75 + EXTENSION_LIMIT;
  private final double REAR_LIMIT = 10.5 + EXTENSION_LIMIT;

  private final CANSparkMax m_arm = new CANSparkMax(5, MotorType.kBrushless);

  private Tower tower;
  private Extender extender;

  public double currentAngle = STARTING_ANGLE;
  private final double ARM_LENGTH = 46.25;
  private final double TOWER_HEIGHT = 50.5;
  private final double GRABBER_WIDTH = 7.75;

  public Arm() {
    m_pidController = m_arm.getPIDController();
    m_encoder = m_arm.getEncoder();
    m_arm.setInverted(true);
    kP = 5e-5;
    kI = 8e-7;
    kD = 0;
    kIz = 0;
    kFF = 0;
    maxVel = 2000;
    maxAcc = 1500;
    kMaxOutput = .2;
    kMinOutput = -.2;
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    m_pidController.setSmartMotionMaxAccel(maxAcc, 0);
    m_pidController.setSmartMotionMaxVelocity(maxVel, 0);
    m_encoder.setPosition(0);
    updatePosition();
  }

  public void updatePosition() {
    currentPos = m_encoder.getPosition() * 2 + STARTING_ANGLE;
  }

  public void resetZero() {
    m_encoder.setPosition(0);
  }

  public void setPickerUpper(Tower tower, Grabber grabber, Extender extender) {
    this.tower = tower;
    this.extender = extender;
  }

  public void moveArm(double speed, boolean isSlowMode) {

    if (speed < .05 && speed > -.05) {
      speed = 0;
    } else if (isSlowMode) {
      speed = speed / 4;
    }
    speed = speed * 0.2;
    if (speed == 0) {

      m_pidController.setReference(0, ControlType.kSmartVelocity);
      System.out.println("moveArm set 0");
    } else if (speed > 0 && !isAtHeightLimit()) {

      m_pidController.setReference(speed * driveSpeed, ControlType.kSmartVelocity);
      System.out.println("move Arm up " + speed * driveSpeed);

    } else if (speed < 0 && !isAtLowerArmLimit()) {

      m_pidController.setReference(speed * driveSpeed, ControlType.kSmartVelocity);
      System.out.println("moveArm down " + speed * driveSpeed);
    }
  }

  public boolean isAtHeightLimit() {
    if (armYDistance() > 77) {
      return true;
    } else {
      return false;
    }
  }

  public boolean isOverHeightLimit() {
    if (armYDistance() > 77.5) {
      return true;
    } else {
      return false;
    }
  }

  public String checkQuadrant() {
    if (tower.returnAngle() > LEFT_FRONT_ANGLE && tower.returnAngle() < RIGHT_FRONT_ANGLE) {
      return "front";
    } else if (tower.returnAngle() > RIGHT_FRONT_ANGLE && tower.returnAngle() < RIGHT_REAR_ANGLE) {
      return "right";
    } else if (tower.returnAngle() > RIGHT_REAR_ANGLE && tower.returnAngle() < LEFT_REAR_ANGLE) {
      return "rear";
    } else {
      return "left";
    }
  }

  public boolean isAtExtentionLimit() {
    String quadrant = checkQuadrant();
    double limit = 0;
    if (quadrant == "front") {
      limit = FRONT_LIMIT;
    } else if (quadrant == "left") {
      limit = LEFT_LIMIT;
    } else if (quadrant == "right") {
      limit = RIGHT_LIMIT;
    } else {
      limit = REAR_LIMIT;
    }

    if (armXDistance() > limit - 1) {
      return true;
    } else {
      return false;
    }
  }

  public boolean isOverExtentionLimit() {
    String quadrant = checkQuadrant();
    double limit = 0;
    if (quadrant == "front") {
      limit = FRONT_LIMIT;
    } else if (quadrant == "left") {
      limit = LEFT_LIMIT;
    } else if (quadrant == "right") {
      limit = RIGHT_LIMIT;
    } else {
      limit = REAR_LIMIT;
    }

    if (armXDistance() > limit - .5) {
      return true;
    } else {
      return false;
    }
  }

  public double armXZDistance() {
    double armX = (extender.returnExtension() + ARM_LENGTH) * Math.sin(Math.toRadians(currentPos));
    return armX;
  }

  public double armYDistance() {
    double armY =
        Math.copySign(
                (extender.returnExtension() + ARM_LENGTH)
                    * Math.sin(Math.toRadians(currentPos - 90)),
                (currentPos - 90))
            + TOWER_HEIGHT;
    return armY;
  }

  public double armXDistance() {
    double angle = normalizeAngle(tower.returnAngle());
    double distance =
        armXZDistance() * Math.cos(Math.toRadians(angle))
            + GRABBER_WIDTH * Math.sin(Math.toRadians(angle));
    return distance;
  }

  public boolean moveArmTo(double position) {
    System.out.println("moveArmTo " + (position - STARTING_ANGLE) / 2);
    m_pidController.setReference((position - STARTING_ANGLE) / 2, ControlType.kSmartMotion);
    double error = position - currentPos;
    return Math.abs(error) < 3;
  }

  public double armZDistance() {
    double angle = normalizeAngle(tower.returnAngle());
    double distance =
        armXZDistance() * Math.sin(Math.toRadians(angle))
            + GRABBER_WIDTH * Math.cos(Math.toRadians(angle));
    return distance;
  }

  public double normalizeAngle(double angle) {
    angle = Math.abs(angle % 90);
    if (angle > 45) {
      angle = 90 - angle;
    }
    return angle;
  }

  public boolean isAtLeftFrame() {
    double angle = tower.returnAngle() % 90;
    String quadrant = checkQuadrant();
    double limit = 0;
    if (angle > 0) {
      return false;
    }
    if (quadrant == "front") {
      limit = 11.25;
    } else if (quadrant == "left") {
      limit = 10;
    } else if (quadrant == "right") {
      limit = 21.75;
    } else {
      limit = 11.25;
    }

    if (armZDistance() > limit) {
      return true;
    } else {
      return false;
    }
  }

  public boolean isAtRightFrame() {
    double angle = tower.returnAngle() % 90;
    String quadrant = checkQuadrant();
    double limit = 0;
    if (angle < 0) {
      return false;
    }
    if (quadrant == "front") {
      limit = 11.25;
    } else if (quadrant == "left") {
      limit = 21.75;
    } else if (quadrant == "right") {
      limit = 10;
    } else {
      limit = 11.25;
    }

    if (armZDistance() > limit) {
      return true;
    } else {
      return false;
    }
  }

  public boolean isAtLowerArmLimit() {
    if (currentPos < 15 || armXDistance() < 2) {
      return true;
    } else {
      return false;
    }
  }

  public boolean isAtSlowLimit() {
    if (currentPos > 60) {
      return true;
    } else {
      return false;
    }
  }

  public double returnAngle() {
    return currentPos;
  }

  public void SmartDashboard() {
    /*
    SmartDashboard.putNumber("arm x distance ", armXDistance());
    SmartDashboard.putNumber("arm y distance", armYDistance());
    SmartDashboard.putNumber("current angle", currentPos);
    SmartDashboard.putNumber("grabber extension", extender.returnExtension());
    SmartDashboard.putNumber("arm encoder", m_encoder.getPosition());
    SmartDashboard.putString("quadrant ", checkQuadrant());
    SmartDashboard.putNumber("Arm angle", currentPos);
    */
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("armMotorOutput", m_arm.getAppliedOutput());
  }
}
