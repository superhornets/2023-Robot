package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class Extender {
  private final CANSparkMax extender = new CANSparkMax(8, MotorType.kBrushless);
  private SparkMaxPIDController m_pidController;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, maxAcc;
  private RelativeEncoder m_encoder;
  private final double GEAR_RATIO = 38.3;
  private Arm arm;

  public Extender() {
    m_pidController = extender.getPIDController();
    m_encoder = extender.getEncoder();
    extender.setInverted(true);
    kP = 1e-5;
    kI = 8e-7;
    kD = 0;
    kIz = 0;
    kFF = 0;
    maxVel = 3000;
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
  }

  public void setPickerUpper(Arm arm) {
    this.arm = arm;
  }

  public void extend(double speed, boolean override) {

    if ((arm.isAtExtentionLimit() || arm.isAtHeightLimit() || (returnExtension() > 14 && !override))
        && speed > 0) {
      speed = 0;
    }
    if ((returnExtension() < 0 && !override) && speed < 0) {
      speed = 0;
    }
    extender.set(speed);
  }

  public boolean extendToPos(double Pos) {
    double target = (Pos * GEAR_RATIO);
    m_pidController.setReference(target, ControlType.kSmartMotion);
    return Math.abs(Pos - returnExtension()) < 1;
  }

  public boolean retractToPos(double Pos) {
    return false;
  }

  public void resetExtenderEncoder() {
    m_encoder.setPosition(0);
  }

  public double returnExtension() {

    return m_encoder.getPosition() / GEAR_RATIO;
  }
}
