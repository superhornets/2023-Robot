package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SparkMaxPIDWrapper {
  private final SparkMaxPIDController pidController;
  private final CachedPIDValues cache;
  private final String dashboardPrefix;

  SparkMaxPIDWrapper(CANSparkMax canSparkMax, String dashbaordPrefix) {
    pidController = canSparkMax.getPIDController();
    cache = new CachedPIDValues(pidController);
    this.dashboardPrefix = dashbaordPrefix += " ";
    updatePIDOnDashboard();
  }

  public void setSmartVelocity(double velocitySetpoint) {
    pidController.setReference(velocitySetpoint, ControlType.kSmartVelocity);
  }

  public void setSmartPosition(double positionSetpoint) {
    pidController.setReference(positionSetpoint, ControlType.kSmartMotion);
  }

  /**
   * Compares the input PID values with a local cache and sends the input values to the motor
   * controller and smart dashboard only if necessary.
   *
   * <p>The cache is used because the pidController.set... methods are documented to use the Set
   * Parameter API and should be used infrequently.
   */
  public void setPID(
      double p,
      double i,
      double d,
      double iZone,
      double feedForward,
      double maxOutput,
      double minOutput,
      double maxVel,
      double maxAcc) {
    setPID(true, p, i, d, iZone, feedForward, maxOutput, minOutput, maxVel, maxAcc);
  }

  /**
   * Should be called all the time. If PID values were sent to the smart dashboard, gets the PID
   * values from the smart dashboard and sets the PID.
   */
  public void periodic() {
    if (!dashboardPrefix.isEmpty()) {
      double newP = SmartDashboard.getNumber("P", cache.p);
      double newI = SmartDashboard.getNumber("I", cache.i);
      double newD = SmartDashboard.getNumber("D", cache.d);
      double newIZone = SmartDashboard.getNumber("iZone", cache.iZone);
      double newFeedForward = SmartDashboard.getNumber("Feed Forward", cache.feedForward);
      double newMaxOutput = SmartDashboard.getNumber("Max Output", cache.maxOutput);
      double newMinOutput = SmartDashboard.getNumber("Min Output", cache.minOutput);
      double newMaxVel = SmartDashboard.getNumber("Max Vel", cache.maxVel);
      double newMaxAcc = SmartDashboard.getNumber("Max Acc", cache.maxAcc);
      setPID(
          false,
          newP,
          newI,
          newD,
          newIZone,
          newFeedForward,
          newMaxOutput,
          newMinOutput,
          newMaxVel,
          newMaxAcc);
    }
  }

  /**
   * See documentation for the public version of this method. This method does the actual work and
   * has an updateDashboard parameter so that it can be called from its own class without updating
   * the dashboard.
   */
  private void setPID(
      boolean updateDashboard,
      double p,
      double i,
      double d,
      double iZone,
      double feedForward,
      double maxOutput,
      double minOutput,
      double maxVel,
      double maxAcc) {
    // TODO setting values to the pidController can return an error. This error is ignored.
    if (p != cache.p) {
      pidController.setP(p);
      cache.p = p;
    }
    if (i != cache.i) {
      pidController.setI(i);
      cache.i = i;
    }
    if (d != cache.d) {
      pidController.setD(d);
      cache.d = d;
    }
    if (iZone != cache.iZone) {
      pidController.setIZone(iZone);
      cache.iZone = iZone;
    }
    if (feedForward != cache.feedForward) {
      pidController.setFF(feedForward);
      cache.feedForward = feedForward;
    }
    if (maxOutput != cache.maxOutput || minOutput != cache.minOutput) {
      pidController.setOutputRange(minOutput, maxOutput);
      cache.minOutput = minOutput;
      cache.maxOutput = maxOutput;
    }
    if (maxVel != cache.maxVel) {
      pidController.setSmartMotionMaxVelocity(maxVel, 0);
      cache.maxVel = maxVel;
    }
    // TODO is `pidController.setSmartMotionMinOutputVelocity(minVel, slotID)` the equivelent to
    // `.setSmartMotionMaxVelocity(maxVel, slotID)` but for Min instead of Max?
    if (maxAcc != cache.maxAcc) {
      pidController.setSmartMotionMaxAccel(maxAcc, 0);
      cache.maxAcc = maxAcc;
    }
    if (updateDashboard) {
      updatePIDOnDashboard();
    }
  }

  private void updatePIDOnDashboard() {
    if (!this.dashboardPrefix.isEmpty()) {
      SmartDashboard.putNumber(this.dashboardPrefix + "P", cache.p);
      SmartDashboard.putNumber(this.dashboardPrefix + "I", cache.i);
      SmartDashboard.putNumber(this.dashboardPrefix + "D", cache.d);
      SmartDashboard.putNumber(this.dashboardPrefix + "iZone", cache.iZone);
      SmartDashboard.putNumber(this.dashboardPrefix + "Feed Forward", cache.feedForward);
      SmartDashboard.putNumber(this.dashboardPrefix + "Max Output", cache.maxOutput);
      SmartDashboard.putNumber(this.dashboardPrefix + "Min Output", cache.minOutput);
      SmartDashboard.putNumber(this.dashboardPrefix + "Max Vel", cache.maxVel);
      SmartDashboard.putNumber(this.dashboardPrefix + "Max Acc", cache.maxAcc);
    }
  }

  public class CachedPIDValues {
    private double p, i, d, iZone, feedForward, maxOutput, minOutput, maxVel, maxAcc;

    CachedPIDValues(SparkMaxPIDController pidController) {
      p = pidController.getP();
      i = pidController.getI();
      d = pidController.getD();
      iZone = pidController.getIZone();
      feedForward = pidController.getFF();
      maxOutput = pidController.getOutputMax();
      minOutput = pidController.getOutputMin();
      maxVel = pidController.getSmartMotionMaxVelocity(0);
      maxAcc = pidController.getSmartMotionMaxAccel(0);
    }
  }
}
