// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private final Joystick m_leftStick = new Joystick(0);
  private final Joystick m_rightStick = new Joystick(1);

  private PickerUpper pickerUpper = new PickerUpper();
  private Drive drive = new Drive();
  private Auto auto = new Auto(drive, pickerUpper);
  private boolean holdMode = true;
  private double extenderSpeed = 1;
  private boolean holdPositionTurret = false;
  private boolean limitFramePerimiter = false;
  private boolean isPlacing = false;
  private boolean isSlowMode = false;
  private boolean isPlacingHigh = false;
  private boolean isPickingUp = false;
  private boolean isRotatingToCube = false;
  private boolean isLightPattern = false;
  private boolean isHoming = false;
  private int lightMode = 0;
  private AddressableLED m_led = new AddressableLED(9);
  private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(68);
  private boolean notPitOverride = false;
  private boolean override = false;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    drive.driveInit();
    drive.NavXInit();
    m_led.setLength(m_ledBuffer.getLength());
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 155, 100, 0);
    }

    m_led.setData(m_ledBuffer);
    m_led.start();

    SmartDashboard.putNumber("Target", 2);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    drive.SmartDashboardPrintout();
    pickerUpper.grabber.periodicGrabber();
    pickerUpper.arm.periodic();
    pickerUpper.arm.updatePosition();
    if (DriverStation.isFMSAttached()) {
      notPitOverride = true;
    } else if (m_leftStick.getRawButtonPressed(11)) {
      notPitOverride = !notPitOverride;
      System.out.println("button 11 pressed");
      if (notPitOverride) {
        lightMode = 0;
        isLightPattern = true;
      } else {
        lightMode = 3;
        isLightPattern = true;
      }

    } else if (!notPitOverride && lightMode != 3) {
      lightMode = 3;
      isLightPattern = true;
    } else if (m_leftStick.getRawButtonPressed(10)) {
      lightMode += 1;
      if (lightMode > 2) {
        lightMode = 0;
      }
      isLightPattern = true;
    }
    SmartDashboard.putBoolean("pit safety", notPitOverride);
    pickerUpper.tower.pitSafety(notPitOverride);

    if (lightMode == 1 && isLightPattern) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        if (i % 2 == 0) {
          m_ledBuffer.setRGB(i, 155, 100, 0);
        } else {
          m_ledBuffer.setRGB(i, 0, 0, 0);
        }
      }
      isLightPattern = false;
      System.out.println("Light Change 1  ");
      m_led.setData(m_ledBuffer);
      m_led.start();
    } else if (lightMode == 2 && isLightPattern) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        if (i % 2 == 0) {
          m_ledBuffer.setRGB(i, 155, 0, 155);
        } else {
          m_ledBuffer.setRGB(i, 0, 0, 0);
        }
      }
      isLightPattern = false;
      m_led.setData(m_ledBuffer);
      m_led.start();
      System.out.println("Light Change 2");

    } else if (lightMode == 0 && isLightPattern) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        m_ledBuffer.setRGB(i, 155, 100, 0);
      }

      m_led.setData(m_ledBuffer);
      m_led.start();
      isLightPattern = false;
      System.out.println("Light Change 0");

    } else if (lightMode == 3 && isLightPattern) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        // final var hue = (0+(i*180/m_ledBuffer.getLength()))%180;
        // m_ledBuffer.setHSV(i, hue, 255, 128);
        m_ledBuffer.setRGB(i, 0, 155, 0);
      }

      m_led.setData(m_ledBuffer);
      m_led.start();
      isLightPattern = false;
      System.out.println("Light Change 3");
    }
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    auto.updateSelection();
    auto.autoInit();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    auto.runSelected();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    drive.teleopInitDrive();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putBoolean("Slow Mode", isSlowMode);
    if (m_leftStick.getRawButtonPressed(3)) {
      isSlowMode = !isSlowMode;
    }
    if (m_leftStick.getX() > .05
        || m_leftStick.getX() < -.05
        || m_leftStick.getY() > .05
        || m_leftStick.getY() < -.05) {
      if (isSlowMode) {
        drive.arcadeTeleop1(m_leftStick.getY() * .1, m_leftStick.getX() * .2);
        System.out.println("is driving slowly");
      } else {
        drive.arcadeTeleop1(m_leftStick.getY() * .75, m_leftStick.getX() * .75);
      }
      drive.setPos();
    } else if (holdMode) {
      drive.holdPosition();
    } else {
      drive.arcadeTeleop1(0, 0);
    }

    if (m_leftStick.getRawButton(6)) {
      holdMode = false;
    }
    if (m_leftStick.getRawButton(7)) {
      holdMode = true;
    }
    if (m_leftStick.getRawButton(4)) {
      drive.resetNavX();
    }
    if (m_rightStick.getRawButton(1)) {
      pickerUpper.grabber.close();
    } else if (m_rightStick.getRawButton(2)) {
      pickerUpper.grabber.open();
    } else {
      pickerUpper.grabber.hold();
    }

    if (!isPlacing && !isPlacingHigh && !isPickingUp) {

      // Arm code
      double armSpeed = m_rightStick.getRawAxis(1);
      pickerUpper.arm.moveArm(armSpeed, isSlowMode);
      if (m_rightStick.getRawButton(11) && !m_rightStick.getRawButton(12)) {
        pickerUpper.arm.resetZero();
      } else if (m_rightStick.getRawButton(6)) {
        pickerUpper.arm.moveArmTo(40);
      }

      // Extender code
      if (m_rightStick.getRawButton(11) && !m_rightStick.getRawButton(12)) {
        override = true;
      } else {
        override = false;
      }
      if ((m_rightStick.getRawButton(7) && !m_rightStick.getRawButton(12))) {
        pickerUpper.extender.extend(extenderSpeed, override);
      } else if ((m_rightStick.getRawButton(8) && !m_rightStick.getRawButton(12))) {
        pickerUpper.extender.extend(-extenderSpeed, override);
      } else if (m_rightStick.getRawButton(11) && !m_rightStick.getRawButton(12)) {
        pickerUpper.extender.resetExtenderEncoder();
      } else {
        pickerUpper.extender.extend(0, false);
      }
    }

    // Tower Code
    if (m_rightStick.getRawButtonPressed(3)) {
      holdPositionTurret = !holdPositionTurret;
    }

    if (m_rightStick.isConnected()) {
      double towerSpeed = m_rightStick.getRawAxis(0);
      if (Math.abs(towerSpeed) > .04 && !holdPositionTurret) {
        pickerUpper.tower.moveTower(towerSpeed, override, isSlowMode);
      } else if (holdPositionTurret) {
        pickerUpper.tower.holdTowerPos();
      } else if (m_rightStick.getRawButton(11) && !m_rightStick.getRawButton(12)) {
        pickerUpper.tower.setZero();
      } else if (isRotatingToCube) {
        if (pickerUpper.tower.rotateToCube()) {
          isRotatingToCube = false;
        }
      } else if (m_rightStick.getRawButton(7) && m_rightStick.getRawButton(12)) {
        pickerUpper.tower.rotateToCubeInit();
        isRotatingToCube = true;
      } else {
        pickerUpper.tower.moveTower(0, false, false);
      }
    }

    // auto pickup and place
    if (m_leftStick.getRawButton(8)) {
      isPlacing = false;
      isPlacingHigh = false;
      isPickingUp = false;
      isHoming = false;
    }
    if (m_rightStick.getRawButton(10) && !m_rightStick.getRawButton(12)) {
      if (!isPlacing) {
        isPlacing = true;
        int target = (int) SmartDashboard.getNumber("Target", 2);
        auto.placePieceInit(target);
      }
    } else if (isPlacing) {
      int target = (int) SmartDashboard.getNumber("Target", 2);
      if (auto.placePiece(target, true, true)) {
        isPlacing = false;
      }
    }

    /*if (m_rightStick.getRawButton(9) && !m_rightStick.getRawButton(12) && !isPlacingHigh) {
      auto.placePieceAutoBySetpointInit();
      isPlacingHigh = true;
    } else if (isPlacingHigh) {
      if (auto.placePieceAutoBySetpoint()) {
        isPlacingHigh = false;
      }
    }*/

    if (m_rightStick.getRawButton(4) && !isHoming) {
      isHoming = true;
      auto.homePickerUpperInit();
    } else if (isHoming) {
      if (auto.homePickerUpper()) {
        isHoming = false;
      }
    }
    if (m_rightStick.getRawButton(10) && m_rightStick.getRawButton(12) && !isPickingUp) {
      isPickingUp = true;
      auto.pickupPieceAutoBySetpointInit();
    } else if (isPickingUp) {
      if (auto.pickupPieceAutoBySetpoint()) {
        isPickingUp = false;
      }
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}

// :)
