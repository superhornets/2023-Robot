// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private static final String kStraightAuto = "Straight auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final Joystick m_leftStick = new Joystick(0);
  private final Joystick m_rightStick = new Joystick(1);
  private double distance = 24;
  private PickerUpper pickerUpper = new PickerUpper();
  private boolean isAutoDriving = false;
  private Drive Drive = new Drive();
  private double angle = 0;
  private boolean isTurning = false;
  private boolean holdMode = true;
  private boolean isAutoLeveling = false;
  private int autoStage = 0;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    Drive.driveInit();
    Drive.NavXInit();
    
    SmartDashboard.putNumber("Distance", distance);
    SmartDashboard.putNumber("angle", angle);
    SmartDashboard.putBoolean("hold Position", holdMode);

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
    Drive.SmartDashboardPrintout();
    SmartDashboard.putNumber("leftStick", m_leftStick.getY());
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
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    autoStage = 0;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        if (autoStage == 0){
          if(Drive.driveTo(-168)){
            autoStage = 1;
          }
        }
        else if(autoStage == 1){
          if(Drive.turnTo(-90)){
            autoStage = 2;
          }
        }
        else if(autoStage == 2){
          if(Drive.driveTo(54)){
            autoStage = 3;
          }
        }
        else if(autoStage == 3){
          if(Drive.turnTo(0)){
            autoStage = 4;
            Drive.levelInit();
          }
        }
        else if(autoStage == 4){
          if(Drive.level()){
            autoStage = 5;
            Drive.setPos();
          }
        }
        else{
          Drive.holdPosition();
        }
      case kStraightAuto:
        if(autoStage == 0){
          Drive.driveOverInit();
          autoStage = 1;
        }
        else if(autoStage == 1){
          if(Drive.driveOverChargingStation()){
            Drive.levelInit();
            autoStage = 2;
          }
        }
        else if(autoStage == 2){
          if(Drive.level()){
            autoStage = 3;
            Drive.setPos();
          }
        }
        else if(autoStage == 3){
          Drive.holdPosition();
        }

        
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    /*currentPos=m_encoder.getPosition();
    currentPosR = m_encoderR.getPosition();
    isMoving = false; */
    Drive.teleopInitDrive();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double distance = SmartDashboard.getNumber("Distance", 0);
    double angle = SmartDashboard.getNumber("angle", 0);
    boolean holdMode = SmartDashboard.getBoolean("holdPosition", false);

    if(m_leftStick.getX() > .05 || m_leftStick.getX() < -.05 || m_leftStick.getY() > .05 || m_leftStick.getY() < -.05){

      Drive.arcade(m_leftStick.getY(), m_leftStick.getX());
    }
    else if(isAutoLeveling){
      if(Drive.level()){
        isAutoLeveling = false;
        System.out.println("stopped leveling");
      }
    }
    else if(m_leftStick.getRawButton(3) && !isAutoDriving){
      Drive.driveTo(distance);
      isAutoDriving = true;
      //System.out.println("button 3");
    }
    else if(!isAutoDriving){
      if(holdMode){
      Drive.holdPosition();
     } 
     else if(isTurning){
      if(Drive.turnTo(angle)){
        isTurning = false;
      }
      
    }

     else{
      Drive.holdSpeed(0);
     }
      
    }
    if(isAutoDriving){
      if(Drive.driveTo(distance)){
        isAutoDriving = false;
      }
    
      /*if(Drive.isDriving(distance)){
        isAutoDriving = false;
        System.out.println("is driving true 2");

      }*/
    }
    if(m_leftStick.getRawButton(5)){
      Drive.stopRotation();
      isTurning = false;
    }
    if(m_leftStick.getRawButton(6)){
        holdMode = false;
    } 
      
    if(m_leftStick.getRawButton(7)){
      holdMode = true;
    }
    if(m_leftStick.getRawButton(4)){
      Drive.resetNavX();
    }
    if(m_leftStick.getRawButton(11)){
      isAutoLeveling = true;
      Drive.levelInit();
    }
    if(m_leftStick.getRawButton(10)){
      isAutoLeveling = false;
    }
    
    double armSpeed = m_rightStick.getRawAxis(1);
    pickerUpper.moveArm(armSpeed);
  
    double towerSpeed = m_rightStick.getRawAxis(2);
    pickerUpper.moveTower(towerSpeed);

    if(m_leftStick.getRawButton(2)){
      Drive.turnTo(angle);
      isTurning = true;
    }
    
    SmartDashboard.putBoolean("isAutoDriving", isAutoDriving);
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

//:)