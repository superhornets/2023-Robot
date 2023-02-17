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
  private static final String kBrokenArmAuto = "Broken Arm";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final Joystick m_leftStick = new Joystick(0);
  private final Joystick m_rightStick = new Joystick(1);
  //private final DifferentialDrive drive = new DifferentialDrive(leftFrontDrive, rightFrontDrive);
  private SparkMaxPIDController m_pidController;
  private SparkMaxPIDController m_pidControllerR;
  private RelativeEncoder m_encoder;
  private RelativeEncoder m_encoderR;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
  private /*final*/ double ROTATIONS_PER_INCH = .5694;
  private double distance = 24;
  private PickerUpper pickerUpper = new PickerUpper();
  private boolean isAutoDriving = false;
  private Drive drive = new Drive();
  private Auto auto = new Auto(drive, pickerUpper);
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
    m_chooser.addOption("Broken Arm", kBrokenArmAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    drive.driveInit();
    drive.NavXInit();
    
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
    drive.SmartDashboardPrintout();
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
          if(drive.driveTo(-168)){
            autoStage = 1;
          }
        }
        else if(autoStage == 1){
          if(drive.turnTo(-90)){
            autoStage = 2;
          }
        }
        else if(autoStage == 2){
          if(drive.driveTo(54)){
            autoStage = 3;
          }
        }
        else if(autoStage == 3){
          if(drive.turnTo(0)){
            autoStage = 4;
            drive.levelInit();
          }
        }
        else if(autoStage == 4){
          if(drive.level()){
            autoStage = 5;
            drive.setPos();
          }
        }
        else{
          drive.holdPosition();
        }


      case kBrokenArmAuto:
        if (autoStage == 0){
          // Back up
          if(drive.driveTo(-24)){
            autoStage = 1;
          }
        }
        else if(autoStage == 1){
          // Forward
          if(drive.driveTo(30)){
            autoStage = 2;
          }
        }
        if (autoStage == 2){
          // Back up
          if(drive.driveTo(-24)){
            autoStage = 3;
          }
        }
          // ;)
        
        else if(autoStage == 3){
          // Turn 180 degrees
          if(drive.turnTo(180)){
            autoStage = 4;
          }
        }
        else if(autoStage == 4){
          // Forward until hit game piece
          if(drive.driveTo(144)){
            autoStage = 5;
          }
        }
        else if(autoStage == 5){
          // Turn 180 degrees in place
          if(drive.turnTo(180)){
            autoStage = 6;
          }
        }
        else if(autoStage == 6){
          // Move to node
          if(drive.driveTo(144)){
            autoStage = 7;
          }
        }

        else if(autoStage == 7){
          // Forward
          if(drive.driveTo(24)){
            autoStage = 8;
          }
        }
        else if(autoStage == 8){
          // Back up
          if(drive.driveTo(-24)){
            autoStage = 9;
          }
        }
        else if(autoStage == 9){
          // Turn "A bit"
          if(drive.turnTo(20)){
            autoStage = 10;
          }
        }
        else if(autoStage == 10){
          // Forward
          if(drive.driveTo(6)){
            autoStage = 11;
          }
        }
        break; 
      case kDefaultAuto:
        if(autoStage == 0){
          drive.driveOverInit();
          autoStage = 1;
        }
        else if(autoStage == 1){
          if(drive.driveOverChargingStation()){
            autoStage = 2;
          }
        }
        else if(autoStage == 2){
          if(drive.turnTo(180)){
            autoStage = 3;
          }
        }
        else if(autoStage == 3){
          if(drive.level()){
            autoStage = 4;
            drive.setPos();
          }
        }
        else if(autoStage == 4){
          drive.holdPosition();
        }

        
      case kStraightAuto:
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
    drive.teleopInitDrive();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double distance = SmartDashboard.getNumber("Distance", 0);
    double angle = SmartDashboard.getNumber("angle", 0);
    boolean holdMode = SmartDashboard.getBoolean("holdPosition", false);

    if(m_leftStick.getX() > .05 || m_leftStick.getX() < -.05 || m_leftStick.getY() > .05 || m_leftStick.getY() < -.05){

      drive.arcade(m_leftStick.getY(), m_leftStick.getX());
    }
    else if(isAutoLeveling){
      if(drive.level()){
        isAutoLeveling = false;
        System.out.println("stopped leveling");
      }
    }
    else if(m_leftStick.getRawButton(3) && !isAutoDriving){
      drive.driveTo(distance);
      isAutoDriving = true;
      //System.out.println("button 3");
    }
    else if(!isAutoDriving){
      if(holdMode){
      drive.holdPosition();
     } 
     else if(isTurning){
      if(drive.turnTo(angle)){
        isTurning = false;
      }
      
    }

     else{
      drive.holdSpeed(0);
     }
      
    }
    if(isAutoDriving){
      if(drive.driveTo(distance)){
        isAutoDriving = false;
      }
    
      /*if(Drive.isDriving(distance)){
        isAutoDriving = false;
        System.out.println("is driving true 2");

      }*/
    }
    if(m_leftStick.getRawButton(5)){
      drive.stopRotation();
      isTurning = false;
    }
    if(m_leftStick.getRawButton(6)){
        holdMode = false;
    } 
      
    if(m_leftStick.getRawButton(7)){
      holdMode = true;
    }
    if(m_leftStick.getRawButton(4)){
      drive.resetNavX();
    }
    if(m_leftStick.getRawButton(11)){
      isAutoLeveling = true;
      drive.levelInit();
    }
    if(m_leftStick.getRawButton(10)){
      isAutoLeveling = false;
    }
    if (m_rightStick.getRawButton(1)){
      pickerUpper.grabber.closeCone();
    }
    else if (m_rightStick.getRawButton(2)){
      pickerUpper.grabber.open();
    }
    
    double armSpeed = m_rightStick.getRawAxis(1);
    pickerUpper.arm.moveArm(armSpeed);
  
    double towerSpeed = m_rightStick.getRawAxis(2);
    pickerUpper.tower.moveTower(towerSpeed);

    if(m_leftStick.getRawButton(2)){
      drive.turnTo(angle);
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