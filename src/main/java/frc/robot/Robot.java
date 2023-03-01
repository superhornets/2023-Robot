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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import org.opencv.core.Mat;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.GenericEntry;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
 
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
  private String quadrant = "a";
  private boolean isRotatingToQuadrant = false;
  private double extenderSpeed = 1;
  private boolean override = false;
  private boolean holdPositionTurret = false;
  private boolean limitFramePerimiter = false;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
  
    drive.driveInit();
    drive.NavXInit();
    
    SmartDashboard.putNumber("Distance", distance);
    SmartDashboard.putNumber("angle", angle);
    SmartDashboard.putBoolean("hold Position", holdMode);
    SmartDashboard.putNumber("tower encoder", pickerUpper.tower.getPosition());
    CameraServer.startAutomaticCapture();



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
    pickerUpper.SmartDashboardPrintout();
    extenderSpeed=SmartDashboard.getNumber("extender speed", extenderSpeed);
    //GenericEntry override = Shuffleboard.getTab("override").add("override", false).withWidget("Toggle Button").getEntry();
    //pickerUpper.tower.safety(SmartDashboard.getBoolean("override", false));
    pickerUpper.grabber.periodic();
    //SmartDashboard.putNumber("extender speed", extenderSpeed);
    pickerUpper.arm.updatePosition();
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
  
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

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
    //boolean holdMode = SmartDashboard.getBoolean("holdPosition", false);

    if(m_leftStick.getX() > .07 || m_leftStick.getX() < -.07 || m_leftStick.getY() > .07 || m_leftStick.getY() < -.07){

      drive.arcade(m_leftStick.getY(), m_leftStick.getX());
    }
    else if(!isAutoDriving){
      if(holdMode){
      drive.holdPosition();
      } 
    }
    else{
      drive.holdSpeed(0);
     }
      
    //}
    /*if(isAutoDriving){
      if(drive.driveTo(distance)){
        isAutoDriving = false;
      }
    
      if(Drive.isDriving(distance)){
        isAutoDriving = false;
        System.out.println("is driving true 2");

      }
    }*/
    /*if(m_leftStick.getRawButton(5)){
      drive.stopRotation();
      isTurning = false;
    }*/
    if(m_leftStick.getRawButton(6)){
        holdMode = false;
    } 
    if(m_leftStick.getRawButton(7)){
      holdMode = true;
    }
    if(m_leftStick.getRawButton(4)){
      drive.resetNavX();
    }



    //Grabber code
    if (m_rightStick.getRawButton(1)){
      pickerUpper.grabber.close();
    }
    else if (m_rightStick.getRawButton(2)){
      pickerUpper.grabber.open();
    }
    else{
      pickerUpper.grabber.hold();
    }
    


    if(pickerUpper.arm.isOverExtentionLimit() || pickerUpper.arm.isOverHeightLimit()){
      if(pickerUpper.grabber.returnExtension() > 0){
        pickerUpper.grabber.extend(-.1);
      }
      else{
        pickerUpper.arm.moveArm(-.1);
      }
    }
    else if(pickerUpper.arm.isAtExtentionLimit()){
      /*double towerSpeed = m_rightStick.getRawAxis(2);
      if(Math.abs(pickerUpper.tower.returnAngle()+45)%90>45){
        if(towerSpeed > 0){
          pickerUpper.tower.moveTower(towerSpeed);
        }
      }
      else{
        if(towerSpeed < 0){
          pickerUpper.tower.moveTower(towerSpeed);
        }
      }*/
      if(m_rightStick.getRawButton(8)){
        pickerUpper.grabber.extend(-.1);
      }
      if(m_rightStick.getRawAxis(1) < 0){
        double armSpeed = m_rightStick.getRawAxis(1);
        pickerUpper.arm.moveArm(armSpeed);
      }

    }
    else if(pickerUpper.arm.isAtHeightLimit()){
      if(m_rightStick.getRawButton(8)){
        pickerUpper.grabber.extend(-.1);
      }
      if(m_rightStick.getRawAxis(1) < 0){
        double armSpeed = m_rightStick.getRawAxis(1);
        pickerUpper.arm.moveArm(armSpeed);
      }
      double towerSpeed = m_rightStick.getRawAxis(0);
      pickerUpper.tower.moveTower(towerSpeed);
    }
    else{
    //Arm code
    double armSpeed = m_rightStick.getRawAxis(1);
    if(armSpeed < .05 && armSpeed > -.05){
      armSpeed = 0;
    }
    pickerUpper.arm.moveArm(armSpeed);
    if(m_rightStick.getRawButton(5)){
      pickerUpper.arm.reseZero();
    }
  
    
    //Extender code
    if(m_rightStick.getRawButton(7) && (pickerUpper.grabber.returnExtension() < 15 || m_rightStick.getRawButton(6))){
      pickerUpper.grabber.extend(extenderSpeed);
    }
    else if(m_rightStick.getRawButton(8) && (pickerUpper.grabber.returnExtension() > 0|| m_rightStick.getRawButton(6))){
      pickerUpper.grabber.extend(-extenderSpeed);
    }
    else if(m_rightStick.getRawButton(9)){
      pickerUpper.grabber.extendToPos(5);
    }
    else if(m_rightStick.getRawButton(10)){
      pickerUpper.grabber.extendToPos(-5);
    }
    else{
      pickerUpper.grabber.extend(0);
    }
    
  }
    //Tower Code
if(m_rightStick.getRawButton(3)){
  holdPositionTurret = !holdPositionTurret;
}
if(m_rightStick.getRawButton(4)){
  limitFramePerimiter = !limitFramePerimiter;
}
    if (m_rightStick.isConnected()) {
      double towerSpeed = m_rightStick.getRawAxis(0);
      if(Math.abs(towerSpeed)> .05 && !holdPositionTurret){
        if((pickerUpper.tower.returnAngle() > 60 && towerSpeed > 0) || (pickerUpper.tower.returnAngle() < -60 && towerSpeed < 0)){
          pickerUpper.tower.moveTower(0);
        }
        else if(((towerSpeed > 0 && pickerUpper.arm.isAtRightFrame()) || (towerSpeed < 0 && pickerUpper.arm.isAtLeftFrame())) && limitFramePerimiter){
          pickerUpper.tower.moveTower(0);
        }
        else{
        pickerUpper.tower.moveTower(towerSpeed);
        }
      }
      /*else if(holdPositionTurret){
        pickerUpper.tower.holdTowerPos();
      }*/
      else if (m_rightStick.getRawButton(11)) {
        pickerUpper.tower.setZero();
      }
      else if (isRotatingToQuadrant == false) {
        if(m_rightStick.getPOV() == 0) {
          quadrant = "a";
          isRotatingToQuadrant = true;
          //pickerUpper.tower.moveTowerToQuadrant("a");
        } 
        else if(m_rightStick.getPOV() == 90) {
          quadrant = "b";
          isRotatingToQuadrant = true;
          //pickerUpper.tower.moveTowerToQuadrant("b");
        }
        else if(m_rightStick.getPOV() == 180) {
          quadrant = "c";
          isRotatingToQuadrant = true;
          //pickerUpper.tower.moveTowerToQuadrant("c");
        }
        else if(m_rightStick.getPOV() == 270) {
          quadrant = "d";
          isRotatingToQuadrant = true;
          //pickerUpper.tower.moveTowerToQuadrant("d");
        }
      }
      else if(isRotatingToQuadrant){
        if(pickerUpper.tower.moveTowerToQuadrant(quadrant)){
          isRotatingToQuadrant = false;
        }
      }
      else{
        pickerUpper.tower.moveTower(0);
      }
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