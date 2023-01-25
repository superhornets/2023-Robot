// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final CANSparkMax leftFrontDrive = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax leftRearDrive = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax rightFrontDrive = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax rightRearDrive = new CANSparkMax(4, MotorType.kBrushless);
  //private final MotorControllerGroup leftDrive = new MotorControllerGroup(leftFrontDrive, leftRearDrive);
  //private final MotorControllerGroup rightDrive = new MotorControllerGroup(rightFrontDrive, rightRearDrive);
  private final Joystick m_leftStick = new Joystick(0);
  private final Joystick m_rightStick = new Joystick(1);
  private final DifferentialDrive drive = new DifferentialDrive(leftFrontDrive, rightFrontDrive);
  private SparkMaxPIDController m_pidController;
  private SparkMaxPIDController m_pidControllerR;
  private RelativeEncoder m_encoder;
  private RelativeEncoder m_encoderR;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private /*final*/ double ROTATIONS_PER_INCH = .5694;
  private double distance = -24;
  private boolean isMoving = false;
  private double currentPos = 0;
  private double currentPosR = 0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    leftFrontDrive.setInverted(true);
    leftRearDrive.setInverted(true);
    m_pidController = leftFrontDrive.getPIDController();
    m_pidControllerR = rightFrontDrive.getPIDController();
    // Encoder object created to display position values
    m_encoder = leftFrontDrive.getEncoder();
    m_encoderR = rightFrontDrive.getEncoder();
    kP = 0.1; 
    kI = 1e-4;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = .25; 
    kMinOutput = -.25;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    m_pidControllerR.setP(kP);
    m_pidControllerR.setI(kI);
    m_pidControllerR.setD(kD);
    m_pidControllerR.setIZone(kIz);
    m_pidControllerR.setFF(kFF);
    m_pidControllerR.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);
    SmartDashboard.putNumber("Rotations per inch", ROTATIONS_PER_INCH);
    SmartDashboard.putNumber("Distance", distance);
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
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here

        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double rotations = SmartDashboard.getNumber("Set Rotations", 0);
    double ROTATIONS_PER_INCH = SmartDashboard.getNumber("rotations per inch", .5694);
    double distance = SmartDashboard.getNumber("Distance", 0);
    SmartDashboard.putNumber("encoder", m_encoder.getPosition());

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidController.setP(p); kP = p;  m_pidControllerR.setP(p); }
    if((i != kI)) { m_pidController.setI(i); kI = i; m_pidControllerR.setI(i);}
    if((d != kD)) { m_pidController.setD(d); kD = d; m_pidControllerR.setD(d);}
    if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; m_pidControllerR.setIZone(iz); }
    if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; m_pidControllerR.setFF(ff);}
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.setOutputRange(min, max); {}
      kMinOutput = min; kMaxOutput = max; m_pidControllerR.setOutputRange(min, max); }
      drive.arcadeDrive(-m_leftStick.getY()*.75, m_leftStick.getX()*.75);
    //if(m_leftStick.getX() > .05 || m_leftStick.getX() < -.05 || m_leftStick.getY() > .05 || m_leftStick.getY() < -.05){
      //drive.arcadeDrive(d, rotations);
     // var speeds = drive.arcadeDriveIK(m_leftStick.getY(), m_leftStick.getX(), true);
     //// m_pidController.setReference(speeds.left, com.revrobotics.CANSparkMax.ControlType.kVelocity);
     // m_pidControllerR.setReference(speeds.right, com.revrobotics.CANSparkMax.ControlType.kVelocity);
    //}
    /*if(m_leftStick.getRawButton(3) && !isMoving){
      currentPos=m_encoder.getPosition();
      currentPosR = m_encoderR.getPosition();
      isMoving = true;
    }
    if(m_encoder.getPosition() <= (distance+.5)*ROTATIONS_PER_INCH && m_encoder.getPosition() >= (distance-.5)*ROTATIONS_PER_INCH){
      isMoving = false;
      //currentPos = m_encoder.getPosition();
      //currentPosR = m_encoderR.getPosition();

      //m_pidController.setReference((currentPos),com.revrobotics.CANSparkMax.ControlType.kPosition);
      //m_pidController.setReference((currentPosR),com.revrobotics.CANSparkMax.ControlType.kPosition);

    }
    if(isMoving){
      m_pidController.setReference((currentPos+distance*ROTATIONS_PER_INCH),com.revrobotics.CANSparkMax.ControlType.kPosition);
      m_pidControllerR.setReference((currentPosR+distance*ROTATIONS_PER_INCH),com.revrobotics.CANSparkMax.ControlType.kPosition);

    }
    SmartDashboard.putBoolean("isMoving", isMoving);
   */ 
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
