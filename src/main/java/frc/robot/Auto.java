package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Auto {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private static final String kStraightAuto = "Straight auto";
  private static final String kBrokenArmAuto = "Broken Arm";
  private static final String idealauto = "place pice";
  private static final String placePiece = "place piece auto";
  private static final String placeAndLevel = "place and level";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private int autoStage = 0;

  private Drive drive;
  private PickerUpper pickerUpper;
  private final double SIDE_LENGTH = 11.75;
  private final double ARM_LENGTH = 46.5;
  private final double EXTENSON_LENGTH = 15;
  private final double ROBOT_ARM_HEIGHT = 50.5;
  private final double TARGET_HEIGHT = 23.5;
  private final double EXTRA_ANGLE = 5;
  private final double METERS_TO_INCHES = 39.37;
  private final double CAMERA_X = 4.25; // forward
  private final double CAMERA_Y = 4.25; // side to side
  private double x = 0;
  private double y = 0;
  private double z = 0;
  private final double pickupAngle = Math.toDegrees(Math.atan(58 / 10)) + 1;
  private final double pickupExtension = Math.sqrt(58 * 58 + 100);

  private double distancePlace = Math.sqrt((49.5 * 49.5) + (11.75 * 11.75));
  private double anglePlace = Math.toDegrees(Math.atan(11.75 / 49.5));
  private double heightPlace = /*Math.toDegrees(Math.atan(distancePlace/((ARM_HEIGHT - 46)+5)));*/
      85;

  private double targetX = 0;
  private double targetY = 0;
  private double targetZ = 0;
  private int placeStage = 0;
  private double armAngle = 0;
  private double turretAngle = 0;
  private int targetID = 0;
  private double extensonLength = 0;
  private int homePickerUpperStage = 0;
  private int autoPlaceStage = 0;
  private double time = 0;
  private int autoPickupStage = 0;
  private double autoPickupAngle = 0;
  private double pickupExtensionDistance = 0;

  public Auto(Drive drive, PickerUpper pickerUpper) {
    this.drive = drive;
    this.pickerUpper = pickerUpper;
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    m_chooser.addOption("Broken Arm", kBrokenArmAuto);
    m_chooser.addOption("place piece auto", placePiece);
    m_chooser.addOption("place pice", idealauto);
    m_chooser.addOption("straight auto", kStraightAuto);
    m_chooser.addOption("place and level", placeAndLevel);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  public void updateSelection() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  public void autoInit() {
    autoStage = 0;
    drive.setPos();
  }

  public void runSelected() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        if (autoStage == 0) {
          if (drive.driveTo(168)) {
            autoStage = 1;
          }
        } else if (autoStage == 1) {
          if (drive.turnTo(90)) {
            autoStage = 2;
          }
        } else if (autoStage == 2) {
          if (drive.driveTo(-54)) {
            autoStage = 3;
          }
        } else if (autoStage == 3) {
          if (drive.turnTo(0)) {
            autoStage = 4;
            drive.levelInit();
          }
        } else if (autoStage == 4) {
          if (drive.level()) {
            autoStage = 5;
            drive.setPos();
          }
        } else {
          drive.holdPosition();
        }

        break;
      case kBrokenArmAuto:
        if (autoStage == 0) {
          // Back up
          if (drive.driveTo(-24)) {
            autoStage = 1;
          }
        } else if (autoStage == 1) {
          // Forward
          if (drive.driveTo(30)) {
            autoStage = 2;
          }
        }
        if (autoStage == 2) {
          // Back up
          if (drive.driveTo(-24)) {
            autoStage = 3;
          }
        }
        // ;)
        else if (autoStage == 3) {
          // Turn 180 degrees
          if (drive.turnTo(180)) {
            autoStage = 4;
          }
        } else if (autoStage == 4) {
          // Forward until hit game piece
          if (drive.driveTo(144)) {
            autoStage = 5;
          }
        } else if (autoStage == 5) {
          // Turn 180 degrees in place
          if (drive.turnTo(180)) {
            autoStage = 6;
          }
        } else if (autoStage == 6) {
          // Move to node
          if (drive.driveTo(144)) {
            autoStage = 7;
          }
        } else if (autoStage == 7) {
          // Forward
          if (drive.driveTo(24)) {
            autoStage = 8;
          }
        } else if (autoStage == 8) {
          // Back up
          if (drive.driveTo(-24)) {
            autoStage = 9;
          }
        } else if (autoStage == 9) {
          // Turn "A bit"
          if (drive.turnTo(20)) {
            autoStage = 10;
          }
        } else if (autoStage == 10) {
          // Forward
          if (drive.driveTo(6)) {
            autoStage = 11;
          }
        }

        break;
      case kDefaultAuto:
        SmartDashboard.putNumber("auto stage", autoStage);

        if (autoStage == 0) {
          drive.driveOverInit();
          autoStage = 1;

        } else if (autoStage == 1) {
          if (drive.driveOverChargingStation()) {
            autoStage = 2;
            drive.levelInit();
          }
        } else if (autoStage == 2) {
          if (drive.level()) {
            autoStage = 3;
            drive.setPos();
          }
        } else if (autoStage == 3) {
          drive.holdPosition();
        }

        break;
      case kStraightAuto:
        break;
      default:
        // Put default auto code here
        break;

      case idealauto:
        SmartDashboard.putNumber("auto stage", autoStage);
        if (autoStage == 0) {
          if (
          /*drive.driveTo(0)*/ true) {
            autoStage = 1;
          }
          pickerUpper.grabber.holdAuto();
        } else if (autoStage == 1) {
          if (pickerUpper.arm.moveArmTo(80)) {
            autoStage = 2;
            time = Timer.getFPGATimestamp();
          }
          pickerUpper.grabber.holdAuto();

        } else if (autoStage == 2) {
          if (drive.driveTo(-14) || Math.abs(Timer.getFPGATimestamp() - time) > 2) {
            autoStage = 3;
          }
          pickerUpper.grabber.holdAuto();

        } else if (autoStage == 3) {
          placePieceAutoBySetpointInit();
          autoStage = 4;
          pickerUpper.grabber.holdAuto();

        } else if (autoStage == 4) {
          if (placePieceAutoBySetpoint()) {
            autoStage = 5;
            time = Timer.getFPGATimestamp();
          }
        } else if (autoStage == 5) {
          if (Math.abs(Timer.getFPGATimestamp() - time) < .4) {
            pickerUpper.grabber.open();
          } else {
            autoStage = 6;
            drive.setPos();
          }
        } else if (autoStage == 6) {
          if (pickerUpper.arm.moveArmTo(110)) {
            autoStage = 7;
          }
        } else if (autoStage == 7) {
          autoStage = 8;

        } else if (autoStage == 8) {
          if (drive.driveTo(180)) {
            drive.setPos();
            autoStage = 9;
          } else {
            if (pickerUpper.tower.moveTowerTo(0)) {
              autoStage = 8;
              time = Timer.getFPGATimestamp();
            } else {
              pickerUpper.extender.extendToPos(0);
            }
            pickerUpper.grabber.close();
          }
        } else if (autoStage == 9) {
          drive.holdPosition();
        }
        SmartDashboard.putNumber("autostage", autoStage);
        break;
      case placePiece:
        if (autoStage == 0) {
          placePieceAutoBySetpointInit();
          autoStage = 1;
        } else if (autoStage == 1) {
          if (placePieceAutoBySetpoint()) {
            autoStage = 2;
          }
        } else if (autoStage == 2) {
          pickerUpper.grabber.open();
          if (Math.abs(time - Timer.getFPGATimestamp()) > .05) {
            autoStage = 3;
          }
        } else if (autoStage == 3) {
          if (pickerUpper.arm.moveArmTo(50)) {
            autoStage = 4;
            drive.driveOverInit();
          }
        } else if (autoStage == 4) {
          if (drive.driveOverChargingStation()) {
            autoStage = 5;
          }
        } else if (autoStage == 5) {
          drive.holdPosition();
        }
      case placeAndLevel:
        if (autoStage == 0) {
          if (drive.driveTo(10)) {
            autoStage = 1;
          }
          pickerUpper.grabber.hold();
        } else if (autoStage == 1) {
          if (pickerUpper.arm.moveArmTo(80)) {
            autoStage = 2;
            time = Timer.getFPGATimestamp();
          }
          pickerUpper.grabber.hold();

        } else if (autoStage == 2) {
          if (drive.driveTo(-20) || Math.abs(Timer.getFPGATimestamp() - time) > 1.5) {
            autoStage = 3;
          }
          pickerUpper.grabber.hold();

        } else if (autoStage == 3) {
          placePieceAutoBySetpointInit();
          autoStage = 4;
          pickerUpper.grabber.hold();

        } else if (autoStage == 4) {
          if (placePieceAutoBySetpoint()) {
            autoStage = 5;
            time = Timer.getFPGATimestamp();
          }
          pickerUpper.grabber.hold();

        } else if (autoStage == 5) {
          if (Math.abs(Timer.getFPGATimestamp() - time) < .2) {
            pickerUpper.grabber.open();
          } else {
            autoStage = 6;
          }
        } else if (autoStage == 6) {
          if (pickerUpper.arm.moveArmTo(110)) {
            autoStage = 7;
          }
        } else if (autoStage == 7) {
          if (pickerUpper.tower.moveTowerTo(0)) {
            autoStage = 8;
            time = Timer.getFPGATimestamp();
            drive.levelInit();
          } else {
            pickerUpper.extender.extendToPos(0);
          }
        } else if (autoStage == 8) {
          if (drive.level()) {
            autoStage = 9;
          }
        } else if (autoStage == 9) {
          drive.holdPosition();
        }
    }
  }

  public void placePieceInit(int target) {
    placeStage = 0;

    if (target == 1 || target == 2 || target == 3) {
      targetX = 8.5;
    } else {
      targetX = 2;
    }

    if (target == 1 || target == 3) {
      targetZ = 10.75;
    } else if (target == 4 || target == 6) {
      targetZ = 22.25;
    } else if (target == 2) {
      targetZ = 0.25;
    } else {
      targetZ = 12.25;
    }

    if (target == 1 || target == 4) {
      targetY = -23.5;
    } else if (target == 2 || target == 5) {
      targetY = 0;
    } else {
      targetY = 23.5;
    }
    System.out.println("target x, y, z: " + targetX + ", " + targetY + ", " + targetZ);
  }

  public boolean placePiece(int target, boolean isFront, boolean isPositive) {
    // double z = 0;
    // Stage 0: read x, y, and z values for the robot.
    if (placeStage == 0) {
      if (drive.checkForTarget(0)) {
        x = drive.targetValues().getX() * METERS_TO_INCHES;
        y = drive.targetValues().getY() * METERS_TO_INCHES;
        z = drive.targetValues().getZ();
        System.out.println("stage 0: x: " + x + " y: " + y);
        // z = drive.targetValues().getZ() * METERS_TO_INCHES;
        targetID = drive.targetID();
        placeStage = 1;
      }
    }
    // Stage 1: Check if the robot is close enough
    else if (placeStage == 1) {
      if (isFront) {
        double xydistance = Math.sqrt(
            ((x + targetX + CAMERA_X) * (x + targetX + CAMERA_X))
                + ((targetY + y + CAMERA_Y) * (targetY + y + CAMERA_Y)));
        double distance = Math.sqrt(
            ((xydistance * xydistance)
                + ((ROBOT_ARM_HEIGHT - targetZ - TARGET_HEIGHT)
                    * (ROBOT_ARM_HEIGHT - targetZ - TARGET_HEIGHT))));
        System.out.println(
            "Stage 1: xydistance: "
                + xydistance
                + " distance: "
                + distance
                + " target x, y, z: "
                + targetX
                + ", "
                + targetY
                + ", "
                + targetZ);

        if ((distance > (ARM_LENGTH
            + EXTENSON_LENGTH) /*|| (Math.abs(targetY-y) > SIDE_LENGTH || distance < ARM_LENGTH)*/)) {
          System.out.println("Too far away " + distance);
          return true;
        } else if (distance < ARM_LENGTH) {
          System.out.println("Too close " + distance);
          return true;
        }
      } else {
        double xzDistance = Math.sqrt(
            ((x + targetX + CAMERA_X) * (x + targetX + CAMERA_X))
                + ((ROBOT_ARM_HEIGHT - targetZ) * (ROBOT_ARM_HEIGHT - targetZ)));
        if (xzDistance > (ARM_LENGTH + EXTENSON_LENGTH) || xzDistance < ARM_LENGTH) {
          System.out.println("Too far away");
          return true;
        }
      }
      placeStage = 2;
    }
    // Stage 2: Check if the robot is in front of the target
    else if (placeStage == 2) {
      if (isFront) {
        System.out.println("stage 2");
        placeStage = 3;
      } else {
        if (Math.abs(targetY + y + CAMERA_Y) < SIDE_LENGTH) {
          placeStage = 3;
        } else {
          if (isPositive) {
            drive.driveTo(targetY - y);
          } else {
            drive.driveTo(-(targetY - y));
          }
        }
      }
    }
    // Stage 3: calculate the arm angle
    else if (placeStage == 3) {
      armAngle = Math.atan((targetX + x + CAMERA_X) / (ROBOT_ARM_HEIGHT - (TARGET_HEIGHT + targetZ)));
      armAngle = Math.toDegrees(armAngle);
      armAngle = armAngle + EXTRA_ANGLE;
      placeStage = 4;
      System.out.println("stage 3: arm angle: " + armAngle);
    }
    // Stage 4: move the arm
    else if (placeStage == 4) {
      System.out.println("stage 4");
      if (pickerUpper.arm.moveArmTo(armAngle)) {
        placeStage = 5;
      }
    }
    // Stage 5: check arm again
    else if (placeStage == 5) {
      if (drive.checkForTarget(targetID)) {
        x = drive.targetValues().getX() * METERS_TO_INCHES;
        y = drive.targetValues().getY() * METERS_TO_INCHES;
        z = drive.targetValues().getZ();
        System.out.println("Stage 5: x: " + x + " y: " + y);
        placeStage = 6;
      } else {
        if (!isFront) {
          y = targetY;
          placeStage = 6;
        }
      }
    }
    // Stage 6: calculate turret angle
    else if (placeStage == 6) {
      // double angle = drive.returnAngle();
      turretAngle = Math.atan((y + targetY + CAMERA_Y) / (x + targetX + CAMERA_X));
      turretAngle = Math.toDegrees(turretAngle + z);
      placeStage = 7;
      System.out.println("stage 6: turretAngle: " + turretAngle);
    }
    // Stage 7: set turret position
    else if (placeStage == 7) {
      System.out.println("stage 7");
      if (pickerUpper.tower.moveTowerTo(turretAngle)) {
        placeStage = 8;
      }
    }
    // Stage 8: calculate extenton distance
    else if (placeStage == 8) {
      double xydistance = Math.sqrt(
          ((x + targetX + CAMERA_X) * (x + targetX + CAMERA_X))
              + ((targetY + y + CAMERA_Y) * (targetY + y + CAMERA_Y)));
      double distance = Math.sqrt(
          ((xydistance * xydistance)
              + ((ROBOT_ARM_HEIGHT - targetZ - TARGET_HEIGHT)
                  * (ROBOT_ARM_HEIGHT - targetZ - TARGET_HEIGHT))));
      /*if(distance>(ARM_LENGTH-2) || distance <(ARM_LENGTH+2)){
          placeStage = 10;
      }
      else{*/
      extensonLength = distance - ARM_LENGTH;
      placeStage = 9;
      System.out.println(
          "stage 8: xydistance: "
              + xydistance
              + " distance: "
              + distance
              + " extenson length"
              + extensonLength);
      // }

    }
    // Stage 9: extend
    else if (placeStage == 9) {
      System.out.println("stage 9");
      if (pickerUpper.extender.extendToPos(extensonLength)) {
        placeStage = 10;
      }
    }
    // Stage 10: done
    else if (placeStage == 10) {
      System.out.println("In Position");
      return true;
    }

    return false;
  }

  public void homePickerUpperInit() {
    homePickerUpperStage = 0;
  }

  public boolean homePickerUpper() {
    if (homePickerUpperStage == 0) {
      // moveArmTo needs to be set (50) is just a refrence
      if (pickerUpper.arm.moveArmTo(50)) {
        homePickerUpperStage = 1;
        System.out.print("move arm to position 50");
      }
    } else if (homePickerUpperStage == 1) {
      if (pickerUpper.extender.extendToPos(0)) {
        homePickerUpperStage = 2;
        System.out.print("extend to pos 0");
      }
    } else if (homePickerUpperStage == 2) {
      if (pickerUpper.tower.moveTowerTo(0)) {
        homePickerUpperStage = 3;
        System.out.print("move tower to angle 0");
      }
    } else if (homePickerUpperStage == 3) {
      if (pickerUpper.arm.moveArmTo(20)) {
        homePickerUpperStage = 4;
        System.out.print("move arm to position 20");
      }
    } else if (homePickerUpperStage == 4) {
      return true;
    }

    return false;
  }

  public void placePieceAutoBySetpointInit() {
    autoPlaceStage = 0;
  }

  public boolean placePieceAutoBySetpoint() {
    if (autoPlaceStage == 0) {
      System.out.println("stage 0" + heightPlace);
      if (pickerUpper.arm.moveArmTo(heightPlace)) {
        autoPlaceStage = 1;
      }
    } else if (autoPlaceStage == 1) {
      System.out.println("stage 1" + anglePlace);

      if (pickerUpper.tower.moveTowerTo(anglePlace)) {
        autoPlaceStage = 2;
      }
    } else if (autoPlaceStage == 2) {
      System.out.println("stage 2" + (distancePlace - ARM_LENGTH));

      if (pickerUpper.extender.extendToPos(distancePlace - ARM_LENGTH)) {
        autoPlaceStage = 3;
      }
    } else if (autoPlaceStage == 3) {
      return true;
    }
    SmartDashboard.putNumber("auto place stage", autoPlaceStage);
    return false;
  }

  public void pickupPieceAutoBySetpointInit() {
    autoPlaceStage = 0;
  }

  public boolean pickupPieceAutoBySetpoint() {
    if (autoPlaceStage == 0) {
      if (pickerUpper.arm.moveArmTo(pickupAngle)) {
        autoPlaceStage = 1;
      }
    } else if (autoPlaceStage == 1) {
      if (pickerUpper.extender.extendToPos(pickupExtension - ARM_LENGTH)) {
        autoPlaceStage = 2;
      }
    } else if (autoPlaceStage == 2) {
      return true;
    }
    return false;
  }

  public void autoPlaceAndDrive() {
    if (autoStage == 0) {
      placePieceInit(4);
      autoStage = 1;
    } else if (autoStage == 1) {
      if (placePiece(4, true, true)) {
        autoStage = 2;
        time = Timer.getFPGATimestamp();
      }
    } else if (autoStage == 2) {
      if (Math.abs(time - Timer.getFPGATimestamp()) > .25) {
        autoStage = 3;
        drive.driveOverInit();
      }
    } else if (autoStage == 3) {
      drive.arcade(.5, 0);
      if (drive.driveOverChargingStation()) {
        autoStage = 4;
        drive.levelInit();
      }

    } else if (autoStage == 4) {
      if (drive.level()) {
        autoStage = 5;
      }
    } else if (autoStage == 5) {
      drive.holdPosition();
    }
  }

  public void autoPickupInit() {
    autoPickupStage = 0;
  }

  public boolean autoPickup() {
    if (autoPickupStage == 0) {
      autoPickupAngle = pickerUpper.tower.rotateToCubeInit();
      autoStage = 1;
    } else if (autoPickupStage == 1) {
      if (pickerUpper.tower.rotateToCube()) {
        autoPickupStage = 2;
      }
    } else if (autoPickupStage == 2) {
      if (drive.checkForTarget(targetID)) {
        x = CAMERA_X + drive.targetValues().getX() * METERS_TO_INCHES;
      } else {
        System.out.println("No Target");
        return true;
      }
      pickupExtensionDistance = x / Math.cos(autoPickupAngle) - ARM_LENGTH;
      if (pickupExtensionDistance < 0 || pickupExtensionDistance > 14) {
        System.out.println("Wrong Distance");
        return true;
      }

      autoPickupStage = 3;
    } else if (autoPickupStage == 3) {
      if (pickerUpper.extender.extendToPos(pickupExtensionDistance)) {
        autoPlaceStage = 4;
      }
    }
    if (autoPickupStage == 4) {
      return true;
    }
    return false;
  }
}
