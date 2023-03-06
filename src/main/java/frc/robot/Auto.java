package frc.robot;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Auto {
    private static final String kDefaultAuto = "Default";
    private static final String kCustomAuto = "My Auto";
    private static final String kStraightAuto = "Straight auto";
    private static final String kBrokenArmAuto = "Broken Arm";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    private int autoStage = 0;

    private Drive drive;
    private PickerUpper pickerUpper;
    private final double FRONT_LENGTH = 22.25;
    private final double SIDE_LENGTH = 11.75;
    private final double ARM_LENGTH = 46.5;
    private final double EXTENSON_LENGTH = 15;
    private final double FRONT_ANGLE_RANGE = 0;
    private final double SIDE_ANGLE_RANGE = 0;
    private final double ROBOT_ARM_HEIGHT = 50.5;
    private final double TARGET_HEIGHT = 23.5;
    private final double ARM_HEIGHT = 0;
    private final double EXTRA_ANGLE = 5;
    private final double METERS_TO_INCHES = 39.37;
    private final double CAMERA_X = 4.25; // forward
    private final double CAMERA_Y = 4.25; // side to side
    private final double CAMERA_Z = 22; // height
    private double x = 0;
    private double y = 0;
    private final double z = 0;
    private final double pickupAngle =90 - Math.toDegrees(Math.atan(58/10));
    private final double pickupExtension = Math.sqrt(58*58+100);

    private double distancePlace = Math.sqrt((49.5*49.5)+(11.75*11.75));
    private double anglePlace = Math.toDegrees(Math.atan(11.75/49.5));
    private double heightPlace = /*Math.toDegrees(Math.atan(distancePlace/((ARM_HEIGHT - 46)+5)));*/ 85;


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

    public Auto(Drive drive, PickerUpper pickerUpper){
        this.drive = drive;
        this.pickerUpper = pickerUpper;

        m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
        m_chooser.addOption("My Auto", kCustomAuto);
        m_chooser.addOption("Broken Arm", kBrokenArmAuto);
        SmartDashboard.putData("Auto choices", m_chooser);
    }

    public void updateSelection() {
        m_autoSelected = m_chooser.getSelected();
        // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
        System.out.println("Auto selected: " + m_autoSelected);
        autoStage = 0;
    }

    public void runSelected() {
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

                break;
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
      
                break;
            case kStraightAuto:

                break;
            default:
              // Put default auto code here
              break;
        }
    }

    public void placePieceInit(int target){
        placeStage = 0;

        if(target == 1 || target == 2 || target == 3){
            targetX = 6.75;
        }
        else{
            targetX = 23.75;
        }

        if(target == 1 || target == 3){
            targetZ = 15.75;
        }
        else if  (target == 4 || target == 6){
            targetZ = 27.25;
        }
        else if (target == 2){
            targetZ = 5.25;
        }
        else{
            targetZ=17.25;
        }

        if (target == 1 || target == 4){
            targetY=-23.5;
        }
        else if(target == 2 || target == 5){
            targetY = 0;
        }
        else{
            targetY = 23.5;
        }
    }
    public boolean placePiece(int target, boolean isFront, boolean isPositive){
            //double z = 0;
            //Stage 0: read x, y, and z values for the robot.
        if(placeStage == 0){
            if(drive.checkForTarget(0)){
                x = drive.targetValues().getX() * METERS_TO_INCHES;
                y = drive.targetValues().getY() * METERS_TO_INCHES;
                //z = drive.targetValues().getZ() * METERS_TO_INCHES;
                targetID = drive.targetID();
                placeStage = 1;
            }
        }
        //Stage 1: Check if the robot is close enough
        else if(placeStage == 1){
            if(isFront){
                double xydistance = Math.sqrt(((x+targetX+CAMERA_X)*(x+targetX+CAMERA_X))+((targetY+y+CAMERA_Y)*(targetY+y+CAMERA_Y)));
                double distance = Math.sqrt(((xydistance*xydistance)+((ROBOT_ARM_HEIGHT-targetZ-TARGET_HEIGHT)*(ROBOT_ARM_HEIGHT-targetZ-TARGET_HEIGHT))));
                if((distance > (ARM_LENGTH+EXTENSON_LENGTH) /*|| (Math.abs(targetY-y) > SIDE_LENGTH || distance < ARM_LENGTH)*/)){
                    System.out.println("Too far away " + distance);
                    return true;
                }
                else if(distance < ARM_LENGTH){
                    System.out.println("Too close " + distance);
                    return true;
                }
            }
            else{
                double xzDistance = Math.sqrt(((x+targetX+CAMERA_X)*(x+targetX+CAMERA_X))+((ROBOT_ARM_HEIGHT-targetZ)*(ROBOT_ARM_HEIGHT-targetZ)));
                if(xzDistance > (ARM_LENGTH+EXTENSON_LENGTH) || xzDistance < ARM_LENGTH){
                    System.out.println("Too far away");
                    return true;
                }

            }
            placeStage = 2;
        }
        //Stage 2: Check if the robot is in front of the target
        else if(placeStage == 2){
            if(isFront){
                placeStage = 3;
            }
            else{
                if(Math.abs(targetY+y+CAMERA_Y) < SIDE_LENGTH){
                    placeStage = 3;
                }
                else{
                    if(isPositive){
                        drive.driveTo(targetY-y);
                    }
                    else{
                        drive.driveTo(-(targetY-y));
                    }
                }
            }
        }
        //Stage 3: calculate the arm angle
        else if(placeStage == 3){
            armAngle = Math.atan((targetX+x+CAMERA_X)/(ROBOT_ARM_HEIGHT-(TARGET_HEIGHT+targetZ)));
            armAngle = Math.toDegrees(armAngle);
            armAngle = 90-armAngle+EXTRA_ANGLE;
            placeStage = 4;
        }
        //Stage 4: move the arm
        else if(placeStage == 4){
            if(pickerUpper.arm.moveArmTo(armAngle)){
                placeStage = 5;
            }
        }
        //Stage 5: check arm again
        else if(placeStage == 5){
            if(drive.checkForTarget(targetID)){
                x = drive.targetValues().getX()*METERS_TO_INCHES;
                y = drive.targetValues().getY()*METERS_TO_INCHES;
                //z = drive.targetValues().getZ();
                placeStage = 6;
            }
            else{
                if(!isFront){
                    y=targetY;
                    placeStage =6;
                }
            }
        }
        //Stage 6: calculate turret angle
        else if(placeStage == 6){
            turretAngle = Math.atan((y+targetY+CAMERA_Y)/(x+targetX+CAMERA_X));
            turretAngle = Math.toDegrees(turretAngle);
            placeStage = 7;
        }
        //Stage 7: set turret position
        else if(placeStage == 7){
            if(pickerUpper.tower.moveTowerTo(armAngle)){
                placeStage = 8;
            }
        }
        //Stage 8: calculate extenton distance
        else if(placeStage == 8){
            double xydistance = Math.sqrt(((x+targetX+CAMERA_X)*(x+targetX+CAMERA_X))+((targetY+y+CAMERA_Y)*(targetY+y+CAMERA_Y)));
            double distance = Math.sqrt(((xydistance*xydistance)+((ROBOT_ARM_HEIGHT-targetZ)*(ROBOT_ARM_HEIGHT-targetZ))));
            /*if(distance>(ARM_LENGTH-2) || distance <(ARM_LENGTH+2)){
                placeStage = 10;
            }
            else{*/
                extensonLength = distance - ARM_LENGTH;
                placeStage = 9;
            //}

        }
        //Stage 9: extend
        else if(placeStage == 9){
            if(pickerUpper.grabber.extendToPos(extensonLength)){
                placeStage = 10;
            }
        }
        //Stage 10: done
        else if(placeStage == 10){
            System.out.println("In Position");
            return true;
        }
        

        

        return false;
    }



    public boolean homePickerUpper(){
        if (homePickerUpperStage == 0){
            //moveArmTo needs to be set (50) is just a refrence
            if (pickerUpper.arm.moveArmTo(50)){
                homePickerUpperStage = 1;
            }
        }
        else if (homePickerUpperStage == 1){
            if (pickerUpper.grabber.extendToPos(0)){
                homePickerUpperStage = 2;
            }
        }
        else if (homePickerUpperStage == 2){
            if (pickerUpper.tower.moveTowerTo(0)){
                homePickerUpperStage = 3;
            }
        }
        else if (homePickerUpperStage == 3){
            if (pickerUpper.arm.moveArmTo(20)){
                homePickerUpperStage = 4;
            }
        }
        else if (homePickerUpperStage == 4){
            return true;
        }

        return false;
    }
    public void placePieceAutoBySetpointInit(){
        autoPlaceStage = 0;

    }
    public boolean placePieceAutoBySetpoint(){
        if(autoPlaceStage == 0){
            System.out.println("stage 0" + heightPlace);
            if(pickerUpper.arm.moveArmTo(heightPlace)){
                autoPlaceStage = 1;
            }
        }
        else if(autoPlaceStage == 1){
            System.out.println("stage 1" + anglePlace);

            if(pickerUpper.tower.moveTowerTo(anglePlace)){
                autoPlaceStage = 2;
            }
        }
        else if(autoPlaceStage ==2){
            System.out.println("stage 2" + (distancePlace - ARM_LENGTH));

            if(pickerUpper.grabber.extendToPos(distancePlace - ARM_LENGTH)){
                autoPlaceStage = 3;
            }
        }
        else if(autoPlaceStage == 3){
            return true;
        }
        return false;
    }

    public void pickupPieceAutoBySetpointInit(){
        autoPlaceStage = 0;

    }
    public boolean pickupPieceAutoBySetpoint(){
        if(autoPlaceStage == 0){
            if(pickerUpper.arm.moveArmTo(pickupAngle)){
                autoPlaceStage = 1;
            }
        }
        else if(autoPlaceStage ==1){
            if(pickerUpper.grabber.extendToPos(pickupExtension - ARM_LENGTH)){
                autoPlaceStage = 2;
            }
        }
        else if(autoPlaceStage == 2){
            return true;
        }
        return false;
    }
}
