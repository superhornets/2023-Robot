package frc.robot;


public class Auto {
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


    private double targetX = 0;
    private double targetY = 0;
    private double targetZ = 0;
    private int placeStage = 0;
    private double armAngle = 0;
    private double turretAngle = 0;
    private int targetID = 0;
    private double extensonLength = 0;

    public Auto(Drive drive, PickerUpper pickerUpper){
        this.drive = drive;
        this.pickerUpper = pickerUpper;
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
            double x = 0;
            double y = 0;
            double z = 0;
            //Stage 0: read x, y, and z values for the robot.
        if(placeStage == 0){
            if(drive.checkForTarget(0)){
                x = drive.targetValues().getX() * METERS_TO_INCHES;
                y = drive.targetValues().getY() * METERS_TO_INCHES;
                z = drive.targetValues().getZ() * METERS_TO_INCHES;
                targetID = drive.targetID();
                placeStage = 1;
            }
        }
        //Stage 1: Check if the robot is close enough
        else if(placeStage == 1){
            if(isFront){
                double xydistance = Math.sqrt(((x+targetX)*(x+targetX))+((targetY-y)*(targetY-y)));
                double distance = Math.sqrt(((xydistance*xydistance)+((ROBOT_ARM_HEIGHT-targetZ)*(ROBOT_ARM_HEIGHT-targetZ))));
                if((distance > (ARM_LENGTH+EXTENSON_LENGTH) || (Math.abs(targetY-y) > SIDE_LENGTH || distance < ARM_LENGTH))){
                    System.out.println("Too far away");
                    return true;
                }
            }
            else{
                double xzDistance = Math.sqrt(((x+targetX)*(x+targetX))+((ROBOT_ARM_HEIGHT-targetZ)*(ROBOT_ARM_HEIGHT-targetZ)));
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
                if(Math.abs(targetY-y) < SIDE_LENGTH){
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
            armAngle = Math.atan((targetX+x)/(ROBOT_ARM_HEIGHT-(TARGET_HEIGHT+targetY)));
            armAngle = Math.toDegrees(armAngle);
            armAngle += EXTRA_ANGLE;
            placeStage = 4;
        }
        //Stage 4: move the arm
        else if(placeStage == 4){
            if(pickerUpper.tower.setArm(armAngle)){
                placeStage = 5;
            }
        }
        //Stage 5: check arm again
        else if(placeStage == 5){
            if(drive.checkForTarget(targetID)){
                x = drive.targetValues().getX();
                y = drive.targetValues().getY();
                z = drive.targetValues().getZ();
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
            turretAngle = Math.atan((y-targetY)/(x+targetX));
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
            double xydistance = Math.sqrt(((x+targetX)*(x+targetX))+((targetY-y)*(targetY-y)));
            double distance = Math.sqrt(((xydistance*xydistance)+((ROBOT_ARM_HEIGHT-targetZ)*(ROBOT_ARM_HEIGHT-targetZ))));
            if(distance>(ARM_LENGTH-2) || distance <(ARM_LENGTH+2)){
                placeStage = 10;
            }
            else{
                extensonLength = distance - ARM_LENGTH;
                placeStage = 9;
            }

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
}
