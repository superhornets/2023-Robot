package frc.robot;

public class Auto {
    private Drive drive;
    private PickerUpper pickerUpper;
    private final double FRONT_LENGTH = 0;
    private final double SIDE_LENGTH = 0;
    private final double ARM_LENGTH = 0;
    private final double EXTENSON_LENGTH = 0;
    private final double FRONT_ANGLE_RANGE = 0;
    private final double SIDE_ANGLE_RANGE = 0;
    private final double ROBOT_ARM_HEIGHT = 0;


    private double targetX = 0;
    private double targetY = 0;
    private double targetZ = 0;
    private int placeStage = 0;

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
        if(placeStage == 0){
            if(drive.checkForTarget(0)){
                x = drive.targetValues().getX();
                y = drive.targetValues().getY();
                z = drive.targetValues().getZ();
                placeStage = 1;
            }
    }
        else if(placeStage == 1){
            if(isFront){
                double xydistance = Math.sqrt(((x+targetX)*(x+targetX))+((targetY-y)*(targetY-y)));
                double distance = Math.sqrt(((xydistance*xydistance)+((ROBOT_ARM_HEIGHT-targetZ)*(ROBOT_ARM_HEIGHT-targetZ))));
                if(distance > (ARM_LENGTH+EXTENSON_LENGTH)){
                    System.out.println("Too far away");
                    return true;
                }
            }
            else{
                double xzDistance = Math.sqrt(((x+targetX)*(x+targetX))+((ROBOT_ARM_HEIGHT-targetZ)*(ROBOT_ARM_HEIGHT-targetZ)));
                if(xzDistance > (ARM_LENGTH+EXTENSON_LENGTH)){
                    System.out.println("Too far away");
                    return true;
                }
            }
            placeStage = 2;
        }
        else if(placeStage == 2){
            if(isFront){
                placeStage = 3;
            }
            else{
                if(Math.abs(targetY-y) < SIDE_LENGTH){
                    placeStage = 3;
                }
                else if((targetY-y) > 0){
                    if(isPositive){
                        drive.arcade(-.2, 0);
                    }
                    else{
                        drive.arcade(.2, 0);
                    }
                }
                else{
                    if(isPositive){
                        drive.arcade(.2, 0);
                    }
                    else{
                        drive.arcade(-.2, 0);
                    }
                }
            }
        }
        else if(placeStage == 3){
            double armAngle = Math.atan(target)
        }
        

        



        return false;
    }
}
