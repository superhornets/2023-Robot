package frc.robot;

public class PickerUpper {
    

    public final Grabber grabber = new Grabber();

    public final Tower tower = new Tower();

    public final Arm arm = new Arm();

    public void SmartDashboardPrintout(){
        tower.SmartDashboardPrintout();
    }
 
    public PickerUpper(){
        
    }

    
}
