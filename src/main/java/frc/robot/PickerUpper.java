package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PickerUpper extends SubsystemBase{
    

    public final Grabber grabber = new Grabber();

    public final Tower tower = new Tower();

    public final Arm arm = new Arm();

    public void SmartDashboardPrintout(){
        tower.SmartDashboardPrintout();
        grabber.grabberSmartDashboard();
        arm.SmartDashboard();
    }
 
    public PickerUpper(){
        arm.setPickerUpper(tower, grabber);
        tower.setPickerUpper(arm, grabber);
    }

    @Override
    public void periodic() {

        tower.setP(tower.fjasdkl.getDouble(0));
    }
}
