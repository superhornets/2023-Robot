package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PickerUpper extends SubsystemBase {

  public final Grabber grabber = new Grabber();

  public final Tower tower = new Tower();

  public final Arm arm = new Arm();

  public final Extender extender = new Extender();

  public PickerUpper() {
    arm.setPickerUpper(tower, grabber, extender);
    tower.setPickerUpper(arm, grabber);
    extender.setPickerUpper(arm);
  }

  public void periodic() {
    tower.SmartDashboardPrintout();
    grabber.grabberSmartDashboard();
    arm.periodic();
  }
}
