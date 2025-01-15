package frc.robot;

import frc.robot.Utils.Constants;
import frc.robot.commands.MoveWholeArmToPositionCommand;
import frc.robot.lib.controls.XBoxController;


public class IO { 
    private static IO instance; 
    private XBoxController driverController;
    private XBoxController manipulatorController;
    
    private IO() {
        driverController = new XBoxController(0);
        manipulatorController = new XBoxController(1);
        
    }
public static IO getInstance() {
    if(instance == null) instance = new IO();

    return instance;
}

public void teleopInit(){

    manipulatorController.whenButtonPressed("Y", new MoveWholeArmToPositionCommand(Constants.L4Position));
    manipulatorController.whenButtonPressed("X", new MoveWholeArmToPositionCommand(Constants.L3Position));
    manipulatorController.whenButtonPressed("RBUMP", new MoveWholeArmToPositionCommand(Constants.L2Position));
    manipulatorController.whenButtonPressed("B", new MoveWholeArmToPositionCommand(Constants.coralStationPosition));
    manipulatorController.whenPOVButtonPressed("W", new MoveWholeArmToPositionCommand(Constants.L2AlgaePosition));
    manipulatorController.whenPOVButtonPressed("E", new MoveWholeArmToPositionCommand(Constants.L3AlgaePosition));
    manipulatorController.whenPOVButtonPressed("N", new MoveWholeArmToPositionCommand(Constants.bargePosition));
    manipulatorController.whenButtonPressed("LBUMP", new MoveWholeArmToPositionCommand(Constants.processorPosition));
    manipulatorController.whenButtonPressed("LTRIG", new MoveWholeArmToPositionCommand(Constants.defaultPosition));
    
}
public XBoxController getDriverController(){
    return driverController;
}
public XBoxController getManipulatorController(){
    return manipulatorController;

}
}