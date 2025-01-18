package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Utils.Constants;
import frc.robot.commands.MoveWholeArmToPositionCommand;
import frc.robot.commands.IntakeAlgaeCommand;
import frc.robot.commands.IntakeCoralCommand;
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
    //Put Commands Here
    Command intakeCoral = new ParallelCommandGroup(
        new MoveWholeArmToPositionCommand(Constants.coralStationPosition),
        new IntakeCoralCommand(Constants.coralIntakeSpeed)
    );

    Command intakeAlgaeL3 = new ParallelCommandGroup(
        new MoveWholeArmToPositionCommand(Constants.L3AlgaePosition),
        new IntakeAlgaeCommand(Constants.algaeIntakeSpeed)
    );

    Command intakeAlgaeL2 = new ParallelCommandGroup(
        new MoveWholeArmToPositionCommand(Constants.L2AlgaePosition),
        new IntakeAlgaeCommand(Constants.algaeIntakeSpeed)
    );

    manipulatorController.whenButtonPressed("Y", new MoveWholeArmToPositionCommand(Constants.L4Position));
    manipulatorController.whenButtonPressed("X", new MoveWholeArmToPositionCommand(Constants.L3Position));
    manipulatorController.whenButtonPressed("RBUMP", new MoveWholeArmToPositionCommand(Constants.L2Position));
    manipulatorController.whenButtonPressed("B", intakeCoral);
    manipulatorController.whenPOVButtonPressed("W", intakeAlgaeL2);
    manipulatorController.whenPOVButtonPressed("E", intakeAlgaeL3);
    manipulatorController.whenPOVButtonPressed("N", new MoveWholeArmToPositionCommand(Constants.bargePosition));
    manipulatorController.whenButtonPressed("LBUMP", new MoveWholeArmToPositionCommand(Constants.processorPosition));
    manipulatorController.whenButtonPressed("LTRIG", new MoveWholeArmToPositionCommand(Constants.defaultPosition));
    manipulatorController.whenButtonPressed("A", new IntakeCoralCommand(Constants.coralScoringSpeed));
    manipulatorController.whenPOVButtonPressed("S", new IntakeAlgaeCommand(Constants.algaeScoringSpeed));
    
}
public XBoxController getDriverController(){
    return driverController;
}
public XBoxController getManipulatorController(){
    return manipulatorController;

}
}