package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Utils.Constants;
import frc.robot.commands.MoveWholeArmToPositionCommand;
import frc.robot.commands.RunAlgaeManipulatorCommand;
import frc.robot.commands.RunCoralManipulatorCommand;
import frc.robot.lib.controls.XBoxController;


public class IO { 
    private static IO instance; 
    private XBoxController driverController;
    private XBoxController manipulatorController;
    private Trigger coralStationProximity;
    
    private IO() {
        driverController = new XBoxController(0);
        manipulatorController = new XBoxController(1);
        
        //coralStationProximity = new Trigger(SubsystemsInst.getInst().drivetrain.nearRedCoral());
    }
public static IO getInstance() {
    if(instance == null) instance = new IO();

    return instance;
}

public void teleopInit(){
    //Put Commands Here
    Command intakeCoral = new SequentialCommandGroup(
        new MoveWholeArmToPositionCommand(Constants.coralElevatorPosition, Constants.coralArmAngle),
        new RunCoralManipulatorCommand(Constants.coralIntakeSpeed)
    );

    Command intakeAlgaeL3 = new SequentialCommandGroup(
        new MoveWholeArmToPositionCommand(Constants.L3AlgaeElevatorPosition, Constants.reefAlgaeAngle),
        new RunAlgaeManipulatorCommand(Constants.algaeIntakeSpeed)
    );

    Command intakeAlgaeL2 = new SequentialCommandGroup(
        new MoveWholeArmToPositionCommand(Constants.L2AlgaeElevatorPosition, Constants.reefAlgaeAngle),
        new RunAlgaeManipulatorCommand(Constants.algaeIntakeSpeed)
    );

    manipulatorController.whenButtonPressed("Y", new MoveWholeArmToPositionCommand(Constants.L4ElevatorPosition, Constants.L4ArmAngle));
    manipulatorController.whenButtonPressed("X", new MoveWholeArmToPositionCommand(Constants.L3ElevatorPosition, Constants.L2L3ArmAngle));
    manipulatorController.whenButtonPressed("RBUMP", new MoveWholeArmToPositionCommand(Constants.L2ElevatorPosition, Constants.L2L3ArmAngle));
    manipulatorController.whenButtonPressed("B", intakeCoral);
    manipulatorController.whenPOVButtonPressed("W", intakeAlgaeL2);
    manipulatorController.whenPOVButtonPressed("E", intakeAlgaeL3);
    manipulatorController.whenPOVButtonPressed("N", new MoveWholeArmToPositionCommand(Constants.bargeElevatorPosition, Constants.bargeAlgaeAngle));
    manipulatorController.whenButtonPressed("LBUMP", new MoveWholeArmToPositionCommand(Constants.processorElevatorPosition, Constants.processorAlgaeAngle));
    manipulatorController.whenButtonPressed("LTRIG", new MoveWholeArmToPositionCommand(Constants.defaultElevatorPosition, Constants.defaultArmAngle));
    manipulatorController.whenButtonPressed("A", new RunCoralManipulatorCommand(Constants.coralScoringSpeed));
    manipulatorController.whenPOVButtonPressed("S", new RunAlgaeManipulatorCommand(Constants.algaeScoringSpeed));

    //TEST COMMANDS
    // manipulatorController.whenButtonPressed("A", new IntakeCoralCommand(3000));
    // manipulatorController.whenButtonPressed("B", new IntakeAlgaeCommand(1000));
    // manipulatorController.whenButtonPressed("X", new TestElevator());
    // manipulatorController.whenButtonPressed("Y", new TestRotaryArm());
    
}

public XBoxController getDriverController(){
    return driverController;
}
public XBoxController getManipulatorController(){
    return manipulatorController;

}
}