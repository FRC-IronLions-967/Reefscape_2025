// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//This command makes the elevator move to a specified position

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SubsystemsInst;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveWholeArmToPositionCommand extends Command {

  private ArmSubsystem armSubsystem;
  private double elevatorPosition;
  private double armPosition;

  /** Creates a new MoveElevatorToPositionCommand. */
  public MoveWholeArmToPositionCommand(double elevatorPosition, double armPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    armSubsystem = SubsystemsInst.getInst().armSubsystem;
    addRequirements(armSubsystem);
    this.elevatorPosition = elevatorPosition;
    this.armPosition = armPosition;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //We want to make sure that the arm doesn't rotate through reef branches.

    // If the elevator is going down, the arm will move then the elevator.
    if (armSubsystem.getElevatorPosition() > elevatorPosition) {
      armSubsystem.moveArm(armPosition);
      if (armSubsystem.isArmInPosition()) {
        armSubsystem.moveElevator(elevatorPosition);
      }
      //If the elevator is going up, the elevator will move then the arm.
    } else if (armSubsystem.getElevatorPosition() < elevatorPosition) {
      armSubsystem.moveElevator(elevatorPosition);
      if (armSubsystem.isElevatorInPosition()) {
        armSubsystem.moveArm(armPosition);
      }
      //If the elevator is not moving, then the arm can move;
    } else {
      armSubsystem.moveArm(armPosition);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armSubsystem.isInPosition();
  }
}
