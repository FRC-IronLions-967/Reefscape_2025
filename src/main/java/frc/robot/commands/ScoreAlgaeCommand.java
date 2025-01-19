// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SubsystemsInst;

//THis command will run the Algae manipulator

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreAlgaeCommand extends Command {
  /** Creates a new RunAlgaeManipulatorCommand. */
  private ArmSubsystem armSubsystem;
  private double speed;

  public ScoreAlgaeCommand(double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    armSubsystem = SubsystemsInst.getInst().armSubsystem;
    addRequirements(armSubsystem);
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.runAlgaeManipulator(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.runAlgaeManipulator(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !armSubsystem.hasAlgae();
  }
}
