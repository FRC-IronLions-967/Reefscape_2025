// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utils.Constants;
import frc.robot.subsystems.ClimberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveClimberCommand extends Command {
  /** Creates a new MoveClimberCommand. */

  private ClimberSubsystem climberSubsystem;

  private double position;

  public MoveClimberCommand(double position) {
    climberSubsystem = new ClimberSubsystem();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climberSubsystem);

    this.position = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climberSubsystem.moveRatchet(Constants.climberRatchetOffPosition);
    if (!climberSubsystem.isRatchetOn()) {
      climberSubsystem.moveClimberArm(position);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climberSubsystem.moveRatchet(Constants.climberRatchetOnPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climberSubsystem.isClimberInPosition(position);
  }
}
