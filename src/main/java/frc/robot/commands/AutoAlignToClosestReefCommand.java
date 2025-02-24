// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utils.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SubsystemsInst;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignToClosestReefCommand extends Command {
  /** Creates a new AutoAlignCommand. */

  private Drivetrain drivetrain;
  private Vision vision;
  private boolean leftSide;
  private PathPlannerPath pathToReef;
  private List<Waypoint> pathWaypoints;

  public AutoAlignToClosestReefCommand(boolean leftSide) {

    drivetrain = SubsystemsInst.getInst().drivetrain;
    vision = SubsystemsInst.getInst().vision;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    addRequirements(vision);

    this.leftSide = leftSide;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pathWaypoints = PathPlannerPath.waypointsFromPoses(drivetrain.getPose(), vision.figureOutClosestBranch(drivetrain.getPose(), leftSide ? Constants.leftPose2ds : Constants.rightPose2ds));
    pathToReef = new PathPlannerPath(pathWaypoints, Constants.pathConstraints, null, new GoalEndState(0, null));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    AutoBuilder.followPath(pathToReef);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
