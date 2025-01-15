// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

  private SparkFlex elevatorVortex;
  private SparkClosedLoopController elevatorVortexController;
  private SparkFlexConfig elevatorVortexConfig;

  private SparkMax rotationMax;
  private SparkClosedLoopController rotationMaxController;
  private SparkMaxConfig rotationMaxConfig;

  private SparkFlex coralManipulatorVortex;
  private SparkClosedLoopController coralManipulatorVortexController;
  private SparkFlexConfig coralManipulatorVortexConfig;

  private SparkFlex algaeManipulatorVortex;
  private SparkClosedLoopController algaeManipulatorVortexController;
  private SparkFlexConfig algaeManipulatorVortexConfig;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {

    elevatorVortex = new SparkFlex(9, MotorType.kBrushless);
    elevatorVortexController = elevatorVortex.getClosedLoopController();
    elevatorVortexConfig = new SparkFlexConfig();

    rotationMax = new SparkMax(10, MotorType.kBrushless);
    rotationMaxController = rotationMax.getClosedLoopController();
    rotationMaxConfig = new SparkMaxConfig();

    coralManipulatorVortex = new SparkFlex(11, MotorType.kBrushless);
    coralManipulatorVortexController = coralManipulatorVortex.getClosedLoopController();
    coralManipulatorVortexConfig = new SparkFlexConfig();

    algaeManipulatorVortex = new SparkFlex(12, MotorType.kBrushless);
    algaeManipulatorVortexController = algaeManipulatorVortex.getClosedLoopController();
    algaeManipulatorVortexConfig = new SparkFlexConfig();


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
