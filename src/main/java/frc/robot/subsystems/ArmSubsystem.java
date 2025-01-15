// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Constants;

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

    elevatorVortexConfig
      .idleMode(IdleMode.kBrake);
    elevatorVortexConfig.encoder
      .positionConversionFactor(Constants.elevatorGearRatio);
    elevatorVortexConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(1, 0, 0);

    elevatorVortex.configure(elevatorVortexConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);


    rotationMaxConfig
      .idleMode(IdleMode.kBrake);
    rotationMaxConfig.encoder
      .positionConversionFactor(Constants.armGearRadius * Math.PI * 2);
    rotationMaxConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(1, 0, 0);

    rotationMax.configure(rotationMaxConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);


    coralManipulatorVortexConfig
      .idleMode(IdleMode.kCoast);
    coralManipulatorVortexConfig.encoder
      .velocityConversionFactor(Constants.coralWheelRadius * Math.PI * 2);
    coralManipulatorVortexConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(1, 0, 0);

    coralManipulatorVortex.configure(coralManipulatorVortexConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);


    algaeManipulatorVortexConfig
      .idleMode(IdleMode.kCoast);
    algaeManipulatorVortexConfig.encoder
      .velocityConversionFactor(Constants.algaeWheelRadius * Math.PI * 2);
    algaeManipulatorVortexConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(1, 0, 0);

    algaeManipulatorVortex.configure(algaeManipulatorVortexConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
