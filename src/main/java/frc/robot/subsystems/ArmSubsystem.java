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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Constants;

public class ArmSubsystem extends SubsystemBase {

  private SparkFlex elevatorVortex;
  private SparkClosedLoopController elevatorVortexController;
  private SparkFlexConfig elevatorVortexConfig;

  private SparkFlex armVortex;
  private SparkClosedLoopController armVortexController;
  private SparkFlexConfig armVortexConfig;

  private SparkFlex coralManipulatorVortex;
  private SparkClosedLoopController coralManipulatorVortexController;
  private SparkFlexConfig coralManipulatorVortexConfig;

  private SparkFlex algaeManipulatorVortex;
  private SparkClosedLoopController algaeManipulatorVortexController;
  private SparkFlexConfig algaeManipulatorVortexConfig;

  private DigitalInput coralLimitSwitch;
  private DigitalInput algaeLimitSwitch;

  private ArmStates state;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {

    state = ArmStates.DEFAULT;

    elevatorVortex = new SparkFlex(9, MotorType.kBrushless);
    elevatorVortexController = elevatorVortex.getClosedLoopController();
    elevatorVortexConfig = new SparkFlexConfig();

    armVortex = new SparkFlex(10, MotorType.kBrushless);
    armVortexController = armVortex.getClosedLoopController();
    armVortexConfig = new SparkFlexConfig();

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


    armVortexConfig
      .idleMode(IdleMode.kBrake);
    armVortexConfig.encoder
      .positionConversionFactor(Constants.armGearRadius * Math.PI * 2);
    armVortexConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(1, 0, 0).positionWrappingInputRange(0, 2*Math.PI)
      .positionWrappingEnabled(true);

    armVortex.configure(armVortexConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);


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


    coralLimitSwitch = new DigitalInput(0);
    algaeLimitSwitch = new DigitalInput(1);
    
  }

  public void moveElevator(double position) {
    elevatorVortexController.setReference(position, ControlType.kPosition);
  }

  public double getElevatorPosition() {
    return elevatorVortex.getEncoder().getPosition();
  }

  public void moveArm(double Angle) {
    armVortexController.setReference(Angle, ControlType.kPosition);
  }

  public double getArmAngle() {
    return armVortex.getEncoder().getPosition();
  }

  public void runCoralManipulator(double speed) {
    coralManipulatorVortexController.setReference(speed, ControlType.kVelocity);
  }

  public boolean hasCoral() {
    return coralLimitSwitch.get();
  }

  public void runAlgaeManipulator(double speed) {
    algaeManipulatorVortexController.setReference(speed, ControlType.kVelocity);
  }

  public boolean hasAlgae() {
    return algaeLimitSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run  
    switch (state) {
      case DEFAULT:
        if (hasAlgae()) {
          if (hasCoral()) {
            state = ArmStates.BOTH_IN;
          } else {
            state = ArmStates.ALGAE_IN;
          }
        } else if (hasCoral()) {
          state = ArmStates.CORAL_IN;
        }
        break;

      case ALGAE_IN:
        if (!hasAlgae()) {
          state = ArmStates.DEFAULT;
        } else if (hasCoral()) {
          state = ArmStates.BOTH_IN;
        }
        break;

      case CORAL_IN:
        if(!hasCoral()) {
          state = ArmStates.DEFAULT;
        } else if (hasAlgae()) {
          state = ArmStates.BOTH_IN;
        }
        break;

      case BOTH_IN:
        if(!hasAlgae()) {
          state = ArmStates.CORAL_IN;
        } else if (!hasCoral()) {
          state = ArmStates.ALGAE_IN;
        }
        break;
    
      default:
        if (hasAlgae()) {
          if (hasCoral()) {
            state = ArmStates.BOTH_IN;
          } else {
            state = ArmStates.ALGAE_IN;
          }
        } else if (hasCoral()) {
          state = ArmStates.CORAL_IN;
        } else {
          state = ArmStates.DEFAULT;
        }
        break;
    }
  }
}
