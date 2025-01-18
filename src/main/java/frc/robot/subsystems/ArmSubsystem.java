// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import java.util.function.BooleanSupplier;

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
import frc.robot.lib.LimitSwitchManager;

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

  private BooleanSupplier coralLimitSwitch;
  private BooleanSupplier algaeLimitSwitch;

  private double elevatorHeightEndGoal;
  private double elevatorHeightCurrentTarget;
  private double rotaryArmEndGoal;
  private double rotaryArmCurrentTarget;

  private ArmStates state;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {

    state = ArmStates.EMPTY;

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
      .positionConversionFactor(Math.PI * 2);
    armVortexConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .pid(1, 0, 0)
      .positionWrappingInputRange(0, 2*Math.PI)
      .positionWrappingEnabled(true);

    armVortex.configure(armVortexConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);


    coralManipulatorVortexConfig
      .idleMode(IdleMode.kCoast);
    coralManipulatorVortexConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(1, 0, 0);

    coralManipulatorVortex.configure(coralManipulatorVortexConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);


    algaeManipulatorVortexConfig
      .idleMode(IdleMode.kCoast);
    algaeManipulatorVortexConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(1, 0, 0);

    algaeManipulatorVortex.configure(algaeManipulatorVortexConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);


    coralLimitSwitch = LimitSwitchManager.getSwitch(0);
    algaeLimitSwitch = LimitSwitchManager.getSwitch(1);
    
  }

  public void moveElevator(double position) {
    elevatorHeightEndGoal = position;
  }

  public double getElevatorPosition() {
    return elevatorVortex.getEncoder().getPosition();
  }

  public void moveArm(double angle) {
    rotaryArmEndGoal = angle;
  }

  public double getArmAngle() {
    return armVortex.getEncoder().getPosition();
  }

  public void runCoralManipulator(double speed) {
    coralManipulatorVortexController.setReference(speed, ControlType.kVelocity);
  }

  public boolean hasCoral() {
    return coralLimitSwitch.getAsBoolean();
  }

  public void runAlgaeManipulator(double speed) {
    algaeManipulatorVortexController.setReference(speed, ControlType.kVelocity);
  }

  public boolean hasAlgae() {
    return algaeLimitSwitch.getAsBoolean();
  }

  private void setArmLoadingState() {
    if(hasAlgae() && hasCoral()) {
      state = ArmStates.BOTH_IN;
    } else if (hasAlgae()) {
      state = ArmStates.ALGAE_IN;
    } else if (hasCoral()) {
      state = ArmStates.CORAL_IN;
    } else {
      state = ArmStates.EMPTY;
    }
  }
  
  /**
   * 
   * @param variable The variable you are checking
   * @return This returns true if the position the rotation arm is set to a safe location.
   */

  private boolean isEndTargetGood(double variable) {
    if (getElevatorPosition() < 1.5) {
      if (4.607 < variable && variable < 6.178) {
        return false;
      }
      if (state == ArmStates.ALGAE_IN && ((0.45 < variable && variable < 2.69)
      || (3.67 < variable && variable < 5.752))) {
        return false; 
      } else if (state == ArmStates.CORAL_IN && (4.904 < variable && variable < 5.712)) {
        return false;
      } else if (state == ArmStates.BOTH_IN && ((0.451 < variable && variable < 2.69) 
      || (3.59 < variable && variable < 5.83))) {
        return false;
      }
    } else if (1.5 <= getElevatorPosition() && getElevatorPosition() < 9.5) {
      if (state == ArmStates.ALGAE_IN && ((0.45 < variable && variable < 2.69)
      || (3.67 < variable && variable < 5.752))) {
        return false; 
      } else if (state == ArmStates.BOTH_IN && ((0.451 < variable && variable < 2.69) 
      || (3.59 < variable && variable < 5.83))) {
        return false;
      }
    } else if (getElevatorPosition() >= 9.5) {
      if ((state == ArmStates.ALGAE_IN || state == ArmStates.BOTH_IN) && 
      (3.67 < variable && variable < 5.752)) {
        return false; 
      } 
    }
    return true;
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run  

    // transitions are less important than just knowing the current state
    setArmLoadingState();

    switch (state) {
      case EMPTY:
        // Determine elevator/arm movement restraints : Cant Go 4PI/3-5PI/3
        // Set PID targets, may need to use intermediate values,
        // then move to the final goal
        if (getElevatorPosition() <= 1.5) {
          if (Math.PI/3 < getArmAngle() && getArmAngle() < 2/3 * Math.PI) {
            //If on the top go to the end position
            rotaryArmCurrentTarget = rotaryArmEndGoal;
          } else if((1.5*Math.PI < getArmAngle() || getArmAngle() < Math.PI/2) && (Math.PI/2 < rotaryArmEndGoal && rotaryArmEndGoal < 3*Math.PI/2)) {
            //If on the right side of the robot and needs to go to the left, go to the top.
            rotaryArmCurrentTarget = Math.PI/2;
          } else if((0.5*Math.PI < getArmAngle() && getArmAngle() < 1.5*Math.PI) && (1.5 * Math.PI < rotaryArmEndGoal || rotaryArmEndGoal < 0.5*Math.PI)) {
            //If on the left side of the robot and needs to go to the right, go to the top.
            rotaryArmCurrentTarget = Math.PI/2;
          } else if ((0.5*Math.PI < getArmAngle() && getArmAngle() < 1.5*Math.PI) && (0.5 * Math.PI < rotaryArmEndGoal && rotaryArmEndGoal < 1.5 *Math.PI)) {
            // If on the left side and needs to be on the left go the position
            rotaryArmCurrentTarget = rotaryArmEndGoal;
          } else if ((1.5 * Math.PI < getArmAngle() || getArmAngle() < 0.5*Math.PI) && (1.5 * Math.PI < rotaryArmEndGoal || rotaryArmEndGoal < 0.5*Math.PI)) {
            // If on the right side and needs to be on the right go the position
          }
          if (isEndTargetGood(rotaryArmCurrentTarget)) {
            armVortexController.setReference(rotaryArmCurrentTarget, ControlType.kPosition);
            //sets the position
          }
        } else {
          armVortexController.setReference(rotaryArmEndGoal, ControlType.kPosition);
        }
        break;

      case ALGAE_IN:
        // Determine elevator/arm movement restraints : Can't Go 5PI/4-7PI/4
        // Set PID targets, may need to use intermediate values,
        // then move to the final goal
        break;

      case CORAL_IN:
        // Determine elevator/arm movement restraints : Can't Go PI/4 - 3PI/4 and no 4PI/3-5PI/3
        // Set PID targets, may need to use intermediate values,
        // then move to the final goal
        break;

      case BOTH_IN:
        // Determine elevator/arm movement restraints : ONLY can go 7PI/4 - PI/4 OR 3PI/4 - 5PI/4
        // Set PID targets, may need to use intermediate values,
        // then move to the final goal
        break;
    
      default:
        
        break;
    }

    
  }
}
