// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

import java.util.function.BooleanSupplier;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
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

  // ----- Simulation -----
  private ElevatorSim elevatorSim;
  private SparkFlexSim elevatorVortexSim;

  private SingleJointedArmSim armSim;
  private SparkFlexSim armVortexSim;

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
      .positionConversionFactor((2.0 * Constants.elevatorSprocketRadius * Math.PI) / Constants.elevatorGearRatio) // to meters
      .velocityConversionFactor((2.0 * Constants.elevatorSprocketRadius * Math.PI) / (60.0 * Constants.elevatorGearRatio)); //to meters/sec
    elevatorVortexConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(10.0, 0, 0);

    elevatorVortex.configure(elevatorVortexConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);


    armVortexConfig
      .idleMode(IdleMode.kBrake);
    armVortexConfig.absoluteEncoder
      .velocityConversionFactor(60.0 * 2.0 * Math.PI)
      .positionConversionFactor(Math.PI * 2);
    armVortexConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .pid(1, 0, 0)
      .positionWrappingInputRange(0, 2*Math.PI)
      .positionWrappingEnabled(false);

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
    
    // ----- Simulation -----
    elevatorVortexSim = new SparkFlexSim(elevatorVortex, DCMotor.getNeoVortex(1));
    elevatorSim = new ElevatorSim(
      DCMotor.getNeoVortex(1), 
      Constants.elevatorGearRatio,
      6.0, //guess and replace with constant 
      Units.inchesToMeters(Constants.elevatorSprocketRadius), //correct, replace with constant
      0.0, 
      1.3716, 
      true, 
      0.0, 
      0.1, 0.0);

    armVortexSim = new SparkFlexSim(armVortex, DCMotor.getNeoVortex(1));
    armSim = new SingleJointedArmSim(
      DCMotor.getNeoVortex(1), 
      Constants.armGearRatio, 
      0.4386668405, 
      0.67, 
      0.0, 
      2.0, 
      true, 
      0.5, 
      0.01, 0.0);
  }

  /**
   * Moves the elevator end goal.
   * @param position the value that the elevator goes to.
   */
  public void moveElevator(double position) {
    elevatorHeightEndGoal = position;
  }

  /**
   * 
   * @return The elevator position.
   */
  public double getElevatorPosition() {
    return elevatorVortex.getEncoder().getPosition();
  }

  /**
   * Moves the rotary arm end goal.
   * @param angle the value that the rotary arm goes to.
   */
  public void moveArm(double angle) {
    rotaryArmEndGoal = angle;
  }

  /**
   * 
   * @return The rotary arm position.
   */
  public double getArmAngle() {
    return armVortex.getEncoder().getPosition();
  }

  /**
   * Sets the coral manipulator to a speed.
   * @param speed The speed at which the wheel runs.
   */
  public void runCoralManipulator(double speed) {
    coralManipulatorVortexController.setReference(speed, ControlType.kVelocity);
  }

  /**
   * 
   * @return If the coral manipulator has Coral in it.
   */
  public boolean hasCoral() {
    return coralLimitSwitch.getAsBoolean();
  }

  /**
   * Sets the Algae manipulator to a speed.
   * @param speed The speed at which the wheels run.
   */
  public void runAlgaeManipulator(double speed) {
    algaeManipulatorVortexController.setReference(speed, ControlType.kVelocity);
  }

  /**
   * 
   * @return If the algae manipulator has Algae in it.
   */
  public boolean hasAlgae() {
    return algaeLimitSwitch.getAsBoolean();
  }

  /**
   * Sets the Arm State based on what game pieces are in the manipuators.
   */
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
   * @return This returns true if the position the rotation arm is set to a safe location.
   */

  private boolean isArmTargetGood() {
    if (getElevatorPosition() < Constants.armFullRotationElevatorHeight) {
      if (Constants.emptyArmMinConstraintForAlgaeManipulator < rotaryArmCurrentTarget && rotaryArmCurrentTarget < Constants.emptyArmMaxConstraintForAlgaeManipulator) {
        return false;
      }
      if (state == ArmStates.ALGAE_IN && ((Constants.armWithAlgaeMinTopConstraint < rotaryArmCurrentTarget && rotaryArmCurrentTarget < Constants.armWithAlgaeMaxTopConstraint)
      || (Constants.armWithAlgaeMinBottomConstraint < rotaryArmCurrentTarget && rotaryArmCurrentTarget < Constants.armWithAlgaeMaxBottomConstraint))) {
        return false; 
      } else if (state == ArmStates.CORAL_IN && (Constants.armWithCoralMinConstraint < rotaryArmCurrentTarget && rotaryArmCurrentTarget < Constants.armWithCoralMaxConstraint)) {
        return false;
      } else if (state == ArmStates.BOTH_IN && ((Constants.armWithAlgaeMinTopConstraint < rotaryArmCurrentTarget && rotaryArmCurrentTarget < Constants.armWithAlgaeMaxTopConstraint) 
      || (Constants.armWithAlgaeMinBottomConstraint < rotaryArmCurrentTarget && rotaryArmCurrentTarget < Constants.armWithAlgaeMaxBottomConstraint))) {
        return false;
      }
    } else if (Constants.armFullRotationElevatorHeight <= getElevatorPosition() && getElevatorPosition() < Constants.armWithAlgaeFullRotationElevatorHeight) {
      if (state == ArmStates.ALGAE_IN && ((Constants.armWithAlgaeMinTopConstraint < rotaryArmCurrentTarget && rotaryArmCurrentTarget < Constants.armWithAlgaeMaxTopConstraint)
      || (Constants.armWithAlgaeMinBottomConstraint < rotaryArmCurrentTarget && rotaryArmCurrentTarget < Constants.armWithAlgaeMaxBottomConstraint))) {
        return false; 
      } else if (state == ArmStates.BOTH_IN && ((Constants.armWithAlgaeMinTopConstraint < rotaryArmCurrentTarget && rotaryArmCurrentTarget < Constants.armWithAlgaeMaxTopConstraint) 
      || (Constants.armWithAlgaeMinBottomConstraint < rotaryArmCurrentTarget && rotaryArmCurrentTarget < Constants.armWithAlgaeMaxBottomConstraint))) {
        return false;
      }
    } else if (getElevatorPosition() >= Constants.armWithAlgaeFullRotationElevatorHeight) {
      if ((state == ArmStates.ALGAE_IN || state == ArmStates.BOTH_IN) && 
      (Constants.armWithAlgaeMinBottomConstraint < rotaryArmCurrentTarget && rotaryArmCurrentTarget < Constants.armWithAlgaeMaxBottomConstraint)) {
        return false; 
      }
    }
    return true;
  } 


  /**
   * Makes the elevator a safe value based on the current position of the arm.
   */
  private void makeElevatorTargetGood() {
    if (elevatorHeightCurrentTarget < Constants.armFullRotationElevatorHeight) {
      if (state == ArmStates.EMPTY) {
        if (Constants.emptyArmMinConstraintForAlgaeManipulator < getArmAngle() && getArmAngle() < Constants.emptyArmMaxConstraintForAlgaeManipulator) {
          elevatorHeightCurrentTarget = ((getElevatorPosition() > Constants.armFullRotationElevatorHeight) ? Constants.armFullRotationElevatorHeight : elevatorHeightCurrentTarget);
        } else {
          elevatorHeightCurrentTarget = elevatorHeightEndGoal;
        }
      } else if (state == ArmStates.ALGAE_IN) {
        if (((Constants.armWithAlgaeMinTopConstraint < getArmAngle() && getArmAngle() < Constants.armWithAlgaeMaxTopConstraint)
        || (Constants.armWithAlgaeMinBottomConstraint < getArmAngle() && getArmAngle() < Constants.armWithAlgaeMaxBottomConstraint))) {
          elevatorHeightCurrentTarget = ((getElevatorPosition() > Constants.armWithAlgaeFullRotationElevatorHeight) 
          ? Constants.armWithAlgaeFullRotationElevatorHeight : elevatorHeightCurrentTarget);
        } else {
          elevatorHeightCurrentTarget = elevatorHeightEndGoal;
        }
      } else if (state == ArmStates.CORAL_IN) {
        if (Constants.armWithCoralMinConstraint < getArmAngle() && getArmAngle() < Constants.armWithCoralMaxConstraint) {
          elevatorHeightCurrentTarget = ((getElevatorPosition() > Constants.armFullRotationElevatorHeight) ? Constants.armFullRotationElevatorHeight : elevatorHeightCurrentTarget);
        } else {
          elevatorHeightCurrentTarget = elevatorHeightEndGoal;
        }
      } else if (state == ArmStates.BOTH_IN) {
        if ((Constants.armWithAlgaeMinTopConstraint < getArmAngle() && getArmAngle() < Constants.armWithAlgaeMaxTopConstraint) 
        || (Constants.armWithAlgaeMinBottomConstraint < getArmAngle() && getArmAngle() < Constants.armWithAlgaeMaxBottomConstraint)) {
          elevatorHeightCurrentTarget = ((getElevatorPosition() > Constants.armWithAlgaeFullRotationElevatorHeight) 
          ? Constants.armWithAlgaeFullRotationElevatorHeight : elevatorHeightCurrentTarget);
        } else {
          elevatorHeightCurrentTarget = elevatorHeightEndGoal;
        }
      }
    } else if (elevatorHeightCurrentTarget < Constants.armWithAlgaeFullRotationElevatorHeight) {
      if (state == ArmStates.ALGAE_IN) {
        if (((Constants.armWithAlgaeMinTopConstraint < getArmAngle() && getArmAngle() < Constants.armWithAlgaeMaxTopConstraint)
        || (Constants.armWithAlgaeMinBottomConstraint < getArmAngle() && getArmAngle() < Constants.armWithAlgaeMaxBottomConstraint))) {
          elevatorHeightCurrentTarget = ((getElevatorPosition() > Constants.armWithAlgaeFullRotationElevatorHeight) 
          ? Constants.armWithAlgaeFullRotationElevatorHeight : elevatorHeightCurrentTarget);
        } else {
          elevatorHeightCurrentTarget = elevatorHeightEndGoal;
        }
      } else if (state == ArmStates.BOTH_IN) {
        if ((Constants.armWithAlgaeMinTopConstraint < getArmAngle() && getArmAngle() < Constants.armWithAlgaeMaxTopConstraint) 
        || (Constants.armWithAlgaeMinBottomConstraint < getArmAngle() && getArmAngle() < Constants.armWithAlgaeMaxBottomConstraint)) {
          elevatorHeightCurrentTarget = ((getElevatorPosition() > Constants.armWithAlgaeFullRotationElevatorHeight) 
          ? Constants.armWithAlgaeFullRotationElevatorHeight : elevatorHeightCurrentTarget);
        } else {
          elevatorHeightCurrentTarget = elevatorHeightEndGoal;
        }
      }
    } else {
      elevatorHeightCurrentTarget = elevatorHeightEndGoal;
    }
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
        if (getElevatorPosition() <= Constants.armFullRotationElevatorHeight) {
          if (Math.PI/3 < getArmAngle() && getArmAngle() < 2/3 * Math.PI) {
            //If on the top go to the end position
            rotaryArmCurrentTarget = rotaryArmEndGoal;
          } else if((1.5*Math.PI < getArmAngle() || getArmAngle() < Math.PI/2) && (Math.PI/2 < rotaryArmEndGoal && rotaryArmEndGoal < 3*Math.PI/2)) {
            //If on the right side of the robot and needs to go to the left, go to the top.
            rotaryArmCurrentTarget = Math.PI/2;
          } else if((0.5*Math.PI < getArmAngle() && getArmAngle() < 1.5*Math.PI) && (1.5 * Math.PI < rotaryArmEndGoal || rotaryArmEndGoal < 0.5*Math.PI)) {
            //If on the left side of the robot and needs to go to the right, go to the top.
            rotaryArmCurrentTarget = Math.PI/2;
          } else if (((0.5*Math.PI < getArmAngle() && getArmAngle() < 1.5*Math.PI) && (0.5 * Math.PI < rotaryArmEndGoal && rotaryArmEndGoal < 1.5 *Math.PI))
          || ((1.5 * Math.PI < getArmAngle() || getArmAngle() < 0.5 * Math.PI) && (1.5 * Math.PI < rotaryArmEndGoal || rotaryArmEndGoal < 0.5*Math.PI))) {
            // If on the same side and needs to be on the same go the position
            rotaryArmCurrentTarget = rotaryArmEndGoal;
          }
        } else {
          rotaryArmCurrentTarget = rotaryArmEndGoal;
        }
        break;

      case ALGAE_IN:
        if (getElevatorPosition() <= Constants.armWithAlgaeFullRotationElevatorHeight) {
          if (((Constants.armWithAlgaeMaxBottomConstraint < getArmAngle() || getArmAngle() < Constants.armWithAlgaeMinTopConstraint) && (Constants.armWithAlgaeMaxBottomConstraint < rotaryArmEndGoal || rotaryArmEndGoal < Constants.armWithAlgaeMinTopConstraint))
           || (((Constants.armWithAlgaeMaxTopConstraint < getArmAngle() && getArmAngle() < Constants.emptyArmMinConstraintForAlgaeManipulator) && (Constants.armWithAlgaeMaxTopConstraint < rotaryArmEndGoal && rotaryArmEndGoal < Constants.emptyArmMinConstraintForAlgaeManipulator)))) {
            rotaryArmCurrentTarget = rotaryArmEndGoal;
            //If the arm is on the same side go to that position.
          } else {
            elevatorHeightCurrentTarget = Constants.armWithAlgaeFullRotationElevatorHeight;
            //Otherwise we need to lift the elevator.
          }
        } else {
          if (Math.PI / 3 < getArmAngle() && getArmAngle() < 2/3 * Math.PI) {
            rotaryArmCurrentTarget = rotaryArmEndGoal;
            // If it is on the top we can just go to our position.
          } else if (((Constants.armWithAlgaeMaxBottomConstraint < getArmAngle() || getArmAngle() < Constants.armWithAlgaeMinTopConstraint) && (Constants.armWithAlgaeMaxBottomConstraint < rotaryArmEndGoal || rotaryArmEndGoal < Constants.armWithAlgaeMinTopConstraint))
          || (((Constants.armWithAlgaeMaxTopConstraint < getArmAngle() && getArmAngle() < Constants.emptyArmMinConstraintForAlgaeManipulator) && (Constants.armWithAlgaeMaxTopConstraint < rotaryArmEndGoal && rotaryArmEndGoal < Constants.emptyArmMinConstraintForAlgaeManipulator)))) {
           rotaryArmCurrentTarget = rotaryArmEndGoal;
           //If the arm is on the same side go to that position.
          } else {
            rotaryArmCurrentTarget = Math.PI/2;
            //Otherwise it will be on the opposite side so put it on the top.
          }
        }
        // Determine elevator/arm movement restraints : Can't Go 5PI/4-7PI/4
        // Set PID targets, may need to use intermediate values,
        // then move to the final goal
        break;

      case CORAL_IN:
        // Determine elevator/arm movement restraints : Can't Go PI/4 - 3PI/4 and no 4PI/3-5PI/3
        // Set PID targets, may need to use intermediate values,
        // then move to the final goal
        if (getElevatorPosition() <= Constants.armFullRotationElevatorHeight) {
          if (((2/3 * Math.PI<= getArmAngle() && getArmAngle() <= Constants.emptyArmMinConstraintForAlgaeManipulator) && (2/3 * Math.PI<= rotaryArmEndGoal && rotaryArmEndGoal <= Constants.emptyArmMinConstraintForAlgaeManipulator)) 
          || ((Constants.armWithCoralMaxConstraint <= getArmAngle() || getArmAngle() <= 2/3 * Math.PI) && (Constants.armWithCoralMaxConstraint <= rotaryArmEndGoal || getArmAngle() <= 2/3 * Math.PI))) {
            rotaryArmCurrentTarget = rotaryArmEndGoal;
            //If it is on the same side go the the position. 
          } else {
            elevatorHeightCurrentTarget = Constants.armFullRotationElevatorHeight;
            // If it is on the opposite side we need to raise the elevator because the algae manipulator hits the drive base
          }
        } else {
          rotaryArmCurrentTarget = rotaryArmEndGoal;
          //If it is higher then this, then it will be safe.
        }
        break;

      case BOTH_IN:
        // Determine elevator/arm movement restraints : ONLY can go 7PI/4 - PI/4 OR 3PI/4 - 5PI/4
        // Set PID targets, may need to use intermediate values,
        // then move to the final goal
        if (getElevatorPosition() <= Constants.armWithAlgaeFullRotationElevatorHeight) {
          if (((Constants.armWithAlgaeMaxBottomConstraint < getArmAngle() || getArmAngle() < Constants.armWithAlgaeMinTopConstraint) && (Constants.armWithAlgaeMaxBottomConstraint < rotaryArmEndGoal || rotaryArmEndGoal < Constants.armWithAlgaeMinTopConstraint)) 
          || ((Constants.armWithAlgaeMaxTopConstraint < getArmAngle() && getArmAngle() < Constants.armWithAlgaeMinBottomConstraint) && (Constants.armWithAlgaeMaxTopConstraint < rotaryArmEndGoal && rotaryArmEndGoal < Constants.armWithAlgaeMinBottomConstraint))) {
            rotaryArmCurrentTarget = rotaryArmEndGoal;
            // If the final position is on the same side as the end goal then move there.
          } else {
            elevatorHeightCurrentTarget = Constants.armWithAlgaeFullRotationElevatorHeight; 
          }
        } else {
          if (Constants.armWithAlgaeMinTopConstraint < getArmAngle() && getArmAngle() < Constants.armWithAlgaeMaxTopConstraint) {
            //if the current position is at the top, move to the final position.
            rotaryArmCurrentTarget = rotaryArmEndGoal;
          } else if (((Constants.armWithAlgaeMaxBottomConstraint < getArmAngle() || getArmAngle() < Constants.armWithAlgaeMinTopConstraint) && (Constants.armWithAlgaeMaxBottomConstraint < rotaryArmEndGoal || rotaryArmEndGoal < Constants.armWithAlgaeMinTopConstraint)) 
          || ((Constants.armWithAlgaeMaxTopConstraint < getArmAngle() && getArmAngle() < Constants.armWithAlgaeMinBottomConstraint) && (Constants.armWithAlgaeMaxTopConstraint < rotaryArmEndGoal && rotaryArmEndGoal < Constants.armWithAlgaeMinBottomConstraint))) {
            rotaryArmCurrentTarget = rotaryArmEndGoal;
            // If the final position is on the same side as the end goal then move there.
          } else {
            //If the final position is opposite the current position, go to the top.
            rotaryArmCurrentTarget = Math.PI/2;
          }
        }
        break;
    
      default:
        
        break;
    }
    if (isArmTargetGood()) {
      armVortexController.setReference(rotaryArmCurrentTarget, ControlType.kPosition);
      // If the arm target is good move there.
    }
    makeElevatorTargetGood();
    elevatorVortexController.setReference(elevatorHeightCurrentTarget, ControlType.kPosition);

  }  
  
  public void simulationPeriodic() {
    elevatorSim.setInput(elevatorVortex.getAppliedOutput() * 12.0);
    elevatorSim.update(Robot.kDefaultPeriod);
    elevatorVortexSim.iterate(Units.metersToInches(elevatorSim.getVelocityMetersPerSecond()), 12.0, Robot.kDefaultPeriod);
    
    armSim.setInput(armVortex.getAppliedOutput() * 12.0);
    armSim.update(Robot.kDefaultPeriod);
    armVortexSim.iterate(armSim.getVelocityRadPerSec(), 12.0, Robot.kDefaultPeriod);
  }
}