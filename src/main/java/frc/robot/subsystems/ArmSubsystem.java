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
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  // Create a Mechanism2d visualization of the elevator
  private final Mechanism2d mech2d = new Mechanism2d(40, 90);
  private final MechanismRoot2d elevator2dRoot = mech2d.getRoot("Elevator Root", 20, 1);
  private final MechanismLigament2d elevatorMech2d =
      elevator2dRoot.append(new MechanismLigament2d("Elevator", 21, 90));
  private final MechanismRoot2d arm2dRoot = mech2d.getRoot("Arm Root", 20, 21);
  private final MechanismLigament2d armMech2d =
      arm2dRoot.append(new MechanismLigament2d("Arm", 18, 90));

  private SingleJointedArmSim armSim;
  private SparkFlexSim armVortexSim;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {

    state = ArmStates.STARTUP;

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
      .smartCurrentLimit(40)
      .idleMode(IdleMode.kBrake);
    elevatorVortexConfig.encoder
      .positionConversionFactor((2.0 * Constants.elevatorSprocketRadius * Math.PI) / Constants.elevatorGearRatio) // to meters
      .velocityConversionFactor((2.0 * Constants.elevatorSprocketRadius * Math.PI) / (60.0 * Constants.elevatorGearRatio)); //to meters/sec
    elevatorVortexConfig.analogSensor
      .positionConversionFactor(5.0 / Units.inchesToMeters(2.0)) // native 0-5V, 2 meter travel
      .velocityConversionFactor(5.0 / Units.inchesToMeters(2.0)); // 
    elevatorVortexConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAnalogSensor)
      .pid(1.0, 0, 0);

    elevatorVortex.configure(elevatorVortexConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);


    armVortexConfig
      .smartCurrentLimit(40)
      .idleMode(IdleMode.kBrake);
    armVortexConfig.absoluteEncoder
      .velocityConversionFactor(2.0 * Math.PI / 60.0)
      .positionConversionFactor(Math.PI * 2);
    armVortexConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .pid(1.0, 0, 0.0)
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
      Units.inchesToMeters(Constants.defaultElevatorPosition-1), 
      Units.inchesToMeters(Constants.bargeElevatorPosition+1), 
      true, 
      0.0, 
      0.001, 0.0);

    // Publish Mechanism2d to SmartDashboard
    // To view the Elevator visualization, select Network Tables -> SmartDashboard -> Elevator Sim
    SmartDashboard.putData("Elevator Sim", mech2d);

    armVortexSim = new SparkFlexSim(armVortex, DCMotor.getNeoVortex(1));
    armSim = new SingleJointedArmSim(
      DCMotor.getNeoVortex(1), 
      Constants.armGearRatio, 
      0.132, 
      0.67, 
      -2*Math.PI, 
      4*Math.PI, 
      true, 
      Math.PI*1.5, 
      0.001, 0.0);
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
    //return elevatorVortex.getEncoder().getPosition();
    return elevatorVortex.getAnalog().getPosition() + Constants.kElevatorAnalogZeroOffset;
  }

  /**
   * Moves the rotary arm to a safe end goal.
   * @param angle the value that the rotary arm goes to.
   */
  public void moveArm(double angle) {
    if (Constants.armWiringMinConstraint >= angle) {
      rotaryArmEndGoal = Constants.armWiringMinConstraint;
    } else if (angle >= Constants.armWiringMaxConstraint) {
      rotaryArmEndGoal = Constants.armWiringMaxConstraint;
    } else {
      rotaryArmEndGoal = angle;
    }
  }

  /**
   * 
   * @return The rotary arm position.
   */
  public double getArmAngle() {
    return armVortex.getAbsoluteEncoder().getPosition();
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
  private void setArmState() {
    if (state != ArmStates.STARTUP || getElevatorPosition() >= Constants.armFullRotationElevatorHeight) {
      if (hasAlgae()) {
        state = ArmStates.ALGAE_IN;
      } else {
        state = ArmStates.EMPTY;
      }
    }
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run  

    // transitions are less important than just knowing the current state
    setArmState();

    switch (state) {
      case STARTUP:
        elevatorHeightCurrentTarget = elevatorHeightEndGoal;
        if (getElevatorPosition() <= Constants.armFullRotationElevatorHeight) {
          rotaryArmCurrentTarget = getArmAngle();
        }
      case EMPTY:

      //Makes Rotation Safe
        if (getElevatorPosition() <= Constants.armFullRotationElevatorHeight + 0.2) { // Fudge factor for imperfect positioning
          rotaryArmCurrentTarget = (rotaryArmEndGoal >= Constants.emptyArmConstraintForAlgaeManipulatorAtE0) ? Constants.emptyArmConstraintForAlgaeManipulatorAtE0 : rotaryArmEndGoal;
        } else {
          rotaryArmCurrentTarget = rotaryArmEndGoal;
        }

      //Makes Elevator Safe
        if (elevatorHeightEndGoal < getElevatorPosition() && elevatorHeightEndGoal < Constants.armFullRotationElevatorHeight && getArmAngle() >= Constants.emptyArmConstraintForAlgaeManipulatorAtE0) {
          elevatorHeightCurrentTarget = Constants.armFullRotationElevatorHeight;
        } else {
          elevatorHeightCurrentTarget = elevatorHeightEndGoal;
        }

        break;

      case ALGAE_IN:

      //Makes Rotation Safe
        if (getElevatorPosition() <= Constants.armWithAlgaeFullRotationElevatorHeight) {
          if (rotaryArmEndGoal <= Constants.armWithAlgaeMinConstraint) {
            rotaryArmCurrentTarget = Constants.armWithAlgaeMinConstraint;
          } else if (rotaryArmEndGoal >= Constants.armWithAlgaeMaxConstraint) {
            rotaryArmCurrentTarget = Constants.armWithAlgaeMaxConstraint;
          } else {
            rotaryArmCurrentTarget = rotaryArmEndGoal;
          }
        } else {
          rotaryArmCurrentTarget = (rotaryArmEndGoal >= Constants.armWithAlgaeMaxConstraint) ? Constants.armWithAlgaeMaxConstraint : rotaryArmEndGoal;
        }

      //Makes Elevation Safe
        if (elevatorHeightEndGoal < getElevatorPosition() && elevatorHeightCurrentTarget < Constants.armWithAlgaeFullRotationElevatorHeight 
        && (Constants.armWithAlgaeMinConstraint <= getArmAngle() && getArmAngle() <= Constants.armWithAlgaeMaxConstraint)) {
          elevatorHeightCurrentTarget = Constants.armWithAlgaeFullRotationElevatorHeight;
        } else {
          elevatorHeightCurrentTarget = elevatorHeightEndGoal;
        }

        break;

      default:
        
        break;
    }

    armVortexController.setReference(rotaryArmCurrentTarget, ControlType.kPosition);
    elevatorVortexController.setReference(elevatorHeightCurrentTarget, ControlType.kPosition);

  }  
  
  public void simulationPeriodic() {
    elevatorSim.setInput(elevatorVortex.getAppliedOutput() * 12.0);
    elevatorSim.update(Robot.kDefaultPeriod);
    elevatorVortexSim.iterate(Units.metersToInches(elevatorSim.getVelocityMetersPerSecond()), 12.0, Robot.kDefaultPeriod);
    elevatorMech2d.setLength(21.0 +  2.0 * Units.metersToInches(elevatorSim.getPositionMeters()));
    
    armSim.setInput(-armVortex.getAppliedOutput() * 12.0);
    armSim.update(Robot.kDefaultPeriod);
    armVortexSim.iterate(-armSim.getVelocityRadPerSec(), 12.0, Robot.kDefaultPeriod);
    arm2dRoot.setPosition(20, 21.0 +  2.0 * Units.metersToInches(elevatorSim.getPositionMeters()));
    armMech2d.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
  }
}