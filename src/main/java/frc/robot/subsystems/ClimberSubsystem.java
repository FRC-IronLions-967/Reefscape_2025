// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Constants;
import frc.robot.lib.LimitSwitchManager;

public class ClimberSubsystem extends SubsystemBase {

  private SparkMax climberMotor;
  private SparkMaxConfig climberMotorConfig;
  private SparkClosedLoopController climberMotorController;
  private Servo ratchetServo;
  private DigitalInput climberLimitSwitch;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    climberMotor = new SparkMax(13, MotorType.kBrushless);
    climberMotorConfig = new SparkMaxConfig();
    climberMotorConfig
      .smartCurrentLimit(40)
      .idleMode(IdleMode.kBrake);
    climberMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(1, 0, 0);

    climberMotor.configure(climberMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    climberMotorController = climberMotor.getClosedLoopController();

    ratchetServo = new Servo(1);

    climberLimitSwitch = new DigitalInput(0);
  }

  /**
   * Checks if ratchet is on.
   * @return If rachet is on
   */

  public boolean isRatchetOn() {
    return ratchetServo.get() < 0.6;
  }

  /**
   * Sets the ratchet servo to a position
   * @param position position (from 0 - 1) to set the servo to
   */

  public void moveRatchet(double position) {
    ratchetServo.set(position);
  }

  public double getRatchetPosition() {
    return ratchetServo.get();
  }

  /**
   * Gets the climber position
   * @return the climb psoition
   */

  public double getClimbPosition() {
    return climberMotor.getEncoder().getPosition();
  }

  /**
   * moves the climber arm
   * IMPORTANT: This method is defferent than runClimberArm!
   * This runs the climber based off position.
   * @param position position to move the climber arm to
   */

  public void moveClimberArm(double position) {
    climberMotorController.setReference(position, ControlType.kPosition);
  }

  /**
   * moves the climber arm 
   * IMPORTANT: This method is different than moveClimberArm!
   * This runs the climber based off speed.
   * @param speed
   */

  public void runClimberArm(double speed) {
    climberMotor.set(speed);
  }

  public boolean isClimberIn() {
    return climberLimitSwitch.get();
  }
  

  /**
   * Checks if the climber is in position
   * @param setPosition The position that the climber should be at
   * @return if the climber is at the set position
   */

  public boolean isClimberInPosition(double setPosition) {
    return setPosition - Constants.climberTolerance < getClimbPosition() && getClimbPosition() < setPosition + Constants.climberTolerance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Ratchet Position", getRatchetPosition());
    SmartDashboard.putBoolean("Is Ratchet On", isRatchetOn());
    SmartDashboard.putBoolean("Climber In", isClimberIn());
    // SmartDashboard.putNumber("Climber Position", getClimbPosition());
  }
}
