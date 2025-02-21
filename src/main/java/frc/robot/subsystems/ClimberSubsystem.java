// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

  private SparkMax climberMotor;
  private SparkMaxConfig climberMotorConfig;
  private SparkClosedLoopController climberMotorController;
  private Servo ratchetServo;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    climberMotor = new SparkMax(13, MotorType.kBrushless);
    climberMotorConfig = new SparkMaxConfig();
    climberMotorConfig
      .smartCurrentLimit(40)
      .idleMode(IdleMode.kBrake);
    climberMotorConfig.closedLoop
      .feedbackSensor.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(1, 0, 0);
    climberMotorConfig.absoluteEncoder.positionConversionFactor(Math.PI * 2);

    ratchetServo = new Servo();
  }

  private boolean isRatchetOn() {
    return ratchetServo.getPosition();
  }

  private void moveRatchet(double position) {
    ratchetServo.set(position);
  }

  private double getClimbPosition() {
    return climberMotor.getAnalog().getPosition();
  }

  private void moveClimberArm(double position) {
    climberMotorController.setReference(position, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
