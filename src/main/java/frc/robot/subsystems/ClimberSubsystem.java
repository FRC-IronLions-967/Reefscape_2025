// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Servo;
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
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(1, 0, 0);
    climberMotorConfig.absoluteEncoder.positionConversionFactor(Math.PI * 2);

    ratchetServo = new Servo(0);
  }

  public boolean isRatchetOn() {
    return ratchetServo.get() < 0;
  }

  public void moveRatchet(double position) {
    ratchetServo.set(position);
  }

  public double getClimbPosition() {
    return climberMotor.getAnalog().getPosition();
  }

  public void moveClimberArm(double position) {
    climberMotorController.setReference(position, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
