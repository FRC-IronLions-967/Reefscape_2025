// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Utils.Constants;

public class SdsSwerveModule {

  private SparkMax driveMotor;
  private SparkClosedLoopController driveMotorController;
  private SparkMaxConfig driveConfig;
  private SparkMax turningMotor;
  private SparkMaxConfig turningConfig;
  private SparkClosedLoopController turningPIDController;
  private FeedbackSensor turningFeedbackSensor;

  private int driveID;
  private int turnID;

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorCANId PWM output for the drive motor.
   * @param turningMotorCANId PWM output for the turning motor.
   * @param turningEncoderAnalogPort Analog input for the turning encoder channel A
   */
  public SdsSwerveModule(
      int driveMotorCANId,
      int turningMotorCANId) {

    driveMotor = new SparkMax(driveMotorCANId, MotorType.kBrushless);
    driveConfig = new SparkMaxConfig();
    driveMotorController = driveMotor.getClosedLoopController();

    turningMotor = new SparkMax(turningMotorCANId, MotorType.kBrushless);
    turningConfig = new SparkMaxConfig();
    turningPIDController = turningMotor.getClosedLoopController();

    
    
    


    //Drive Motor Config
    driveConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).secondaryCurrentLimit(80).inverted(true);

    driveConfig.encoder.velocityConversionFactor((2.0 * Math.PI * Constants.kWheelRadius) / 
    (Constants.kSecondsPerMinute * Constants.kGearRatio)).positionConversionFactor((2.0 * Math.PI * Constants.kWheelRadius) / Constants.kGearRatio);

    driveConfig.closedLoop.pidf(Constants.swerveDriveMotorP, Constants.swerveDriveMotorI, Constants.swerveDriveMotorD, Constants.swerveDriveMotorFF);




    //Turning Motor Config
    turningConfig.idleMode(IdleMode.kBrake);
    turningConfig.encoder.positionConversionFactor(2*Math.PI).inverted(true);

    turningConfig.closedLoop.pid(Constants.swerveTurningP, Constants.swerveTurningI, Constants.swerveTurningD)
    .positionWrappingEnabled(true).feedbackSensor(turningFeedbackSensor.kPrimaryEncoder);//KIRA CHECK THIS! Old code: turningPIDController.setFeedbackDevice(turningMotor.getAbsoluteEncoder()); 

    turningConfig.signals.primaryEncoderPositionPeriodMs(20);
    // turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    // turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20); //THIS IS THE OLD CODE SOMEONE(KIRA) SHOULD DOUBLE CHECK THIS.

    turningMotor.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //This is needed to acsess outside the constructor.
    driveID = driveMotorCANId;
    turnID = turningMotorCANId;
  }


  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveMotor.getEncoder().getVelocity(), new Rotation2d(ConvertedTurningPosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveMotor.getEncoder().getPosition(), new Rotation2d(ConvertedTurningPosition()));
  }

  public void changeDriveToBrake() {
    driveConfig.idleMode(IdleMode.kBrake);
  }

  public void changeDriveToCoast() {
    driveConfig.idleMode(IdleMode.kCoast);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(ConvertedTurningPosition()));

    double convertedPosition = MathUtil.angleModulus(state.angle.getRadians()) + Math.PI;
      
    // if (i == 0) {
    //   System.out.println("Measured Angle   " + driveID + ":   " + ConvertedTurningPosition());
    //   System.out.println("Commanded Angle  " + driveID + ":   " + state.angle.getRadians());
    //   System.out.println("Commanded Speed " + driveID + ":   " + state.speedMetersPerSecond);
    //   System.out.println("Motor Speed     " + driveID + ": " + driveMotor.getEncoder().getVelocity());
    //   SmartDashboard.putNumber("Swerve Angle " + turnID, state.angle.getRadians());
    // }
    // i = (i + 1) % 100;

    // SmartDashboard.putNumber("Drive RPM" + driveID, driveMotor.getEncoder().getVelocity());
    // SmartDashboard.putNumber("Module Angle" + driveID,turningEncoder.getAbsolutePosition());
    // SmartDashboard.putNumber("Commanded Angle" + driveID, state.angle.getRadians());

    turningPIDController.setReference(convertedPosition, ControlType.kPosition);
    driveMotorController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
    //if (turnOutput > 0.5 ) {
    //  turningMotor.setVoltage(turnOutput);
    //} else {
    //  turningMotor.setVoltage(0);
    //}
  }

  private double ConvertedTurningPosition() {
    return turningMotor.getAbsoluteEncoder().getPosition();
  }

}