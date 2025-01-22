package frc.robot.Utils;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import static edu.wpi.first.units.Units.*;

public final class Constants {

    public static final double kDriveRadius = Math.sqrt(0.308 * 0.308 + 0.308 * 0.308); //radius from center of drive to one module
    public static final double kMaxSpeed = 4.42; // 4.42 meters per second / 14.5 ft per second
    public static final double kMaxAcceleration = 10.0; // 6.0 meters per second per second
    public static final double kMaxAngularSpeed = kMaxSpeed / kDriveRadius; // Maximum angular velocity 
    public static final double kMaxAngularAcceleration = kMaxAcceleration / kDriveRadius; // Maximum angular acceleration
    public static final double kWheelRadius = 0.0489;
    //the number above is acurate
    public static final double kCoefficientFriction = 1.542;
    public static final double kDriveGearRatio = 6.75;
    public static final double kSteerGearRatio = 150.0/7.0;
    //needs tunings
    public static final double kSecondsPerMinute = 60.0;

    public static final double objectDetectionCameraYawOffset = 0.0;
    public static final double kMaxNeoSpeed = 5676.0;

    public static final double swerveTurningP = 1.5;
    public static final double swerveTurningI = 0.0;
    public static final double swerveTurningD = 0.5;
    public static final double swerveDriveMotorP = 0.08;
    public static final double swerveDriveMotorI = 0.0;
    public static final double swerveDriveMotorD = 0.025;
    public static final double swerveDriveMotorFF = 0.28;

    // Sim Feedforward
    // Linear drive feed forward
    public static final SimpleMotorFeedforward kDriveSimFF =
            new SimpleMotorFeedforward( // real
                    0.25, // Voltage to break static friction
                    2.65, // Volts per meter per second
                    0.3 // Volts per meter per second squared
                    );
    // Steer feed forward
    public static final SimpleMotorFeedforward kSteerSimFF =
            new SimpleMotorFeedforward( // real
                    0.1, // Voltage to break static friction
                    0.1, // Volts per radian per second
                    0.01 // Volts per radian per second squared
                    );
  
    // private static final double kModuleMaxAngularVelocity = kMaxAngularSpeed;
    // private static final double kModuleMaxAngularAcceleration =
    //     2 * Math.PI; // radians per second squared
    public static final Translation2d m_frontLeftLocation = new Translation2d(0.308, 0.308);
    public static final Translation2d m_frontRightLocation = new Translation2d(0.308, -0.308);
    public static final Translation2d m_backLeftLocation = new Translation2d(-0.308, 0.308);
    public static final Translation2d m_backRightLocation = new Translation2d(-0.308, -0.308);
      //real numbers xare put in above
    public static final SwerveDriveKinematics m_kinematics =
    new SwerveDriveKinematics(
       m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
    new TrapezoidProfile.Constraints(
        kMaxAngularSpeed, kMaxAngularAcceleration);

    public static final RobotConfig autoConfig = new RobotConfig(
          Kilograms.of(45.0), 
          KilogramSquareMeters.of(1.45), 
          new ModuleConfig(kWheelRadius, kMaxSpeed, kCoefficientFriction, DCMotor.getNEO(1), 80.0, 4),
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    // Vision Constants
    //Cam mounted facing backward, 0.298 meters behind center, 0.58 meters up from center.
    public static final Transform3d kRobotToCam =
        new Transform3d(new Translation3d(-0.298, 0.0, 0.58), new Rotation3d(0, 0.2, Math.PI)); 

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    public static final double elevatorGearRatio =  1.0; // needs tuning
    public static final double armGearRadius = 1.0; //needs tuning
    public static final double coralWheelRadius = 1.0; //needs tunig
    public static final double algaeWheelRadius = 1.0; //needs tuning\

    public static final double[] L4Position = {1, 1}; //needs tuning
    public static final double[] L3Position = {1, 1}; //needs tuning
    public static final double[] L2Position = {1, 1}; //needs tuning
    public static final double[] coralStationPosition = {1, 1}; //needs tuning
    public static final double[] L2AlgaePosition = {1, 1}; //needs tuning
    public static final double[] L3AlgaePosition = {1, 1}; //needs tuning
    public static final double[] processorPosition = {1, 1}; //needs tuning
    public static final double[] bargePosition = {1, 1}; //needs tuning
    public static final double[] defaultPosition = {1, 1}; //needs tuning

    public static final double coralIntakeSpeed = 5600; //needs tuning
    public static final double coralScoringSpeed = 5600; //needs tuning
    public static final double algaeIntakeSpeed = 5600; //needs tuning
    public static final double algaeScoringSpeed = 5600; //needs tuning

    public static final double armFullRotationElevatorHeight = 1.5;
    public static final double armWithAlgaeFullRotationElevatorHeight = 9.5;
    public static final double emptyArmMinConstraintForAlgaeManipulator = 4.607;
    public static final double emptyArmMaxConstraintForAlgaeManipulator = 6.178;
    public static final double armWithAlgaeMinTopConstraint = 0.45;
    public static final double armWithAlgaeMaxTopConstraint = 2.69;
    public static final double armWithAlgaeMinBottomConstraint = 3.67;
    public static final double armWithAlgaeMaxBottomConstraint = 5.83;
    public static final double armWithCoralMinConstraint = 1.76;
    public static final double armWithCoralMaxConstraint = 2.57;

    //CASE EMPTY: vertical +- 0.105rad OR elevator +1.5in
    //CASE CORAL vertical 0.192 - 1 to the right OR elevator +1.5in
    //CASE ALGAE vertical +- 1.04rad bottom and +- 1.12rad OR elevator +9.5in and +-1.04rad bottom
    //Case BOTH +- 1.12 top and bottom vertical OR elevator +9.5in and +-1.04rad Algae side bottom
    


}
