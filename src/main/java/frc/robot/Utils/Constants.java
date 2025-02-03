package frc.robot.Utils;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
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
    public static final double kWheelRadius = 0.04;
    //the number above is acurate
    public static final double kCoefficientFriction = 1.1;
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
                    0.2, // Voltage to break static friction
                    0.1, // Volts per radian per second
                    0.02 // Volts per radian per second squared
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
    //Cam mounted on the right front swerve module facing the reef when scoring.
    public static final Transform3d kRobotToCamFront = 
        new Transform3d(new Translation3d (0.216, -0.305, 0.154), new Rotation3d(0.0,-0.349, 0.785));
    public static final Transform3d kRobotToCamRear =
        new Transform3d(new Translation3d(-0.235, 0.210, 0.176), new Rotation3d(0.0, -1.04, Math.PI)); 

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    public static final Pose3d kLeftRedStation = kTagLayout.getTagPose(1).get();
    public static final Pose3d kRightRedStation = kTagLayout.getTagPose(2).get();
    public static final Pose3d kLeftBlueStation = kTagLayout.getTagPose(13).get();
    public static final Pose3d kRightBlueStation = kTagLayout.getTagPose(12).get();

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    public static final double elevatorGearRatio =  14.087; // taken from robot 2/1
    public static final double elevatorSprocketRadius = 0.8755; // inches
    public static final double armGearRatio = 103.448; //taken from CAD 1/28
    public static final double coralWheelRadius = 1.0; //needs tunig
    public static final double algaeWheelRadius = 1.0; //needs tuning\

    public static final double coralIntakeSpeed = 5600; //needs tuning
    public static final double coralScoringSpeed = 5600; //needs tuning
    public static final double algaeIntakeSpeed = 5600; //needs tuning
    public static final double algaeScoringSpeed = 5600; //needs tuning

    public static final double armWiringMinConstraint = 0.262;
    public static final double armWiringMaxConstraint = 4.101;
    public static final double armFullRotationElevatorHeight = 2.5;
    public static final double armWithAlgaeFullRotationElevatorHeight = 22;
    public static final double emptyArmConstraintForAlgaeManipulatorAtE0 = 2.679; // Shouldn't go past this without raising the elevator.
    public static final double armWithAlgaeMinConstraint = 1.288;
    public static final double armWithAlgaeMaxConstraint = 1.865;
    // State coral has no additional constraints.


    // Arm positions
    public static final double L2L3ArmAngle = 3.75;
    public static final double L4ArmAngle = 4.05;
    public static final double coralArmAngle = 0.681;
    public static final double reefAlgaeAngle = Math.PI / 2.0;
    public static final double bargeAlgaeAngle = 0.29;
    public static final double processorAlgaeAngle = 1.89;
    public static final double defaultArmAngle = Math.PI;
    // Elevator Positions
    public static final double L2ElevatorPosition = 4;
    public static final double L3ElevatorPosition = 9.25;
    public static final double L4ElevatorPosition = 26;
    public static final double L2AlgaeElevatorPosition = 8;
    public static final double L3AlgaeElevatorPosition = 15;
    public static final double coralElevatorPosition = 11.596;
    public static final double bargeElevatorPosition = 27;
    public static final double defaultElevatorPosition = 0;

    public static final double[] L4Position = {L4ElevatorPosition, L4ArmAngle};
    public static final double[] L3Position = {L3ElevatorPosition, L2L3ArmAngle};
    public static final double[] L2Position = {L2ElevatorPosition, L2L3ArmAngle};
    public static final double[] coralStationPosition = {coralElevatorPosition, coralArmAngle}; //needs tuning
    public static final double[] L2AlgaePosition = {L2AlgaeElevatorPosition, reefAlgaeAngle}; //needs tuning
    public static final double[] L3AlgaePosition = {L3AlgaeElevatorPosition, reefAlgaeAngle}; //needs tuning
    public static final double[] processorPosition = {defaultElevatorPosition, processorAlgaeAngle}; //needs tuning
    public static final double[] bargePosition = {bargeElevatorPosition, bargeAlgaeAngle}; //needs tuning
    public static final double[] defaultPosition = {defaultElevatorPosition, defaultArmAngle};

    


}
