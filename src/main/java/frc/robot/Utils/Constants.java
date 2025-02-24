package frc.robot.Utils;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    public static final double kElevatorAnalogZeroOffset = 4.23;
    public static final double kArmZeroOffset = 1.961;

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

    public static final double coralIntakeSpeed = -6500; //needs tuning
    public static final double coralScoringSpeed = 6500; //needs tuning
    public static final double algaeIntakeSpeed = 5600; //needs tuning
    public static final double algaeScoringSpeed = -5600; //needs tuning

    public static final double armWiringMinConstraint = 0.0;
    public static final double armWiringMaxConstraint = 4.732;
    public static final double armFullRotationElevatorHeight = 5.0;
    public static final double armWithAlgaeFullRotationElevatorHeight = 23;
    public static final double emptyArmConstraintForAlgaeManipulatorAtE0 = 2.734; // Shouldn't go past this without raising the elevator.
    public static final double armWithAlgaeMinConstraint = 1.551;
    public static final double armWithAlgaeMaxConstraint = 2.103;
    // State coral has no additional constraints.


    // Arm positions
    public static final double L2L3ArmAngle = 4.02;
    public static final double L4ArmAngle = 4.187;
    public static final double coralArmAngle = 0.681;
    public static final double reefAlgaeAngle = 1.710;
    public static final double bargeAlgaeAngle = 0.29;
    public static final double processorAlgaeAngle = 1.89;
    public static final double climbArmAngle = Math.PI / 2;
    public static final double defaultArmAngle = Math.PI;
  
    // Elevator Positions
    public static final double L2ElevatorPosition = 13.27;
    public static final double L3ElevatorPosition = 27.04;
    public static final double L4ElevatorPosition = 54.4;
    public static final double L2AlgaeElevatorPosition = 14.272;
    public static final double L3AlgaeElevatorPosition = 32.85;
    public static final double coralElevatorPosition = 26.11;
    public static final double bargeElevatorPosition = 54.0;
    public static final double climbElevatorPosition = 0.0;   
    public static final double processorElevatorPosition = 1.0;

    public static final double climberTapValue = 10.0;
    public static final double climberOutPosition = 140.0;
    public static final double climberInPosition = 0.0;
    public static final double climberRatchetOnPosition = 1.0;
    public static final double climberRatchetOffPosition = 0.0;

    public static final double elevatorTolerance = 1.0;
    public static final double armTolerance = 0.1;
    public static final double climberTolerance = 5.0;

    //Reef names named on the game manual pg 24.
    public static final Pose2d aPose2d = new Pose2d(3.108, 4.14, Rotation2d.fromDegrees(0));
    public static final Pose2d bPose2d = new Pose2d(3.108, 3.871, Rotation2d.fromDegrees(0));
    public static final Pose2d cPose2d = new Pose2d(3.656, 2.9, Rotation2d.fromDegrees(60));
    public static final Pose2d dPose2d = new Pose2d(3.925, 2.735, Rotation2d.fromDegrees(60));
    public static final Pose2d ePose2d = new Pose2d(5.051, 2.735, Rotation2d.fromDegrees(120));
    public static final Pose2d fPose2d = new Pose2d(5.35, 2.9, Rotation2d.fromDegrees(120));
    public static final Pose2d gPose2d = new Pose2d(5.888, 3.871, Rotation2d.fromDegrees(180));
    public static final Pose2d hPose2d = new Pose2d(5.888, 4.14, Rotation2d.fromDegrees(180));
    public static final Pose2d iPose2d = new Pose2d(5.35, 5.189, Rotation2d.fromDegrees(-120));
    public static final Pose2d jPose2d = new Pose2d(5.051, 5.34, Rotation2d.fromDegrees(-120));
    public static final Pose2d kPose2d = new Pose2d(3.925, 5.34, Rotation2d.fromDegrees(-60));
    public static final Pose2d lPose2d = new Pose2d(3.656, 5.189, Rotation2d.fromDegrees(-60));
    public static final Pose2d leftCoralStationPose2d = new Pose2d(1.256, 7, Rotation2d.fromDegrees(127.5));
    public static final Pose2d rightCoralStationPose2d = new Pose2d(1.256, 1, Rotation2d.fromDegrees(-127.5));

    public static final Pose2d[] leftPose2ds = {aPose2d, cPose2d, ePose2d, gPose2d, iPose2d, kPose2d};
    public static final Pose2d[] rightPose2ds = {bPose2d, dPose2d, fPose2d, hPose2d, jPose2d, lPose2d};

    public static final PathConstraints pathConstraints = new PathConstraints(3, 3, 540, 720, 12, false);
}
