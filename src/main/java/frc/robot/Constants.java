// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

// ££ I still don't understand why putting k in front of variables is the standard in WPILib
public final class Constants {

  // ££ Constants for the Grabber
  public static final class Grabber {
    public static final int kControllerPort = 0;
    public static final double kWheelSpeed = 0.2;
    public static final double kGrabberSpeed = 0.05;
    public static final double kPositiveEncoderRotationLimit = 0.41;
    public static final double kNegativeEncoderRotationLimit = 0.3;
    public static final int kGrabberMotorId = 16;
    public static final int kCompliantMotorIdOne = 1;
    public static final int kCompliantMotorIdTwo = 10;
    public static final double ktargetAmperage = 4.0;
    public static final int kCurrentLimit = 7;
    public static final int kGearRatio = 25;
  }

  /** ++ constants for DRIVE TRAIN -------------------------------------------*/
  public static final class DriveTrain {

    // <> constants for individual modules
    public static final class ModuleConstants {

      // <> pidf values / min and max outputs
      public static final class PIDF {

        public static final double kDrivingP = 0.35;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 0;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 0.4;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;
      }

      // <> everything having to do with the physical assembly of the modules
      public static final class PhysicalProperties {

        /**
         * <> direct quote from rev robotics:
         *
         * The MAXSwerve module can be configured with one of three pinion gears:
         * 12T, 13T, or 14T. This changes the drive speed of the module
         * (a pinion gear with more teeth will result in a robot that drives faster).
         */
        public static final int kDrivingMotorPinionTeeth = 13;

        // <> if constructed correctly, all modules' turning encoders will be reversed
        public static final boolean kTurningEncoderInverted = true;

        // <> required for various calculations
        public static final double kWheelDiameterMeters = 0.0762;
      }

      // all the encoder factors
      public static final class EncoderFactors {

        // <> quote from rev robotics:
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
        public static final double kDrivingMotorReduction =
          (45.0 * 22) / (PhysicalProperties.kDrivingMotorPinionTeeth * 15);

        public static final double kDrivingEncoderPositionFactor =
          (PhysicalProperties.kWheelDiameterMeters * Math.PI) /
          kDrivingMotorReduction;
        public static final double kDrivingEncoderVelocityFactor =
          (
            (PhysicalProperties.kWheelDiameterMeters * Math.PI) /
            kDrivingMotorReduction
          ) /
          60.0;

        public static final double kTurningEncoderPositionFactor =
          (2 * Math.PI);
        public static final double kTurningEncoderVelocityFactor =
          (2 * Math.PI) / 60.0;
      }

      // <> the maximum wheel speed the modules will turn for
      // <> (in meters per second)
      public static final double kModuleMinSpeed = 0.02;

      // <> pid connects at 0 and 2 pi because rotation is continuous
      public static final double kTurningEncoderPositionPIDMinInput = 0; // <> radians
      public static final double kTurningEncoderPositionPIDMaxInput =
        Math.PI * 2; // <> radians

      // <> idle modes
      public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
      public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

      // <> current limits
      public static final int kDrivingMotorCurrentLimit = 50; // <> amps
      public static final int kTurningMotorCurrentLimit = 20; // <> amps
    }

    public static final class DriveConstants {

      // <> spark max ids
      public static final class IDs {

        // <> driving ids
        public static final int kFrontLeftDrivingCanId = 3;
        public static final int kRearLeftDrivingCanId = 9;
        public static final int kFrontRightDrivingCanId = 5;
        public static final int kRearRightDrivingCanId = 13;

        // <> turning ids
        public static final int kFrontLeftTurningCanId = 11;
        public static final int kRearLeftTurningCanId = 2;
        public static final int kFrontRightTurningCanId = 8;
        public static final int kRearRightTurningCanId = 15;
      }

      // <> absolute encoder offsets (should be multiples of pi / 2
      // <> if the encoders were zeroed properly in rev client)
      public static final class ModuleOffsets {

        public static final Rotation2d kFrontLeftOffset = Rotation2d.fromRadians(
          Math.PI * 0.5
        );
        public static final Rotation2d kFrontRightOffset = Rotation2d.fromRadians(
          Math.PI
        );
        public static final Rotation2d kBackLeftOffset = Rotation2d.fromRadians(
          0
        );
        public static final Rotation2d kBackRightOffset = Rotation2d.fromRadians(
          Math.PI * 1.5
        );
      }

      // <> things involving the physical setup of the chassis
      public static final class ChassisKinematics {

        // <> distance between centers of right and left wheels on robot
        public static final double kRobotWidth = Units.inchesToMeters(27);
        // <> distance between front and back wheels on robot
        public static final double kRobotLength = Units.inchesToMeters(32);

        // <> kinematics (defined with above constants)
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
          new Translation2d(kRobotLength / 2, kRobotWidth / 2),
          new Translation2d(kRobotLength / 2, -kRobotWidth / 2),
          new Translation2d(-kRobotLength / 2, kRobotWidth / 2),
          new Translation2d(-kRobotLength / 2, -kRobotWidth / 2)
        );
      }

      // <> max temperatures for the drive train motors
      public static final class TempConstants {
        public static final double max550TempCelsius = 100;
        public static final double max1650TempCelsius = 100;
      }

      // <> stuff pertaining to trajectory following,
      // <> not the actual autonomous period
      public static final class AutoConstants {

        // <> max speeds (only for pathfinding, not controlling)
        public static final double kMaxMetersPerSecond = 1.8;
        public static final double kMaxAngularMetersPerSecond = 1 * Math.PI;
        public static final double kMaxAngularAccelerationMetersPerSecond =
          1.4 * Math.PI;

        // <> pid constraints for turning
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
          kMaxAngularMetersPerSecond,
          kMaxAngularAccelerationMetersPerSecond
        );

        // pid controls
        public static final double kMovementPInitial = 0.5;
        public static final double kMovementIInitial = 0;
        public static final double kMovementDInitial = 0;

        public static final double kMovementPTrajectoryEnd = 3;
        public static final double kMovementITrajectoryEnd = 0.5;
        public static final double kMovementDTrajectoryEnd = 0.1;

        public static final double kTurningP = 1;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0.0005;

        // <> config for generated trajectories
        public static final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
          DriveConstants.AutoConstants.kMaxMetersPerSecond,
          DriveConstants.AutoConstants.kMaxAngularMetersPerSecond
        )
          .setKinematics(ChassisKinematics.kDriveKinematics);

        public static final PIDController movementPidControllerInitial = new PIDController(
          kMovementPInitial,
          kMovementIInitial,
          kMovementDInitial
        );

        public static final PIDController movementPidControllerTrajectoryEnd = new PIDController(
          kMovementPTrajectoryEnd,
          kMovementITrajectoryEnd,
          kMovementDTrajectoryEnd
        );

        // <> leniency for ending SwerveAutoMoveCommands
        public static double angleLeniencyDegrees = 0.8;
        public static double positionLeniencyMeters = 0.035;
      }

      // <> if the driving is field relative
      public static final boolean kFieldRelative = true;
      public static final Rotation2d kGyroOffset = Rotation2d.fromDegrees(0);

      // <> speed damper (flat constant supplied speed is multiplied by)
      public static final double kDrivingSpeedDamper = 12; // <> meters per second
      public static final double kAngularSpeedDamper = 2.6 * Math.PI; // <> radians per second

      // <> max speed
      public static final double kMaxMetersPerSecond = 2.5;

      // <> this should be true
      public static final boolean kGyroReversed = true;
    }
  }

  /** ++ constants for JOYSTICKS -------------------------------------------- */
  public static final class Joysticks {

    public static final int primaryControllerID = 0;
    public static final int secondaryControllerID = 1;

    // ++ OTHER JOYSTICK CONSTANTS --
    public static final double deadZoneSize = 0.12;
    /**  ++ lowPassFilterStrength should be between 0 & 1. The closer it is to 1, the smoother acceleration will be. */
    public static final double driveLowPassFilterStrength = 0.91;
    public static final double rotationLowPassFilterStrength = 0.2;
    // ++ we probably don't want the speed damcursjdjdjdpers as finals incase we want a fastmode/to change them later
    public static final double driveSpeedDamper = 0.9;
    public static final double rotationDamper = 0.8;

    // ss This is the multiplier for Fast Mode
    // explained in JoyUtil.java
    public static final double fastModeMaxMultiplier = 0.5;
    // :> Slow mode multiplier
    public static final double slowModeMultiplier = 3; 

    /** ++ the damper for the D-Pad inputs */
    public static final double dPadDamper = 0.7;

    // ++ JOYSTICK CURVE CONSTANTS --
    public static final double aCoeff = 0.7;
    public static final int firstPower = 3;

    public static final int secondPower = 1; 
    public static final double bCoeff = (1.0 - aCoeff); 


  }

  /** ++ constants for WRIST and ARM ---------------------------------------------------- */
  public static final class WristAndArm {
    /**
     * H! This class just holds all the motor ids
     */
    public static class MotorIDs {
      public static final int armPivot = 12;
      public static final int armExtension = 7;
      public static final int WristPitchRight = 4;
      public static final int WristPitchLeft = 6;
      public static final int WristRoll = 20;
    }

    public static class PID {
      public static class Extension {
        public static final double P  = 3.5 * 0;// H! 2.0
        public static final double I  = 0.0;
        public static final double D  = 0.0;
        public static final double FF = 0.01 * 0;
      }

      public static class Pivot {
        public static final double P  = 1.0 * 0;// H! 5.0
        public static final double I  = 0.0;
        public static final double D  = 0.0;
        public static final double FF = 0.1 * 0;
      }

      public static class Pitch {
        public static final double P  = 0.75 * 0;
        public static final double I  = 0.0125 * 0;
        public static final double D  = 0.0;
        public static final double FF = 0.01 * 0;
      }

      public static class Roll {
        public static final double P  = 1.5 * 0;
        public static final double I  = 0.0;
        public static final double D  = 0.01 * 0;
        public static final double FF = 0.1 * 0;
      }
    }

    public static final double extensionEncoderConversionFactor = (Units.inchesToMeters(2.707) * Math.PI) / (36);

    // H! Holds the data for the positions of stuff in the arm
    public static final double minLength = 0.92804 + 0.05;
    public static final double maxLength = 1.5494;
    public static final double wristLength = 0/*Units.inchesToMeters(10)/*0.072327*/;
    

    public static final double changeXMultiplier = 0.10 / 50;
    public static final double changeYMultiplier = 0.10 / 50;
    public static final double changePitchMultiplier = Units.degreesToRadians(15) / 50;
    public static final double changeRollMultiplier = Units.degreesToRadians(15) / 50;

    //&& x and y max and min from pivot in meters
    public static final double maxX = 0.535069 + 1.2192;
    public static final double minX = -0.277731 - 1.2192;
    public static final double maxY = 1.9812 - 0.476364;
    public static final double minY = 0 - 0.476364;

    public static final int pivotCurrentLimit = 30; // H! This is a temporary change! It was 30 before.
    public static final int extensionCurrentLimit = 10; // H! This is a temporary change! It was 20 before.
    public static final int pitchCurrentLimit = 25; // H! This is a temporary change! It was 10 before. // :D hi I just changed this from 30 to 15
    public static final int rollCurrentLimit = 5; // H! This is a temporary change! It was 10 before. // :D hi I just changed this from 2 to 5

    public static final int NEO1650CurrentLimitHard = 40; // H! This is a temporary change! It was 40 before.
    public static final int NEO550CurrentLimitHard = 40; // H! This is a temporary change! It was 20 before.

    public static final double atSetpointThreshold = 0.005;
  }


  /** ++ constants for PHOTONVISION ----------------------------------------------------- */
  public static final class PhotonVision {
    public static final String cameraName1 = "Microsoft_LifeCam_HD-3000";
    public static final String cameraName2 = "Global_Shutter_Camera";
  }

  /** ++ constants for NEOs ------------------------------------------------------------- */
  public static final class NEOs {

    public static double maxNEORPM = 5500.0;
  }

  /** H! Constants for what automation stuff needs to do -------------------------------- */
  public static final class AutomationConfigure {
    // H! TODO: None of these constants are right
    public static final class Cone{
      public static final class HighTarget {
        public static final double armX = 0.0;
        public static final double armY = 0.0;
        public static final double armPitch = 0.0;
        public static final double armRoll = 0.0;
      }
  
      public static final class MidTarget {
        public static final double armX = 0.0;
        public static final double armY = 0.0;
        public static final double armPitch = 0.0;
        public static final double armRoll = 0.0;
      }
  
      public static final class LowTarget {
        public static final double armX = 0.0;
        public static final double armY = 0.0;
        public static final double armPitch = 0.0;
        public static final double armRoll = 0.0;
      }
    }

    public static final class Cube{
      public static final class HighTarget {
        public static final double armX = 0.0;
        public static final double armY = 0.0;
        public static final double armPitch = 0.0;
        public static final double armRoll = 0.0;
      }
  
      public static final class MidTarget {
        public static final double armX = 0.0;
        public static final double armY = 0.0;
        public static final double armPitch = 0.0;
        public static final double armRoll = 0.0;
      }
  
      public static final class LowTarget {
        public static final double armX = 0.0;
        public static final double armY = 0.0;
        public static final double armPitch = 0.0;
        public static final double armRoll = 0.0;
      }
    }

  }

  /** ++ field measurement constants */ // ---------------------------------------------------
  public static final class FieldConstants {
    public static final double targetPositionsY[] = {
      4.983,
      4.424,
      3.866,
      3.307,
      2.748,
      2.189,
      1.630,
      1.072,
      0.513
    };
    public static final class Blue{
      public static final class grabberPositions{
        public static final Pose3d highTargetsXZ = new Pose3d(new Translation3d(0.379,0,0), new Rotation3d());
        public static final Pose3d middleTargetsXZ = new Pose3d(new Translation3d(0.811,0,0), new Rotation3d());
        public static final Pose3d lowTargetsXZ = new Pose3d(new Translation3d(1.185,0,0), new Rotation3d());
        public static final Pose3d fieldCenterGamePieces[] = {
          new Pose3d(new Translation3d(7.068,4.577,0), new Rotation3d()),
          new Pose3d(new Translation3d(7.068,3.358,0), new Rotation3d()),
          new Pose3d(new Translation3d(7.068,2.138,0), new Rotation3d()),
          new Pose3d(new Translation3d(7.068,0.919,0), new Rotation3d())
        };
      }
      
      // :D position of the robot's chassis:
      public static double scoringChassisPositionX = 0;
      public static Pose2d scoringPositions[] = {
        new Pose2d(
          Constants.FieldConstants.Blue.scoringChassisPositionX,
          Constants.FieldConstants.targetPositionsY[0],
          new Rotation2d(Math.PI)
        ),
        new Pose2d(
          Constants.FieldConstants.Blue.scoringChassisPositionX,
          Constants.FieldConstants.targetPositionsY[1],
          new Rotation2d(Math.PI)
        ),
        new Pose2d(
          Constants.FieldConstants.Blue.scoringChassisPositionX,
          Constants.FieldConstants.targetPositionsY[2],
          new Rotation2d(Math.PI)
        ),
        new Pose2d(
          Constants.FieldConstants.Blue.scoringChassisPositionX,
          Constants.FieldConstants.targetPositionsY[3],
          new Rotation2d(Math.PI)
        ),
        new Pose2d(
          Constants.FieldConstants.Blue.scoringChassisPositionX,
          Constants.FieldConstants.targetPositionsY[4],
          new Rotation2d(Math.PI)
        ),
        new Pose2d(
          Constants.FieldConstants.Blue.scoringChassisPositionX,
          Constants.FieldConstants.targetPositionsY[5],
          new Rotation2d(Math.PI)
        ),
        new Pose2d(
          Constants.FieldConstants.Blue.scoringChassisPositionX,
          Constants.FieldConstants.targetPositionsY[6],
          new Rotation2d(Math.PI)
        ),
        new Pose2d(
          Constants.FieldConstants.Blue.scoringChassisPositionX,
          Constants.FieldConstants.targetPositionsY[7],
          new Rotation2d(Math.PI)
        ),
        new Pose2d(
          Constants.FieldConstants.Blue.scoringChassisPositionX,
          Constants.FieldConstants.targetPositionsY[8],
          new Rotation2d(Math.PI)
        )
      };

      public static Pose2d doubleLoadingZone = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
      public static Pose2d singleLoadingZone = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
      public static Pose2d chargeStationBottomLeft = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
      public static Pose2d chargeStationTopRight = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
      public static Pose2d fieldBottomLeft = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
      public static Pose2d fieldTopRight = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
      public static Pose2d dividerTip = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
      
    }
    public static final class Red{
      public static final class grabberPositions{
        public static final Pose3d highTargetsXZ = new Pose3d(new Translation3d(16.162,0,0), new Rotation3d());
        public static final Pose3d middleTargetsXZ = new Pose3d(new Translation3d(15.730,0,0), new Rotation3d());
        public static final Pose3d lowTargetsXZ = new Pose3d(new Translation3d(15.350,0,0), new Rotation3d());
        public static final Pose3d fieldCenterGamePieces[] = {
          new Pose3d(new Translation3d(9.473,4.577,0), new Rotation3d()),
          new Pose3d(new Translation3d(9.473,3.358,0), new Rotation3d()),
          new Pose3d(new Translation3d(9.473,2.138,0), new Rotation3d()),
          new Pose3d(new Translation3d(9.473,0.919,0), new Rotation3d())
        };
      }

      public static double scoringChassisPositionX = 0;
      public static Pose2d scoringPositions[] = {
        new Pose2d(
          Constants.FieldConstants.Red.scoringChassisPositionX,
          Constants.FieldConstants.targetPositionsY[0],
          new Rotation2d()
        ),
        new Pose2d(
          Constants.FieldConstants.Red.scoringChassisPositionX,
          Constants.FieldConstants.targetPositionsY[1],
          new Rotation2d()
        ),
        new Pose2d(
          Constants.FieldConstants.Red.scoringChassisPositionX,
          Constants.FieldConstants.targetPositionsY[2],
          new Rotation2d()
        ),
        new Pose2d(
          Constants.FieldConstants.Red.scoringChassisPositionX,
          Constants.FieldConstants.targetPositionsY[3],
          new Rotation2d()
        ),
        new Pose2d(
          Constants.FieldConstants.Red.scoringChassisPositionX,
          Constants.FieldConstants.targetPositionsY[4],
          new Rotation2d()
        ),
        new Pose2d(
          Constants.FieldConstants.Red.scoringChassisPositionX,
          Constants.FieldConstants.targetPositionsY[5],
          new Rotation2d()
        ),
        new Pose2d(
          Constants.FieldConstants.Red.scoringChassisPositionX,
          Constants.FieldConstants.targetPositionsY[6],
          new Rotation2d()
        ),
        new Pose2d(
          Constants.FieldConstants.Red.scoringChassisPositionX,
          Constants.FieldConstants.targetPositionsY[7],
          new Rotation2d()
        ),
        new Pose2d(
          Constants.FieldConstants.Red.scoringChassisPositionX,
          Constants.FieldConstants.targetPositionsY[8],
          new Rotation2d()
        )
      };

      public static Pose2d doubleLoadingZone = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
      public static Pose2d singleLoadingZone = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
      public static Pose2d chargeStationBottomLeft = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
      public static Pose2d chargeStationTopRight = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
      public static Pose2d fieldBottomLeft = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
      public static Pose2d fieldTopRight = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
      public static Pose2d dividerTip = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
    }
  }

  // H! This is my fault, so feel free to move it to a better place if need be
  /** H! Enum for the diferent target heights */
  public static enum Target {
    HIGH_TARGET, MID_TARGET, LOW_TARGET
  }
}
