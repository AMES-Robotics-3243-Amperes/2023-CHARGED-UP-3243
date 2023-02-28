// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
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

  /**
   * H! Enum for the different target heights
   */
  public static enum Target {
    HIGH_TARGET, MID_TARGET, LOW_TARGET
  }

  // ££ Constants for the Grabber
  public static final class Grabber {
    public static final int kControllerPort = 0;
    public static final double kWheelSpeed = 0.3;
    public static final double kGrabberSpeed = 0.2;
    public static final double kPositiveEncoderRotationLimit = 0.4;
    public static final double kNegativeEncoderRotationLimit = 0.25;
    public static final int kGrabberMotorId = 16;
    public static final int kCompliantMotorIdOne = 1;
    public static final int kCompliantMotorIdTwo = 60;
    public static final int kCurrentLimit = 15;
    public static final int kCurrentTarget = 2;
    public static final int kGearRatio = 25;

    public static final double kPositionP = 7;
    public static final double kPositionI = 0;
    public static final double kPositionD = 0;
    public static final double kPositionFF = 0.03;

    public static final double kCurrentP = 0.03;
    public static final double kCurrentI = 0;
    public static final double kCurrentD = 0;
    public static final double kCurrentFF = 0.01;
  }

  /**
   * ++ constants for DRIVE TRAIN -------------------------------------------
   */
  public static final class DriveTrain {

    // <> constants for individual modules
    public static final class ModuleConstants {

      // <> the maximum wheel speed the modules will turn for
      // <> (in meters per second)
      public static final double kModuleMinSpeed = 0.02;
      // <> pid connects at 0 and 2 pi because rotation is continuous
      public static final double kTurningEncoderPositionPIDMinInput = 0; // <> radians
      public static final double kTurningEncoderPositionPIDMaxInput = Math.PI * 2; // <> radians
      // <> idle modes
      public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
      public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;
      // <> current limits
      public static final int kDrivingMotorCurrentLimit = 50; // <> amps
      public static final int kTurningMotorCurrentLimit = 20; // <> amps

      // <> pidf values / min and max outputs
      public static final class PIDF {

        public static final double kDrivingP = 0.4;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 0;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 0.45;
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
         * <p>
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
          (PhysicalProperties.kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction;
        public static final double kDrivingEncoderVelocityFactor =
          ((PhysicalProperties.kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0;

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI);
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0;
      }
    }

    public static final class DriveConstants {

      // <> if the driving is field relative
      public static final boolean kFieldRelative = true;
      // <> speed damper (flat constant supplied speed is multiplied by)
      public static final double kDrivingSpeedDamper = 12; // <> meters per second
      public static final double kAngularSpeedDamper = 2.6 * Math.PI; // <> radians per second
      // <> max speed
      public static final double kMaxMetersPerSecond = 2.5;
      // <> this should be true
      public static final boolean kGyroReversed = false;

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

        public static final Rotation2d kFrontLeftOffset = Rotation2d.fromRadians(Math.PI * 0.5);
        public static final Rotation2d kFrontRightOffset = Rotation2d.fromRadians(Math.PI);
        public static final Rotation2d kBackLeftOffset = Rotation2d.fromRadians(0);
        public static final Rotation2d kBackRightOffset = Rotation2d.fromRadians(Math.PI * 1.5);
      }

      // <> things involving the physical setup of the chassis
      public static final class ChassisKinematics {

        // <> distance between centers of right and left wheels on robot
        public static final double kRobotWidth = Units.inchesToMeters(27);
        // <> distance between front and back wheels on robot
        public static final double kRobotLength = Units.inchesToMeters(32);

        // <> kinematics (defined with above constants)
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
          new Translation2d(kRobotLength / 2, kRobotWidth / 2), new Translation2d(kRobotLength / 2, -kRobotWidth / 2),
          new Translation2d(-kRobotLength / 2, kRobotWidth / 2),
          new Translation2d(-kRobotLength / 2, -kRobotWidth / 2));
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
        public static final double kMaxAngularAccelerationMetersPerSecond = 1.4 * Math.PI;

        // <> pid constraints for turning
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
          kMaxAngularMetersPerSecond, kMaxAngularAccelerationMetersPerSecond);

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
          DriveConstants.AutoConstants.kMaxAngularMetersPerSecond).setKinematics(ChassisKinematics.kDriveKinematics);

        public static final PIDController movementPidControllerInitial = new PIDController(kMovementPInitial,
          kMovementIInitial, kMovementDInitial);

        public static final PIDController movementPidControllerTrajectoryEnd = new PIDController(
          kMovementPTrajectoryEnd, kMovementITrajectoryEnd, kMovementDTrajectoryEnd);

        // <> leniency for ending SwerveAutoMoveCommands
        public static double angleLeniencyDegrees = 0.8;
        public static double positionLeniencyMeters = 0.035;
      }

      public static final class BalanceConstants {
        // <> the max angle that is considered balanced
        public static final Rotation2d kMaxBalanceLeniency = Rotation2d.fromDegrees(2);

        // <> how long the robot must balance for the command to end
        public static final double kBalanceTimeSeconds = 1;

        // <> pid stuff while balancing
        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0;

        // <> max speed while balancing
        public static final double kMaxBalanceMetersPerSecond = 0.8;
        // <> max accel while balancing
        public static final double kMaxBalanceAccelMetersPerSecond = 0.4;

        public static final TrapezoidProfile.Constraints kPIDControllerConstraints = new TrapezoidProfile.Constraints(
          kMaxBalanceMetersPerSecond, kMaxBalanceAccelMetersPerSecond);

        public static final ProfiledPIDController PIDController = new ProfiledPIDController(kP, kI, kD,
          kPIDControllerConstraints);
      }
    }
  }

  /**
   * ++ constants for JOYSTICKS --------------------------------------------
   */
  public static final class Joysticks {

    public static final int primaryControllerID = 0;
    public static final int secondaryControllerID = 1;

    // ++ OTHER JOYSTICK CONSTANTS --
    public static final double deadZoneSize = 0.12;
    /**
     * ++ lowPassFilterStrength should be between 0 & 1. The closer it is to 1, the smoother acceleration will be.
     */
    public static final double driveLowPassFilterStrength = 0.91;
    public static final double rotationLowPassFilterStrength = 0.2;
    // ++ we probably don't want the speed damcursjdjdjdpers as finals in case we want a fastmode/to change them later
    public static final double driveSpeedDamper = 0.9;
    public static final double rotationDamper = 0.8;

    // ss This is the multiplier for Fast Mode
    // explained in JoyUtil.java
    public static final double fastModeMaxMultiplier = 0.5;
    // :> Slow mode multiplier
    public static final double slowModeMultiplier = 3;

    /**
     * ++ the damper for the D-Pad inputs
     */
    public static final double dPadDamper = 0.7;

    // ++ JOYSTICK CURVE CONSTANTS --
    public static final double aCoeff = 0.7;
    public static final int firstPower = 3;

    public static final int secondPower = 1;
    public static final double bCoeff = (1.0 - aCoeff);


  }

  /**
   * ++ constants for WRIST and ARM ----------------------------------------------------
   */
  public static final class WristAndArm {

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
    public static final int pivotCurrentLimit = 40; // H! This is a temporary change! It was 30 before. // :D hi I just changed this from 30 to 40
    public static final int extensionCurrentLimit = 20; // H! This is a temporary change! It was 20 before. // :D hi I just changed this from 10 to 30
    public static final int pitchCurrentLimit = 20; // H! This is a temporary change! It was 10 before. // :D hi I just changed this from 30 to 15
    public static final int rollCurrentLimit = 5; // H! This is a temporary change! It was 10 before. // :D hi I just changed this from 2 to 5
    public static final int NEO1650CurrentLimitHard = 60; // H! This is a temporary change! It was 40 before. // :D hi I just changed this from 20 to 60
    public static final int NEO550CurrentLimitHard = 40; // H! This is a temporary change! It was 20 before.
    public static final double atSetpointThreshold = 0.005;

    public static final double pivotOutputRange = 0.65;
    public static final double wristRollEncoderSetZeroOffset = 0.163;

    /**
     * H! This class just holds all the motor ids
     */
    public static class MotorIDs {
      public static final int armPivot = 12;
      public static final int armExtension = 7;
      public static final int WristPitchRight = 4;
      public static final int WristPitchLeft = 6;
      public static final int WristRoll = 20;
      public static final int relative = 0;
      public static final int absolute = 0;
    }

    public static class PID {
      public static class Extension {
        public static final double P  = 4.5;// H! 2.0
        public static final double I  = 0.0;
        public static final double D  = 0.0;
        public static final double FF = 0.01;
      }

      public static class Pivot {
        public static final double P  = 5.0;// H! 5.0
        public static final double I  = 0.0;
        public static final double D  = 0.0;
        public static final double FF = 0.1;
      }

      public static class Pitch {
        public static final double P = 0.75;
        public static final double I = 0.0125;
        public static final double D = 0.0;
        public static final double FF = 0.01;
      }

      public static class Roll {
        public static final double P = 1.5;
        public static final double I = 0.0;
        public static final double D = 0.01;
        public static final double FF = 0.1;
      }
    }
  }

  /**
   * ++ constants for PHOTONVISION -----------------------------------------------------
   */
  public static final class PhotonVision {
    // :> This fills me with nothing but dread
    public static final String cameraName1 = "Arducam_OV9281_MMN1";
    public static final String cameraName2 = "Global_Shutter_Camera";
  }

  /**
   * ++ constants for limelight stuff, anything involved with calculations or keys etc
   */
  public static final class Limelight {
    // :> I desperately need to change all of these when they change, these are definitely not the final values
    // ++ ====== actual limelight values ============
    /**
     * angle of the limelight; degrees up from horizontal
     */
    public static final double limelightAngleOffset = 40.7;

    // ++ ======= field/robot measurements ============ (all in meters)
    public static final double pole1Height = .865;
    public static final double pole2Height = 1.17;

    /**
     * ++ this should be the distance of the limelight above the ground
     */
    public static final double lemonHeight = 1.41666667;
    /**
     * this is the difference in height between the shooter and the hub, in meters
     */
    public static final double LemontoPole1Height = pole1Height - lemonHeight;
    public static final double LemontoPole2Height = pole2Height - lemonHeight;

    // +
  }

  /**
   * ++ constants for NEOs -------------------------------------------------------------
   */
  public static final class NEOs {

    public static double maxNEORPM = 5500.0;
  }

  /**
   * H! Constants for what automation stuff needs to do --------------------------------
   */
  public static final class AutomationConfigure {
    // H! TODO: None of these constants are right
    public static final class Cone {
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

    public static final class Cube {
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

  /**
   * ++ field measurement constants
   */ // ---------------------------------------------------
  // :D these constants are very nasty, beware! Also don't reformat this!!! >:(
  public static final class FieldConstants {
    public static final double[] targetPositionsY = {4.983, 4.424, 3.866, 3.307, 2.748, 2.189, 1.630, 1.072, 0.513};
    public static final double[] targetPositionsHiZ = {1.373, 1.0727, 1.373, 1.373, 1.0727, 1.373, 1.373, 1.0727,
                                                       1.373};
    public static final double[] targetPositionsMidZ = {1.068, 0.7687, 1.068, 1.068, 0.7687, 1.068, 1.068, 0.7687,
                                                        1.068};
    public static final double targetPositionsLowZ = 0.203;
    public static Pose2d fieldBottomLeft = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
    public static Pose2d fieldTopRight = new Pose2d(new Translation2d(16.485, 8.102), new Rotation2d(0));

    public static final class Blue {
      // :D position of the robot's chassis:
      public static double scoringChassisPositionX = 1.85;
      public static Pose2d[] scoringPositions = {new Pose2d(Constants.FieldConstants.Blue.scoringChassisPositionX,
        Constants.FieldConstants.targetPositionsY[0], new Rotation2d(Math.PI)), new Pose2d(
        Constants.FieldConstants.Blue.scoringChassisPositionX, Constants.FieldConstants.targetPositionsY[1],
        new Rotation2d(Math.PI)), new Pose2d(Constants.FieldConstants.Blue.scoringChassisPositionX,
        Constants.FieldConstants.targetPositionsY[2], new Rotation2d(Math.PI)), new Pose2d(
        Constants.FieldConstants.Blue.scoringChassisPositionX, Constants.FieldConstants.targetPositionsY[3],
        new Rotation2d(Math.PI)), new Pose2d(Constants.FieldConstants.Blue.scoringChassisPositionX,
        Constants.FieldConstants.targetPositionsY[4], new Rotation2d(Math.PI)), new Pose2d(
        Constants.FieldConstants.Blue.scoringChassisPositionX, Constants.FieldConstants.targetPositionsY[5],
        new Rotation2d(Math.PI)), new Pose2d(Constants.FieldConstants.Blue.scoringChassisPositionX,
        Constants.FieldConstants.targetPositionsY[6], new Rotation2d(Math.PI)), new Pose2d(
        Constants.FieldConstants.Blue.scoringChassisPositionX, Constants.FieldConstants.targetPositionsY[7],
        new Rotation2d(Math.PI)), new Pose2d(Constants.FieldConstants.Blue.scoringChassisPositionX,
        Constants.FieldConstants.targetPositionsY[8], new Rotation2d(Math.PI))};
      public static Pose2d doubleLoadingZone = new Pose2d(new Translation2d(1.4, 6.75), new Rotation2d(Math.PI));
      public static Pose2d singleLoadingZone = new Pose2d(new Translation2d(2.342, 7), new Rotation2d(Math.PI / 2));
      public static Pose2d chargeStationBottomLeft = new Pose2d(new Translation2d(3.276, 1.522), new Rotation2d(0));
      public static Pose2d chargeStationTopRight = new Pose2d(new Translation2d(4.495, 3.979), new Rotation2d(0));
      public static Pose2d dividerTip = new Pose2d(new Translation2d(3.276, 5.487), new Rotation2d(0));

      public static final class autoPositions {
        public static final Pose2d[] lowerPath = {new Pose2d(new Translation2d(2.25, 0.75),
          new Rotation2d(0)), new Pose2d(new Translation2d(6, 0.75), new Rotation2d(0))};
        public static final Pose2d[] upperPath = {new Pose2d(new Translation2d(2.25, 4.6),
          new Rotation2d(0)), new Pose2d(new Translation2d(6, 4.6), new Rotation2d(0))};
      }

      public static final class grabberPositions {
        public static final double highTargetsX = 0.379;
        public static final double middleTargetsX = 0.811;
        public static final double lowTargetsX = 1.185;
        public static final Pose3d[] fieldCenterGamePieces = {new Pose3d(new Translation3d(7.068, 4.577, 0),
          new Rotation3d()), new Pose3d(new Translation3d(7.068, 3.358, 0), new Rotation3d()), new Pose3d(
          new Translation3d(7.068, 2.138, 0), new Rotation3d()), new Pose3d(new Translation3d(7.068, 0.919, 0),
          new Rotation3d())};
      }

    }

    public static final class Red {
      public static double scoringChassisPositionX = 14.697;
      public static Pose2d[] scoringPositions = {new Pose2d(Constants.FieldConstants.Red.scoringChassisPositionX,
        Constants.FieldConstants.targetPositionsY[0], new Rotation2d()), new Pose2d(
        Constants.FieldConstants.Red.scoringChassisPositionX, Constants.FieldConstants.targetPositionsY[1],
        new Rotation2d()), new Pose2d(Constants.FieldConstants.Red.scoringChassisPositionX,
        Constants.FieldConstants.targetPositionsY[2], new Rotation2d()), new Pose2d(
        Constants.FieldConstants.Red.scoringChassisPositionX, Constants.FieldConstants.targetPositionsY[3],
        new Rotation2d()), new Pose2d(Constants.FieldConstants.Red.scoringChassisPositionX,
        Constants.FieldConstants.targetPositionsY[4], new Rotation2d()), new Pose2d(
        Constants.FieldConstants.Red.scoringChassisPositionX, Constants.FieldConstants.targetPositionsY[5],
        new Rotation2d()), new Pose2d(Constants.FieldConstants.Red.scoringChassisPositionX,
        Constants.FieldConstants.targetPositionsY[6], new Rotation2d()), new Pose2d(
        Constants.FieldConstants.Red.scoringChassisPositionX, Constants.FieldConstants.targetPositionsY[7],
        new Rotation2d()), new Pose2d(Constants.FieldConstants.Red.scoringChassisPositionX,
        Constants.FieldConstants.targetPositionsY[8], new Rotation2d())};
      public static Pose2d doubleLoadingZone = new Pose2d(new Translation2d(15, 6.75), new Rotation2d(0));
      public static Pose2d singleLoadingZone = new Pose2d(new Translation2d(14.199, 7), new Rotation2d(Math.PI / 2));
      public static Pose2d chargeStationBottomLeft = new Pose2d(new Translation2d(12.046, 1.522), new Rotation2d(0));
      public static Pose2d chargeStationTopRight = new Pose2d(new Translation2d(13.265, 3.979), new Rotation2d(0));
      public static Pose2d dividerTip = new Pose2d(new Translation2d(13.265, 5.487), new Rotation2d(0));

      public static final class autoPositions {
        public static final Pose2d[] lowerPath = {new Pose2d(new Translation2d(14.25, 0.75),
          new Rotation2d(Math.PI)), new Pose2d(new Translation2d(10.5, 0.75), new Rotation2d(Math.PI))};
        public static final Pose2d[] upperPath = {new Pose2d(new Translation2d(14.25, 4.6),
          new Rotation2d(Math.PI)), new Pose2d(new Translation2d(10.25, 4.6), new Rotation2d(Math.PI))};
      }

      public static final class grabberPositions {
        public static final double highTargetsX = 16.162;
        public static final double middleTargetsX = 15.730;
        public static final double lowTargetsX = 15.350;
        public static final Pose3d[] fieldCenterGamePieces = {new Pose3d(new Translation3d(9.473, 4.577, 0),
          new Rotation3d()), new Pose3d(new Translation3d(9.473, 3.358, 0), new Rotation3d()), new Pose3d(
          new Translation3d(9.473, 2.138, 0), new Rotation3d()), new Pose3d(new Translation3d(9.473, 0.919, 0),
          new Rotation3d())};
      }
    }
  }

  // H! This is my fault, so feel free to move it to a better place if need be
}
