// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utility_classes.LegAnklePosition;

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
    // ++ IDs for the motors:
    public static final int grabberOpenerMotorID = 16;
    public static final int wheelMotorOneID = 1;
    public static final int wheelMotorTwoID = 14;

    // // ++ motor spin speeds
    // public static final double wheelMotorSpeed = 0.2;
    // public static final double openCloseSpeed = 0.05;

    // ++ grabber maximum/minimum positions
    public static final double maximumGrabberLimit = 0.6;
    public static final double minimumGrabberLimit = 0.4;

    // :D grabber open/close setpoints
    public static final double openGrabberSetpoint = 0.4;
    public static final double closedGrabberSetpoint = 0.6;

    // ++ current limits
    public static final int hardOpenerMotorCurrentLimit = 30;
    public static final int hardWheelMotorCurrentLimit = 30;
    public static final int softOpenerMotorCurrentLimit = 15; // ++ this will have to be changed to adequately
    // compress game pieces
    public static final int softWheelMotorCurrentLimit = 7; // ++ will have to be experimentally tuned

    // ++ gear ratios
    public static final double grabberMotorOpenerGearRatio = (1 / 1); // ++ find actual values! // :D by the way, the
    // absolute encoder is on the ouput shaft of the grabber, so the conversion is 1:1
    public static final double wheelMotorGearRatio = (1 / 1);

    // ++ PID values
    public static final double openerMotorPGain = 2;
    public static final double openerMotorIGain = 0.0;
    public static final double openerMotorDGain = 0.0;

    public static final double wheelMotorPGain = 0.00004;
    public static final double wheelMotorIGain = 0.000000;
    public static final double wheelMotorDGain = 0.00000;
  
    // ++ wheel spin speed constants
    public static final double openGrabberToWidthSetpoint = 0.44;
    public static final double intakeWheelSpeed = 10000;
    public static final double ambientWheelSpeed = 1000;
    public static final double ejectWheelSpeed = -1000000000000000.0;
  }

  /**
   * ++ constants for DRIVE TRAIN -------------------------------------------
   */
  public static final class DriveTrain {

    // <> constants for individual modules
    public static final class ModuleConstants {

      // <> the maximum robot speed the modules will turn for
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

        public static final double kTurningP = 0.5;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0.3;
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

      // <> if controls are field relative
      public static final boolean kDrivingFieldRelative = true;
      public static final boolean kTurningFieldRelative = true;

      // <> speed damper (flat constant supplied speed is multiplied by)
      public static final double kDrivingSpeedDamper = 1; // <> meters per second
      public static final double kAngularSpeedDamper = 1 * Math.PI; // <> radians per second
      // <> max speed
      public static final double kMaxMetersPerSecond = 2;
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

      public static final class FieldRelativeTurningConstants {
        public static final double kP = 4;
        public static final double kI = 0;
        public static final double kD = 0;

        // <> these values aren't exact but are pretty close
        public static final double kMaxAngularAccelerationDegreesPerSecond = 350;
        public static final double kMaxAngularVelocityDegreesPerSecond = 325;

        public static final TrapezoidProfile.Constraints kConstraints = new TrapezoidProfile.Constraints(
          kMaxAngularVelocityDegreesPerSecond, kMaxAngularAccelerationDegreesPerSecond);

        public static final ProfiledPIDController kPidController = new ProfiledPIDController(kP, kI, kD, kConstraints);
      }

      // <> stuff pertaining to auto driving
      public static final class AutoConstants {
        public static final double kP = 0.2;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kGoalVelocityMagnitudeScalar = 3.2;

        public static final double kMaxVelocityMetersPerSecond = 0.6;
        public static final double kMaxAccelerationMetersPerSecondSq = 2.5;
        public static final double kMaxJerkMetersPerSecondCubed = 3.5;

        public static final TrapezoidProfile.Constraints kConstraints = new TrapezoidProfile.Constraints(
          kMaxAccelerationMetersPerSecondSq, kMaxJerkMetersPerSecondCubed);

        public static final double kMaxMetersFromGoal = 0.06;
        public static final Rotation2d kMaxRotationFromGoal = Rotation2d.fromDegrees(2);

        public static final double kMaxLenientMetersFromGoal = 0.18;
        public static final Rotation2d kMaxLenientRotationFromGoal = Rotation2d.fromDegrees(6);
      }

      public static final class BalanceConstants {
        // <> the max angle that is considered balanced
        public static final Rotation2d kMaxBalanceLeniency = Rotation2d.fromDegrees(1.5);

        // <> how long the robot must balance for the command to end
        public static final double kBalanceTimeSeconds = 1.5;

        // <> pid stuff while balancing
        public static final double kP = 0.0135;
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
  public static class JoyUtilConstants {
    // <> size of controller deadzone
    public static final double kDeadzone = 0.12;
    // <> max amount controller output can change per second
    public static final double kRateLimitLeft = 4;
    public static final double kRateLimitRight = 3.4;
    // <> curve stuff
    public static final int exponent1 = 1;
    public static final int exponent2 = 3;
    public static final double coeff1 = 0.4;
    public static final double coeff2 = 0.6;
    // <> fast and slow mode
    public static final double leftTriggerSpeedMultiplier = 1.5;
    public static final double rightTriggerSpeedMultiplier = 0.4;
    // <> ports
    public static int primaryControllerID = 0;
    public static int secondaryControllerID = 1;
  }

  /**
   * ++ constants for WRIST and ARM ----------------------------------------------------
   */
  public static final class WristAndArm {

    public static final double movementTimeoutDuration = 5.0;

    public static final Translation3d pivotOffset = new Translation3d(Units.inchesToMeters(32) / 2 - .27773100, 0,
      .44255 + Units.inchesToMeters(2.662500 / 2));

    public static final double extensionEncoderConversionFactor = (Units.inchesToMeters(2.707) * Math.PI) / (36);
    public static final double pitchEncoderConversionFactor = 1/60;
    public static final double pivotEncoderConversionFactor = 42/60;
    

    // ++ pitch limits
    // public static final double maxPitchPos;
    // public static final double minPitchPos;

    // ++ ------- pickup points -------
    public static final double pivotPickupPos = Limits.pivotMin;
    public static final double extensionPickupPos = 0.99;
    public static final double rollPickupPos = 0.55;
    public static final double pitchPickupPos = 0.0;


    public static final double pivotEncoderOffset = 0.28125; // TODO :D check this value
    public static final double wristLength = Units.inchesToMeters(5)/*0.072327*/;
    public static final double changeXMultiplier = 0.10 / 50;
    public static final double changeYMultiplier = 0.10 / 50;
    public static final double changePitchMultiplier = Units.degreesToRadians(15) / 50;
    public static final double changeRollMultiplier = Units.degreesToRadians(15) / 50;

    
    public static final int pivotCurrentLimit = 39; // H! This is a temporary change! It was 30 before. // :D hi I
    // just changed this from 30 to 40
    public static final int extensionCurrentLimit = 15; // H! This is a temporary change! It was 20 before. // :D hi
    // I just changed this from 10 to 30
    public static final int pitchCurrentLimit = 29; // H! This is a temporary change! It was 10 before. // :D hi I
    // just changed this from 30 to 15
    public static final int rollCurrentLimit = 20; // H! This is a temporary change! It was 10 before. // :D hi I just
    // changed this from 5 to 10
    public static final int NEO1650CurrentLimitHard = 40; // H! This is a temporary change! It was 40 before. // :D
    // hi I just changed this from 20 to 60
    public static final int NEO550CurrentLimitHard = 30; // H! This is a temporary change! It was 20 before.
    public static final double atSetpointThreshold = 0.02;

    public static final double pivotOutputRange = 0.15;
    public static final double pitchOutputRange = 0.3;
    public static final double extensionOutputRange = 0.5;
    public static final double rollOutputRange = 0.3;
    public static final double wristRollEncoderSetZeroOffset = 0.89855;
    public static final double wristPitchEncoderSetZeroOffset = 0.2574555;// :D prev value: 0.866; I changed it because it was flipped 180 degrees from what the standard on the pivot is
    public static final double wristPivotEncoderSetZeroOffset = 0.196875;

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

    public static class StartingPosition {
      public static final double pivot = 0.308;
      public static final double extension = 1.09;
      public static final double pitch = 0.463;
      public static final Double roll = 0.5;
    }

    public static class StartingSetpoints { // :D DONE: make these values not use inverse kinematics, it'll be easier to visually see and check/adjust
      public static final double pivot = 0.308;
      public static final double extension = 1.09;
      public static final double pitch = 0.463;
      public static final Double roll = null;
    }

    public static class PickupSetpoints {
      public static final double pivot = 0.53;
      public static final double extension = 1.09;
      public static final double pitch = 0.5;
      public static final Double roll = null;
    }

    public static class PickupSetpointsLOW { // :D TODO: get the actual values
      public static final double pivot = 0.565;
      public static final double extension = 1.09;
      public static final double pitch = 0.7;
      public static final Double roll = null;
    }

    public static class DoubleLoadSetpoints {
      public static final double pivot = 0.37;
      public static final double extension = 1.09;
      public static final double pitch = 0.6;
      public static final Double roll = null;
    }

    public static class PlacementSetpoints {
      public static final class High {
        public static final double pivot = 0.17;
        public static final double extension = WristAndArm.Limits.extensionMax-0.1;
        public static final double pitch = 0.722;
        public static final Double roll = null;
      }

      public static final class Middle {
        public static final double pivot = 0.151;
        public static final double extension = 1.09;
        public static final double pitch = 0.722;
        public static final Double roll = null;
      }

      public static final class Low {
        public static final double pivot = 0.205;
        public static final double extension = 1.09;
        public static final double pitch = 0.817;
        public static final Double roll = null;
      }
    }

    public static class PID {
      public static class Extension {
        public static final double P = 5.0;// H! 2.0
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double FF = 0.01;
      }

      public static class Pivot {
        public static final double P = 13.0 * 0.5;
        public static final double I = 0.00004*0; // 0
        public static final double D = 5*0; // 1
        public static final double FF = 0.1;
      }

      public static class Pitch {
        public static final double P = 6;
        public static final double I = 0.001;
        public static final double D = 0.0;
        public static final double FF = 0.01;
      }

      public static class Roll {
        public static final double P = 0.95;
        public static final double I = 0.0003;
        public static final double D = 0.0;
        public static final double FF = 0.1;
      }
    }

    /** All the maximums, minimums, and other values associated with stoping the legAnkle from
     * destroying itself, or breaking FRC rules.
     * H!
     */
    public static final class Limits {
      //&& x and y max and min from pivot in meters (H! to avoid breaking the height limits and distance from frame perimeter limit, that is)
      // H! Note this currently are only applied when using IK
      public static final double maxX = 0.535069 + 1.2192;
      public static final double minX = -0.277731 - 1.2192;
      public static final double maxY = 1.9812 - 0.476364;
      public static final double minY = 0 - 0.476364;

      // H! Extension limits
      public static final double extensionMin = 0.92804 /*+ 0.05*/;
      public static final double extensionMax = extensionMin + Units.inchesToMeters(30);// :D approx. equals: 1.69; prev value: 1.5494
      // ++ pivot limits
      public static final double pivotMax = 0.58;
      public static final double pivotMin = 0.075;
      // H! Pitch limitssssessss
      public static final double pitchMax = 0.95;
      public static final double pitchMin = 0.05;
      // H! Roll lime-ets
      public static final double rollMax = 1.0;
      public static final double rollMin = 0.5;
    }
  }

  /**
   * ++ constants for PHOTONVISION -----------------------------------------------------
   */
  public static final class PhotonVision {
    // :> This fills me with nothing but dread
    public static final String cameraName1 = "Backward_Global_Camera";
    public static final String cameraName2 = "Forward_Global_Camera";
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
  public static final class AutomationConfiguration {
    /**The position the leg ankle will try to move to at the beginning of the match
     * H!
     */
    public static final LegAnklePosition initialLegAnklePositonMovement = new LegAnklePosition( // :D DONE: make this not be IK and also consolodate with the other initial position stuff
      WristAndArm.StartingSetpoints.extension,
      WristAndArm.StartingSetpoints.pivot,
      WristAndArm.StartingSetpoints.pitch,
      0.5
    );

    /**The position of the leg ankle to easily pick up game pieces from the double loading station
     * H!
     */
    public static final LegAnklePosition legAnkleDoubleLoadingPosition = new LegAnklePosition( // :D TODO: find these actual values
      WristAndArm.DoubleLoadSetpoints.extension, 
      WristAndArm.DoubleLoadSetpoints.pivot, 
      WristAndArm.DoubleLoadSetpoints.pitch, 
      WristAndArm.DoubleLoadSetpoints.roll
    );

    /**The position of the leg ankle to easily pick up game pieces
     * H!
     */
    public static final LegAnklePosition legAnklePickupPositionCrane = new LegAnklePosition(
      WristAndArm.PickupSetpoints.extension, 
      WristAndArm.PickupSetpoints.pivot, 
      WristAndArm.PickupSetpoints.pitch, 
      WristAndArm.PickupSetpoints.roll
    );

    public static final LegAnklePosition legAnklePickupPositionSweep = new LegAnklePosition(
      WristAndArm.PickupSetpointsLOW.extension, 
      WristAndArm.PickupSetpointsLOW.pivot, 
      WristAndArm.PickupSetpointsLOW.pitch, 
      WristAndArm.PickupSetpointsLOW.roll
    );

    // :D I capitalized the HIGH, MIDDLE, and LOW so that it would stand out more, if you don't really like it, feel free to change
    public static final LegAnklePosition legAnklePlacementPositionHIGH = new LegAnklePosition(
      WristAndArm.PlacementSetpoints.High.extension,
      WristAndArm.PlacementSetpoints.High.pivot,
      WristAndArm.PlacementSetpoints.High.pitch, 
      WristAndArm.PlacementSetpoints.High.roll
    );

    public static final LegAnklePosition legAnklePlacementPositionMIDDLE = new LegAnklePosition(
      WristAndArm.PlacementSetpoints.Middle.extension,
      WristAndArm.PlacementSetpoints.Middle.pivot,
      WristAndArm.PlacementSetpoints.Middle.pitch, 
      WristAndArm.PlacementSetpoints.Middle.roll
    );

    public static final LegAnklePosition legAnklePlacementPositionLOW = new LegAnklePosition(
      WristAndArm.PlacementSetpoints.Low.extension,
      WristAndArm.PlacementSetpoints.Low.pivot,
      WristAndArm.PlacementSetpoints.Low.pitch, 
      WristAndArm.PlacementSetpoints.Low.roll
    );

    public static final LegAnklePosition legAnkleNeutralPosition = new LegAnklePosition(
      WristAndArm.StartingSetpoints.extension,
      WristAndArm.StartingSetpoints.pivot,
      WristAndArm.StartingSetpoints.pitch,
      WristAndArm.StartingSetpoints.roll
    );

    // H! TODO: None of these constants are right // :D can this code be deleted?
    /*public static final class Cone {
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
    }*/

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
      public static double scoringChassisPositionX = 1.85 + 0.05;
      public static Pose2d[] scoringPositions = {new Pose2d(Constants.FieldConstants.Blue.scoringChassisPositionX,
        Constants.FieldConstants.targetPositionsY[0], new Rotation2d()), new Pose2d(
        Constants.FieldConstants.Blue.scoringChassisPositionX, Constants.FieldConstants.targetPositionsY[1],
        new Rotation2d()), new Pose2d(Constants.FieldConstants.Blue.scoringChassisPositionX,
        Constants.FieldConstants.targetPositionsY[2], new Rotation2d()), new Pose2d(
        Constants.FieldConstants.Blue.scoringChassisPositionX, Constants.FieldConstants.targetPositionsY[3],
        new Rotation2d()), new Pose2d(Constants.FieldConstants.Blue.scoringChassisPositionX,
        Constants.FieldConstants.targetPositionsY[4], new Rotation2d()), new Pose2d(
        Constants.FieldConstants.Blue.scoringChassisPositionX, Constants.FieldConstants.targetPositionsY[5],
        new Rotation2d()), new Pose2d(Constants.FieldConstants.Blue.scoringChassisPositionX,
        Constants.FieldConstants.targetPositionsY[6], new Rotation2d()), new Pose2d(
        Constants.FieldConstants.Blue.scoringChassisPositionX, Constants.FieldConstants.targetPositionsY[7],
        new Rotation2d()), new Pose2d(Constants.FieldConstants.Blue.scoringChassisPositionX,
        Constants.FieldConstants.targetPositionsY[8], new Rotation2d())};

      public static Pose2d doubleLoadingZone = new Pose2d(new Translation2d(1.4, 6.75), new Rotation2d(Math.PI));
      public static Pose2d singleLoadingZone = new Pose2d(new Translation2d(2.342, 7), new Rotation2d(Math.PI / 2));
      public static Pose2d chargeStationBottomLeft = new Pose2d(new Translation2d(3.276, 1.522), new Rotation2d(0));
      public static Pose2d chargeStationTopRight = new Pose2d(new Translation2d(4.495, 3.979), new Rotation2d(0));
      public static Pose2d dividerTip = new Pose2d(new Translation2d(3.276, 5.487), new Rotation2d(0));

      public static final class autoPositions {
        public static final Pose2d[] lowerPath = {new Pose2d(new Translation2d(2.25, 0.75),
          new Rotation2d(0)), new Pose2d(new Translation2d(6 - 3, 0.75), new Rotation2d(0))}; // TODO the -3
        public static final Pose2d[] upperPath = {new Pose2d(new Translation2d(2.25, 4.6),
          new Rotation2d(0)), new Pose2d(new Translation2d(6 - 3, 4.6), new Rotation2d(0))}; // TODO this too
      }

      public static final class grabberPositions {
        public static final double highTargetsX = 0.379;
        public static final double middleTargetsX = 0.811;
        public static final double lowTargetsX = 1.185;
        public static final double fieldCenterGamePiecesX = 7.068 - 3; // :D for testing purposes, due to limited space. TODO: remove the "- 1"

        public static final Pose3d[] fieldCenterGamePieces = {new Pose3d(new Translation3d(fieldCenterGamePiecesX, 4.577, 0),
          new Rotation3d()), new Pose3d(new Translation3d(fieldCenterGamePiecesX, 3.358, 0), new Rotation3d()), new Pose3d(
          new Translation3d(fieldCenterGamePiecesX, 2.138, 0), new Rotation3d()), new Pose3d(new Translation3d(fieldCenterGamePiecesX, 0.919, 0),
          new Rotation3d())};
      }

    }

    public static final class Red {
      public static double scoringChassisPositionX = 14.697 - 0.05;
      public static Pose2d[] scoringPositions = {new Pose2d(Constants.FieldConstants.Red.scoringChassisPositionX,
        Constants.FieldConstants.targetPositionsY[0], new Rotation2d(Math.PI)), new Pose2d(
        Constants.FieldConstants.Red.scoringChassisPositionX, Constants.FieldConstants.targetPositionsY[1],
        new Rotation2d(Math.PI)), new Pose2d(Constants.FieldConstants.Red.scoringChassisPositionX,
        Constants.FieldConstants.targetPositionsY[2], new Rotation2d(Math.PI)), new Pose2d(
        Constants.FieldConstants.Red.scoringChassisPositionX, Constants.FieldConstants.targetPositionsY[3],
        new Rotation2d(Math.PI)), new Pose2d(Constants.FieldConstants.Red.scoringChassisPositionX,
        Constants.FieldConstants.targetPositionsY[4], new Rotation2d(Math.PI)), new Pose2d(
        Constants.FieldConstants.Red.scoringChassisPositionX, Constants.FieldConstants.targetPositionsY[5],
        new Rotation2d(Math.PI)), new Pose2d(Constants.FieldConstants.Red.scoringChassisPositionX,
        Constants.FieldConstants.targetPositionsY[6], new Rotation2d(Math.PI)), new Pose2d(
        Constants.FieldConstants.Red.scoringChassisPositionX, Constants.FieldConstants.targetPositionsY[7],
        new Rotation2d(Math.PI)), new Pose2d(Constants.FieldConstants.Red.scoringChassisPositionX,
        Constants.FieldConstants.targetPositionsY[8], new Rotation2d(Math.PI))};
      public static Pose2d doubleLoadingZone = new Pose2d(new Translation2d(15, 6.75), new Rotation2d(0));
      public static Pose2d singleLoadingZone = new Pose2d(new Translation2d(14.199, 7), new Rotation2d(Math.PI / 2));
      public static Pose2d chargeStationBottomLeft = new Pose2d(new Translation2d(12.046, 1.522), new Rotation2d(0));
      public static Pose2d chargeStationTopRight = new Pose2d(new Translation2d(13.265, 3.979), new Rotation2d(0));
      public static Pose2d dividerTip = new Pose2d(new Translation2d(13.265, 5.487), new Rotation2d(0));

      public static final class autoPositions {
        public static final Pose2d[] lowerPath = {new Pose2d(new Translation2d(14.25, 0.75), // TODO remove the 3
          new Rotation2d(Math.PI)), new Pose2d(new Translation2d(10.5 + 3, 0.75), new Rotation2d(Math.PI))};
        public static final Pose2d[] upperPath = {new Pose2d(new Translation2d(14.25, 4.6), // TODO this too
          new Rotation2d(Math.PI)), new Pose2d(new Translation2d(10.25 + 3, 4.6), new Rotation2d(Math.PI))};
      }

      public static final class grabberPositions {
        public static final double highTargetsX = 16.162;
        public static final double middleTargetsX = 15.730;
        public static final double lowTargetsX = 15.350;
        public static final double fieldCenterGamePiecesX = 9.473 + 3;

        public static final Pose3d[] fieldCenterGamePieces = {new Pose3d(new Translation3d(fieldCenterGamePiecesX, 4.577, 0),
          new Rotation3d()), new Pose3d(new Translation3d(fieldCenterGamePiecesX, 3.358, 0), new Rotation3d()), new Pose3d(
          new Translation3d(fieldCenterGamePiecesX, 2.138, 0), new Rotation3d()), new Pose3d(new Translation3d(fieldCenterGamePiecesX, 0.919, 0),
          new Rotation3d())};
      }
    }
  }

  // H! This is my fault, so feel free to move it to a better place if need be


  public static final class LED {
    //&& These are the constants for the LEDSubsystem.
    //&& They can be moved to JSON later, I just don't know how to do that yet.

    //&& TODO: Set the correct value for the PWM port, because I don't know if 0 is correct.
    public static final int pwmPort = 0;
  }
}
