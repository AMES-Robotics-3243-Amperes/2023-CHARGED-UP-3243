// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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

  /** ++ constants for DRIVE TRAIN -------------------------------------------*/
  public static final class DriveTrain {
    public static final class DriveConstants {
      // <> if the driving is field relative
      public static final boolean fieldRelative = true;

      // <> max ALLOWED speeds
      public static final double kMaxSpeedMetersPerSecond = 4.8;
      public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
  
      // <> distance between centers of right and left wheels on robot
      public static final double kRobotWidth = Units.inchesToMeters(26.5);
      // <> distance between front and back wheels on robot
      public static final double kRobotLength = Units.inchesToMeters(26.5);

      // <> configure kinematics
      public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
          new Translation2d(kRobotLength / 2, kRobotWidth / 2),
          new Translation2d(kRobotLength / 2, -kRobotWidth / 2),
          new Translation2d(-kRobotLength / 2, kRobotWidth / 2),
          new Translation2d(-kRobotLength / 2, -kRobotWidth / 2));
  
      // <> angular offsets of the modules
      public static final double kFrontLeftChassisAngularOffset = 0;
      public static final double kFrontRightChassisAngularOffset = 0;
      public static final double kBackLeftChassisAngularOffset = 0;
      public static final double kBackRightChassisAngularOffset = 0;
  
      // <> spark max ids
      public static final int kFrontLeftDrivingCanId = 11;
      public static final int kRearLeftDrivingCanId = 13;
      public static final int kFrontRightDrivingCanId = 15;
      public static final int kRearRightDrivingCanId = 17;
  
      public static final int kFrontLeftTurningCanId = 10;
      public static final int kRearLeftTurningCanId = 12;
      public static final int kFrontRightTurningCanId = 14;
      public static final int kRearRightTurningCanId = 16;
  
      // <> if the gyro is reversed
      public static final boolean kGyroReversed = false;
    }
    
    public static final class ModuleConstants {
      /** 
       * <> direct quote from rev robotis:
       * 
       * The MAXSwerve module can be configured with one of three pinion gears:
       * 12T, 13T, or 14T. This changes the drive speed of the module
       * (a pinion gear with more teeth will result in a robot that drives faster).
      */
      public static final int kDrivingMotorPinionTeeth = 14;
  
      // <> if constructed correctly, all modules' turning encoders will be reversed
      public static final boolean kTurningEncoderInverted = true;
  
      // <> required for various calculations
      public static final double kWheelDiameterMeters = 0.0762;
      public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

      // <> quote from revrobotics:
      // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
      public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
  
      public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
          / kDrivingMotorReduction; // meters
      public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
          / kDrivingMotorReduction) / 60.0; // meters per second
  
      public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // <> radians
      public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // <> radians per second
  
      public static final double kTurningEncoderPositionPIDMinInput = 0; // <> radians
      public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // <> radians
  
      // <> pidf stuff
      public static final double kDrivingP = 0.04;
      public static final double kDrivingI = 0;
      public static final double kDrivingD = 0;
      public static final double kDrivingFF = 0;
      public static final double kDrivingMinOutput = -1;
      public static final double kDrivingMaxOutput = 1;
  
      public static final double kTurningP = 1;
      public static final double kTurningI = 0;
      public static final double kTurningD = 0;
      public static final double kTurningFF = 0;
      public static final double kTurningMinOutput = -1;
      public static final double kTurningMaxOutput = 1;
  
      // <> idle modes
      public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
      public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;
  
      // <> current limits
      public static final int kDrivingMotorCurrentLimit = 50; // <> amps
      public static final int kTurningMotorCurrentLimit = 20; // <> amps
    }
  }



  /** ++ constants for JOYSTICKS -------------------------------------------- */
  public static final class Joysticks {
    public static final int primaryControllerID = 0;
    public static final int secondaryControllerID = 1;

    // ++ OTHER JOYSTICK CONSTANTS --
    public static final double deadZoneSize = 0.15;
    /**  ++ lowPassFilterStrength should be between 0 & 1. The closer it is to 1, the smoother acceleration will be. */
    public static final double driveLowPassFilterStrength = 0.91;
    public static final double rotationLowPassFilterStrength = 0.2;
    // ++ we probably don't want the speed damcursjdjdjdpers as finals incase we want a fastmode/to change them later 
    public static final double driveSpeedDamper = 0.65; 
    public static final double rotationDamper = 8.0; 

    // ss This is the multiplier for Fast Mode
    // explained in JoyUtil.java
    public static final double fastModeMaxMultiplier = 0.5;

    /** ++ the damper for the D-Pad inputs */
    public static final double dPadDamper = 0.7;


    // ++ JOYSTICK CURVE CONSTANTS --
    public static final double aCoeff = 0.7;
    public static final int firstPower = 3;

    public static final int secondPower = 1; 
    public static final double bCoeff = (1.0 - aCoeff); 


  }

  /** ++ constants for GRABBER ---------------------------------------------------------- */
  public static final class Grabber {

  }
   

  /** ++ constants for WRIST and ARM ---------------------------------------------------- */
  public static final class WristAndArm {

  }


  /** ++ constants for PHOTONVISION ----------------------------------------------------- */
  public static final class PhotonVision {

  }

  /** ++ constants for NEOs ------------------------------------------------------------- */
  public static final class NEOs{
    public static double maxNEORPM = 5500.0;
  }


}