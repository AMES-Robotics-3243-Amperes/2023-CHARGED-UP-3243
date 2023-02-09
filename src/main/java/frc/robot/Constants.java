// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

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
    public static final double kWheelSpeed = 0.5;
    public static final double kGrabberSpeed = 1.0;
    public static final double kPositiveEncoderRotationLimit = 0.5;
    public static final double kNegativeEncoderRotationLimit = 0.5;
    public static final int kMotorId = 8;
    public static final double ktargetAmperage = 2.0;
    public static final int kCurrentLimit = 35;
  }
   

  /** ++ constants for WRIST and ARM ---------------------------------------------------- */
  public static final class WristAndArm {
    /**
     * H! This class just holds all the motor ids
     */
    public static class MotorIDs {
      public static final int armPivot = 1000; // H! TODO insert the motor IDs
      public static final int armExtension = 1001;
      public static final int WristPitch = 1002;
      public static final int WristRoll = 1003;
    }

    // H! Holds the data for the positions of stuff in the arm
    // H! TODO Insert the actually correct data
    public static final double minLength = 0.0;
    public static final double maxLength = 1000.0;
    public static final double wristLength = 3.0;
    

    public static final double changeXMultiplier = 0.1;
    public static final double changeYMultiplier = 0.1;
    public static final double changePitchMultiplier = 0.1;
    public static final double changeRollMultiplier = 0.1;
  }


  /** ++ constants for PHOTONVISION ----------------------------------------------------- */
  public static final class PhotonVision {

  }

  /** ++ constants for NEOs ------------------------------------------------------------- */
  public static final class NEOs{
    public static double maxNEORPM = 5500.0;
  }

  /** H! Constants for what automation stuff needs to do -------------------------------- */
  public static final class AutomationConfigure {
    // H! TODO None of these constants are right
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
    public static Pose2d blueScoringPositions[]; // [++ make these final
    public static Pose2d redScoringPositions[];
  }

  // H! This is my fault, so feel free to move it to a better place if need be
  /** H! Enum for the diferent target heights */
  public static enum Target {
    HIGH_TARGET, MID_TARGET, LOW_TARGET
  }
}