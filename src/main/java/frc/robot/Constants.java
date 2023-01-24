// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPrimaryPort = 0;
    public static final int kDriverControllerSecondaryPort = 0;
  }

  /**
   * H! This class just holds all the motor ids
   */
  public static class MotorIDs {
    public static final int armPivot = 1000; // H! TODO insert the motor IDs
    public static final int armExtension = 1001;
    public static final int WristPitch = 1002;
    public static final int WristRoll = 1003;
  }

  /**
   * H! Holds the data for the positions of stuff in the arm
   */
  public static class ArmData { // H! TODO Insert the actually correct data
    public static final double minLength = 0.0;
    public static final double maxLength = 1000.0;
    public static final double wristLength = 3.0;
  }
}
