// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**;
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

 // ££ I still don't understand why putting k in front of variables is the standard in WPILib
public final class Constants {
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