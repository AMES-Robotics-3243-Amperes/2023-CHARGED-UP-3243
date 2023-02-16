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

 // ££ I still don't understand why putting k in front of variables is the standard in WPILib
public final class Constants {
<<<<<<< Updated upstream
    public static final double kWheelSpeed = 0.5;
    public static final double kGrabberSpeed = 1.0;
    public static final double kPositiveEncoderRotationLimit = 0.5;
    public static final double kNegativeEncoderRotationLimit = 0.5;
    public static final int kMotorId = 8;
    public static final double ktargetAmperage = 2.0;
    public static final int kCurrentLimit = 35;
=======
    public static final int kControllerPort = 0;
    public static final double kWheelSpeed = 0.5;
    public static final double kGrabberSpeed = 0.1;
    public static final double kPositiveEncoderRotationLimit = 50;
    public static final double kNegativeEncoderRotationLimit = 10;
    public static final int kGrabberMotorId = 7;
    public static final int kCompliantMotorIdOne = 60;
    public static final int kCompliantMotorIdTwo = 10;
    public static final double ktargetAmperage = 5.0;
    public static final int kCurrentLimit = 7;
    public static final int kGearRatio = 25;
>>>>>>> Stashed changes
}