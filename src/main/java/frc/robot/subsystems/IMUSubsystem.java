// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * <> A wrapper for a {@link AHRS} with a bunch of useful stuff
 */
public class IMUSubsystem extends SubsystemBase {
  private AHRS m_imu = new AHRS();

  /** Creates a new IMUSubsystem. */
  public IMUSubsystem() {}

  @Override
  public void periodic() {}

  /**
   * <>
   *
   * @return the yaw value that the gyro reads
   */
  public Rotation2d getYaw() {
    Rotation2d raw_angle = m_imu.getRotation2d();

    return Constants.DriveTrain.DriveConstants.kGyroReversed ? raw_angle.times(-1) : raw_angle;
  }

  /**
   * <>
   *
   * @return the displacement from starting position that the imu reads
   */
  public Translation2d getDisplacement() {
    return new Translation2d(m_imu.getDisplacementX(), m_imu.getDisplacementY());
  }

  /**
   * <>
   *
   * @return robot's turn rate in degrees per second
   */
  public double getTurnRate() {
    return m_imu.getRate() * (Constants.DriveTrain.DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * <>
   *
   * @return the upwards angle of the robot
   */
  public double getInclinationRadians() {
    double tanRoll = Math.tan(Math.toRadians(m_imu.getRoll()));
    double tanPitch = Math.tan(Math.toRadians(m_imu.getPitch()));

    return Math.atan(Math.sqrt(tanRoll * tanRoll + tanPitch * tanPitch));
  }
}
