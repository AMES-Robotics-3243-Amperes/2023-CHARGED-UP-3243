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
  private final AHRS m_imu = new AHRS();

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
  public Rotation2d getInclinationRadians() {
    double tanRoll = Math.tan(Math.toRadians(m_imu.getRoll()));
    double tanPitch = Math.tan(Math.toRadians(m_imu.getPitch()));

    return Rotation2d.fromRadians(Math.atan(Math.sqrt(tanRoll * tanRoll + tanPitch * tanPitch)));
  }

  /**
   * <>
   *
   * @apiNote this assumes that the robot would only experience a change in
   * pitch if it were on the charge station and facing straight forward:
   * it must be parallel with the gyro's idea of the game field
   *
   * @return the calculated angle of the charge station (positive values mean that
   * turning to face directly 0 degrees and driving forwards will go "up" the charge station)
   */
  public Rotation2d getChargeLevel() {
    // this is the raw angle we use to determine the magnitude of the leaning,
    // but we need to determine what way it's leaning another way
    Rotation2d upwardRotation = getInclinationRadians();

    // extract the signs of the cos and sin of a shifted angle
    boolean cosPos = Math.cos(getYaw().getRadians() - Math.PI / 4) >= 0;
    boolean sinPos = Math.sin(getYaw().getRadians() - Math.PI / 4) >= 0;

    // and use their signs to figure out what way we're facing
    // and how to find the sign of the angle
    if (cosPos && sinPos) {
      // we're facing forwards
      return m_imu.getPitch() >= 0 ? upwardRotation : upwardRotation.times(-1);
    } else if (!cosPos && !sinPos) {
      // facing backwards
      return m_imu.getPitch() < 0 ? upwardRotation : upwardRotation.times(-1);
    } else if (!cosPos) {
      // facing left
      return m_imu.getRoll() >= 0 ? upwardRotation.times(-1) : upwardRotation;
    } else {
      // facing right
      return m_imu.getRoll() < 0 ? upwardRotation.times(-1) : upwardRotation;
    }
  }
}
