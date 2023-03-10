// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrain.DriveConstants;
import frc.robot.FieldPosManager;

public class DriveSubsystem extends SubsystemBase {

  // <> create swerve modules
  private final SwerveModule m_frontLeft = new SwerveModule(DriveConstants.IDs.kFrontLeftDrivingCanId,
    DriveConstants.IDs.kFrontLeftTurningCanId, DriveConstants.ModuleOffsets.kFrontLeftOffset);

  private final SwerveModule m_frontRight = new SwerveModule(DriveConstants.IDs.kFrontRightDrivingCanId,
    DriveConstants.IDs.kFrontRightTurningCanId, DriveConstants.ModuleOffsets.kFrontRightOffset);

  private final SwerveModule m_rearLeft = new SwerveModule(DriveConstants.IDs.kRearLeftDrivingCanId,
    DriveConstants.IDs.kRearLeftTurningCanId, DriveConstants.ModuleOffsets.kBackLeftOffset);

  private final SwerveModule m_rearRight = new SwerveModule(DriveConstants.IDs.kRearRightDrivingCanId,
    DriveConstants.IDs.kRearRightTurningCanId, DriveConstants.ModuleOffsets.kBackRightOffset);

  // <> gyro
  private final IMUSubsystem m_imuSubsystem = new IMUSubsystem();

  // <> field pos manager
  private final FieldPosManager m_fieldPosManager;

  // <> odometry for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DriveConstants.ChassisKinematics.kDriveKinematics,
    getHeading(), getModulePositions());

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem(FieldPosManager fieldPosManager) {
    m_fieldPosManager = fieldPosManager;

    resetEncoders();
  }

  @Override
  public void periodic() {
    m_odometry.update(getHeading(), getModulePositions());
    m_fieldPosManager.updateFieldPosWithPhotonVisionPose(m_odometry.getPoseMeters());
  }

  /**
   * <>
   *
   * @return the pose
   */
  public Pose2d getPose() {
    return new Pose2d(m_fieldPosManager.getRobotPose().getTranslation(), getHeading());
  }

  /**
   * <> drive the robot
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // <> apply dampers defined in constants
    xSpeed *= DriveConstants.kDrivingSpeedDamper;
    ySpeed *= DriveConstants.kDrivingSpeedDamper;
    rot *= DriveConstants.kAngularSpeedDamper;

    // <> adjust the inputs if field relative is true
    SwerveModuleState[] swerveModuleStates = DriveConstants.ChassisKinematics.kDriveKinematics.toSwerveModuleStates(
      fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading()) : new ChassisSpeeds(
        xSpeed, ySpeed, rot));

    // <> desaturate wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxMetersPerSecond);

    // <> set desired wheel speeds
    m_frontLeft.setDesiredState(swerveModuleStates[0], false);
    m_frontRight.setDesiredState(swerveModuleStates[1], false);
    m_rearLeft.setDesiredState(swerveModuleStates[2], false);
    m_rearRight.setDesiredState(swerveModuleStates[3], false);
  }

  /**
   * <> set wheels into an x position to prevent movement
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), true);
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), true);
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
  }

  /**
   * <> set the swerve modules' desired states
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // <> desaturate wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kDrivingSpeedDamper);

    // <> set the desired states
    m_frontLeft.setDesiredState(desiredStates[0], true);
    m_frontRight.setDesiredState(desiredStates[1], true);
    m_rearLeft.setDesiredState(desiredStates[2], true);
    m_rearRight.setDesiredState(desiredStates[3], true);
  }

  /**
   * <> reset the drive encoders
   */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /**
   * <>
   *
   * @return the robot's heading
   */
  public Rotation2d getHeading() {
    return m_imuSubsystem.getYaw();
  }

  /**
   * <> stops all modules
   */
  public void stopModules() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_rearLeft.stop();
    m_rearRight.stop();
  }

  /**
   * <>
   *
   * @return the positions of the swerve modules
   */
  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[]{m_frontLeft.getPosition(), m_frontRight.getPosition(), m_rearLeft.getPosition()
      , m_rearRight.getPosition()};
  }

  /**
   * <>
   *
   * @return charge station level (see {@link IMUSubsystem}'s charge station level method
   * for more specific details
   */
  public Rotation2d getChargeLevel() {
    return m_imuSubsystem.getChargeLevel();
  }

  /**
   * <>
   *
   * @return if the motors are at an ok temperature (will return false if at unsafe temperatures)
   */
  public boolean getMotorsOkTemperature() {
    return !(m_frontLeft.isTooHot() || m_frontRight.isTooHot() || m_rearLeft.isTooHot() || m_rearRight.isTooHot());
  }
}
