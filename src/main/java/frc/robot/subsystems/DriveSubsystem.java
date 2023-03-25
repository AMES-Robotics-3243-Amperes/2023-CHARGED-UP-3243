// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrain.DriveConstants;
import frc.robot.utility_classes.GeneralUtil;
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

  // <> pid controller for when turning is field relative (degrees)
  private ProfiledPIDController m_thetaPidController;

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem(FieldPosManager fieldPosManager, ProfiledPIDController thetaPIDController) {
    m_fieldPosManager = fieldPosManager;
    m_thetaPidController = thetaPIDController;

    m_thetaPidController.enableContinuousInput(-180, 180);
    resetFieldRelativeTurningPid();

    resetEncoders();
  }

  @Override
  public void periodic() {
    m_odometry.update(m_imuSubsystem.getYaw(), getModulePositions());
    m_fieldPosManager.updateFieldPosWithSwerveData(m_odometry.getPoseMeters());
  }

  /**
   * <> resets the pid controller for field relative turning
   * (smart to do at command initialization)
   */
  public void resetFieldRelativeTurningPid() {
    m_thetaPidController.reset(getHeading().getRadians(), m_imuSubsystem.getTurnRate().getRadians());
  }

  /**
   * <>
   *
   * @return the pose
   */
  public Pose2d getPose() {
    return m_fieldPosManager.getRobotPose();
  }

  /**
   * <> drive the robot
   *
   * @param xSpeed        speed of the robot in the x direction (forward)
   * @param ySpeed        speed of the robot in the y direction (sideways)
   * @param rotation      angular rate of the robot in radians
   * @param fieldRelative whether the provided x and y speeds are relative to the
   *                      field
   */
  private void driveWithRawSpeeds(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
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
   * <> drive the robot and reset pids
   *
   * @param xSpeed        speed of the robot in the x direction (forward)
   * @param ySpeed        speed of the robot in the y direction (sideways)
   * @param rotationSpeed angular rate of the robot in radians
   * @param fieldRelative whether the provided x and y speeds are relative to the
   *                      field
   */
  public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldRelative) {
    // <> because we're not driving field relative, reset those pids
    resetFieldRelativeTurningPid();

    driveWithRawSpeeds(xSpeed, ySpeed, rotationSpeed, fieldRelative);
  }

  /**
   * <> drive the robot with a rotation setpoint
   *
   * @param xSpeed        speed of the robot in the x direction (forward)
   * @param ySpeed        speed of the robot in the y direction (sideways)
   * @param rotation      or goal of field relative driving
   * @param fieldRelative whether the provided x and y speeds are relative to the
   *                      field
   */
  public void drive(double xSpeed, double ySpeed, Rotation2d rotation, boolean fieldRelative) {
    double clampedGoal = GeneralUtil.clampRotation2d(rotation).getDegrees();
    double rotationSpeed = Math.toRadians(m_thetaPidController.calculate(getClampedHeading().getDegrees(), clampedGoal));

    // <> now that we got a speed, drive using raw speeds
    driveWithRawSpeeds(xSpeed, ySpeed, -rotationSpeed, fieldRelative);
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
   * @param desiredStates the desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    setModuleStates(desiredStates, true);
  }

  /**
   * <> set the swerve modules' desired states
   *
   * @param desiredStates              the desired SwerveModule states.
   * @param allowLowSpeedModuleTurning if the modules should turn even if told to go very slow
   */
  public void setModuleStates(SwerveModuleState[] desiredStates, boolean allowLowSpeedModuleTurning) {
    // <> desaturate wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kDrivingSpeedDamper);

    // <> set the desired states
    m_frontLeft.setDesiredState(desiredStates[0], allowLowSpeedModuleTurning);
    m_frontRight.setDesiredState(desiredStates[1], allowLowSpeedModuleTurning);
    m_rearLeft.setDesiredState(desiredStates[2], allowLowSpeedModuleTurning);
    m_rearRight.setDesiredState(desiredStates[3], allowLowSpeedModuleTurning);
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
   * <> returns the raw rotation from the imu
   */
  public Rotation2d getIMURotation() {
    return m_imuSubsystem.getYaw();
  }

  /**
   * <>
   *
   * @return the robot's heading
   */
  public Rotation2d getHeading() {
    if (m_fieldPosManager == null) {
      return m_imuSubsystem.getYaw();
    }

    return m_fieldPosManager.getRobotPose().getRotation();
  }

  /**
   * <> for use with functions that require a specific range (such as pids)
   *
   * @return a rotation 2d with angles from -180 to 180
   */
  public Rotation2d getClampedHeading() {
    return GeneralUtil.clampRotation2d(getHeading());
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
