// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveTrain.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * A command used to drive a {@link DriveSubsystem} to a Pose2d
 */
public class SwerveAutoMoveCommand extends CommandBase {
  private final DriveSubsystem m_subsystem;
  private final double maxDistanceFromSetpointMeters;
  private final Rotation2d maxAngleFromSetpoint;
  // <> used to stop the robot from accelerating too quickly
  private final SlewRateLimiter limiter = new SlewRateLimiter(
    DriveConstants.AutoConstants.kMaxAccelerationMetersPerSecondSq);
  private Pose2d goal;

  /**
   * <> Crates a new {@link SwerveAutoMoveCommand}.
   *
   * @param subsystem                     the {@link DriveSubsystem} to control
   * @param destination                   where to drive the {@link DriveSubsystem} to
   * @param maxDistanceFromSetpointMeters farthest the robot can be from the destination to stop the command
   * @param maxAngleFromSetpoint          farthest the robot can be from proper rotation to stop command
   */
  public SwerveAutoMoveCommand(DriveSubsystem subsystem, Pose2d destination, double maxDistanceFromSetpointMeters,
                               Rotation2d maxAngleFromSetpoint) {
    this.m_subsystem = subsystem;
    this.goal = destination;
    this.maxDistanceFromSetpointMeters = maxDistanceFromSetpointMeters;
    this.maxAngleFromSetpoint = maxAngleFromSetpoint;

    addRequirements(m_subsystem);
  }

  /**
   * <> gets the distance from the goal
   *
   * @return the distance from the goal
   */
  private double getDistanceFromGoal() {
    Pose2d subsystemPose = m_subsystem.getPose();

    // <> note: there is no need to take the absolute
    // value of these because the values are squared
    double xError = subsystemPose.getX() - goal.getX();
    double yError = subsystemPose.getY() - goal.getY();

    return Math.sqrt(xError * xError + yError * yError);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Pose2d subsystemPose = m_subsystem.getPose();

    // <> find the angle to the goal
    double xError = subsystemPose.getX() - goal.getX();
    double yError = subsystemPose.getY() - goal.getY();
    Rotation2d angle = Rotation2d.fromRadians(Math.atan2(yError, xError) + Math.PI);

    // <> find speed and drive
    double speed = limiter.calculate(DriveConstants.AutoConstants.kMaxVelocityMetersPerSecond);
    m_subsystem.drive(speed * angle.getCos(), speed * angle.getSin(), goal.getRotation(), true);
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.drive(0, 0, goal.getRotation(), false);
  }

  @Override
  public boolean isFinished() {
    boolean distanceOk = getDistanceFromGoal() <= maxDistanceFromSetpointMeters;

    double rotationErrorDegrees = Math.abs(
      m_subsystem.getPose().getRotation().getDegrees() - goal.getRotation().getDegrees());
    boolean rotationOk = rotationErrorDegrees <= maxAngleFromSetpoint.getDegrees();

    return distanceOk && rotationOk;
  }

  /**
   * <> change the goal of the robot
   */
  public void changeGoal(Pose2d newGoal) {
    goal = newGoal;
  }
}
