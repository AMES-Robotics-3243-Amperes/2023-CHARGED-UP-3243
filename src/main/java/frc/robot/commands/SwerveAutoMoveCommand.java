// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

/**
 * A command used to drive a {@link DriveSubsystem} to a Pose2d
 */
public class SwerveAutoMoveCommand extends CommandBase {
  private final DriveSubsystem m_subsystem;
  private final Pose2d destination;

  private final ProfiledPIDController xPidController;
  private final ProfiledPIDController yPidController;
  private final ProfiledPIDController thetaPIDController;

  private final double maxDistanceFromSetpointMeters;
  private final Rotation2d maxAngleFromSetpoint;

  /**
   * Crates a new {@link SwerveAutoMoveCommand}. The PID controllers
   * will be manually configured with everything but constraints and
   * P, I, and D values.
   *
   * @param subsystem                     the {@link DriveSubsystem} to control
   * @param destination                   where to drive the {@link DriveSubsystem} to
   * @param xPidController                the pid controller for moving the robot across the x-axis
   * @param yPidController                the pid controller for moving the robot across the y-axis
   * @param thetaPIDController            the pid controller for controlling rotation
   * @param maxDistanceFromSetpointMeters farthest the robot can be from the destination to stop the command
   * @param maxAngleFromSetpoint          farthest the robot can be from proper rotation to stop command
   */
  public SwerveAutoMoveCommand(DriveSubsystem subsystem, Pose2d destination, ProfiledPIDController xPidController,
                               ProfiledPIDController yPidController, ProfiledPIDController thetaPIDController,
                               double maxDistanceFromSetpointMeters, Rotation2d maxAngleFromSetpoint) {
    this.m_subsystem = subsystem;
    this.destination = destination;
    this.xPidController = xPidController;
    this.yPidController = yPidController;
    this.thetaPIDController = thetaPIDController;
    this.maxDistanceFromSetpointMeters = maxDistanceFromSetpointMeters;
    this.maxAngleFromSetpoint = maxAngleFromSetpoint;

    this.xPidController.setGoal(destination.getX());
    this.yPidController.setGoal(destination.getY());
    this.thetaPIDController.setGoal(destination.getRotation().getDegrees());

    this.thetaPIDController.enableContinuousInput(-180, 180);

    addRequirements(m_subsystem);
  }

  @Override
  public void initialize() {
    Pose2d robotPose = m_subsystem.getPose();

    xPidController.reset(robotPose.getX());
    yPidController.reset(robotPose.getY());
    thetaPIDController.reset(robotPose.getRotation().getDegrees());
  }

  @Override
  public void execute() {
    double xOutput = xPidController.calculate(m_subsystem.getPose().getX());
    double yOutput = yPidController.calculate(m_subsystem.getPose().getY());
    double thetaOutput = thetaPIDController.calculate(m_subsystem.getHeading().getDegrees());

    m_subsystem.drive(xOutput, yOutput, -Math.toRadians(thetaOutput), true);
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.drive(0, 0, 0, false);
  }

  @Override
  public boolean isFinished() {
    Pose2d subsystemPose = m_subsystem.getPose();

    double xError = Math.abs(destination.getX() - subsystemPose.getX());
    double yError = Math.abs(destination.getY() - subsystemPose.getY());
    double distanceError = Math.sqrt(xError * xError + yError * yError);
    boolean distanceOk = distanceError <= maxDistanceFromSetpointMeters;

    double rotationErrorDegrees = Math.abs(
      subsystemPose.getRotation().getDegrees() - destination.getRotation().getDegrees());
    boolean rotationOk = rotationErrorDegrees <= maxAngleFromSetpoint.getDegrees();

    return distanceOk && rotationOk;
  }
}
