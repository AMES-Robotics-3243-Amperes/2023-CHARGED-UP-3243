// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
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

  private final PIDController xPidController;
  private final PIDController yPidController;
  private final PIDController thetaPIDController;

  private final double maxDistanceFromSetpointMeters;
  private final Rotation2d maxAngleFromSetpoint;

  public SwerveAutoMoveCommand(DriveSubsystem subsystem, Pose2d destination, PIDController xPidController,
                               PIDController yPidController, PIDController thetaPIDController,
                               double maxDistanceFromSetpointMeters, Rotation2d maxAngleFromSetpoint) {
    this.m_subsystem = subsystem;
    this.destination = destination;
    this.xPidController = xPidController;
    this.yPidController = yPidController;
    this.thetaPIDController = thetaPIDController;
    this.maxDistanceFromSetpointMeters = maxDistanceFromSetpointMeters;
    this.maxAngleFromSetpoint = maxAngleFromSetpoint;

    this.xPidController.setSetpoint(destination.getX());
    this.yPidController.setSetpoint(destination.getY());
    this.thetaPIDController.setSetpoint(destination.getRotation().getDegrees());

    addRequirements(m_subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double xOutput = xPidController.calculate(m_subsystem.getPose().getX());
    double yOutput = yPidController.calculate(m_subsystem.getPose().getY());
    double thetaOutput = thetaPIDController.calculate(m_subsystem.getHeading().getDegrees());

    m_subsystem.drive(xOutput, yOutput, thetaOutput, true);
  }

  @Override
  public void end(boolean interrupted) {}

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
