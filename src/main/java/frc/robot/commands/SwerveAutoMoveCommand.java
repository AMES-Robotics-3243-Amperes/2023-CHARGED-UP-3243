// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

/**
 * A command used to drive a {@link DriveSubsystem} to a Pose2d
 */
public class SwerveAutoMoveCommand extends CommandBase {
  private final DriveSubsystem m_subsystem;
  private Pose2d goal;

  private final ProfiledPIDController positionPIDController;
  private final ProfiledPIDController thetaPIDController;

  private final double maxDistanceFromSetpointMeters;
  private final Rotation2d maxAngleFromSetpoint;

  /**
   * <> Crates a new {@link SwerveAutoMoveCommand}. The PID controllers
   * will be manually configured with everything but constraints and
   * P, I, and D values.
   *
   * @param subsystem                     the {@link DriveSubsystem} to control
   * @param destination                   where to drive the {@link DriveSubsystem} to
   * @param positionPIDController         the pid controller for moving the robot
   * @param thetaPIDController            the pid controller for rotating the robot
   * @param maxDistanceFromSetpointMeters farthest the robot can be from the destination to stop the command
   * @param maxAngleFromSetpoint          farthest the robot can be from proper rotation to stop command
   */
  public SwerveAutoMoveCommand(DriveSubsystem subsystem, Pose2d destination,
                               ProfiledPIDController positionPIDController, ProfiledPIDController thetaPIDController,
                               double maxDistanceFromSetpointMeters, Rotation2d maxAngleFromSetpoint) {
    this.m_subsystem = subsystem;
    this.goal = destination;
    this.positionPIDController = positionPIDController;
    this.thetaPIDController = thetaPIDController;
    this.maxDistanceFromSetpointMeters = maxDistanceFromSetpointMeters;
    this.maxAngleFromSetpoint = maxAngleFromSetpoint;

    this.positionPIDController.setGoal(destination.getX());
    this.thetaPIDController.setGoal(destination.getRotation().getDegrees());

    this.thetaPIDController.enableContinuousInput(-180, 180);

    addRequirements(m_subsystem);
  }

  /**
   * <> get a transform representing the robot's distance from goal
   *
   * @return a transform from the destination to the robot's position
   */
  private Transform2d getTransformationFromGoal() {
    return new Transform2d(goal, m_subsystem.getPose());
  }

  /**
   * <> gets the distance from the goal
   *
   * @return the distance from the goal
   */
  private double getDistanceFromGoal() {
    Transform2d offset = getTransformationFromGoal();

    // <> note: there is no need to take the absolute
    // value of these because the values are squared
    double xError = offset.getX();
    double yError = offset.getY();

    return Math.sqrt(xError * xError + yError * yError);
  }

  @Override
  public void initialize() {
    Pose2d robotPose = m_subsystem.getPose();

    positionPIDController.reset(robotPose.getY());
    thetaPIDController.reset(robotPose.getRotation().getDegrees());
  }

  @Override
  public void execute() {
    double drivingOutput = positionPIDController.calculate(getDistanceFromGoal());
    double thetaOutput = thetaPIDController.calculate(m_subsystem.getDiscontinuousHeading().getDegrees());

    driveWithPIDOutputs(drivingOutput, thetaOutput);
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.drive(0, 0, 0, false);
  }

  @Override
  public boolean isFinished() {
    Pose2d subsystemPose = m_subsystem.getPose();

    double xError = Math.abs(goal.getX() - subsystemPose.getX());
    double yError = Math.abs(goal.getY() - subsystemPose.getY());
    double distanceError = Math.sqrt(xError * xError + yError * yError);
    boolean distanceOk = distanceError <= maxDistanceFromSetpointMeters;

    double rotationErrorDegrees = Math.abs(
      subsystemPose.getRotation().getDegrees() - goal.getRotation().getDegrees());
    boolean rotationOk = rotationErrorDegrees <= maxAngleFromSetpoint.getDegrees();

    return distanceOk && rotationOk;
  }

  /**
   * <> drives the robot according to the PID outputs
   *
   * @param drivingOutput the output of the driving PID (speed of robot)
   * @param thetaOutput   output of theta PID (rotation of robot)
   */
  private void driveWithPIDOutputs(double drivingOutput, double thetaOutput) {
    Transform2d offsetFromSetpoint = getTransformationFromGoal();

    Rotation2d driveAngle = Rotation2d.fromRadians(Math.atan2(offsetFromSetpoint.getY(), offsetFromSetpoint.getX()));

    double xComponent = driveAngle.getCos() * drivingOutput;
    double yComponent = driveAngle.getSin() * drivingOutput;

    m_subsystem.drive(xComponent, yComponent, -Math.toRadians(thetaOutput), true);
  }

  /**
   * <> change the goal of the robot
   */
  public void changeGoal(Pose2d newGoal) {
    goal = newGoal;
  }
}
