// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveTrain.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utility_classes.GeneralUtil;

import java.util.ArrayList;
import java.util.List;

/**
 * A command used to drive a {@link DriveSubsystem} to a series Pose2d's
 */
public class SwerveAutoMoveCommand extends CommandBase {
  private final DriveSubsystem m_subsystem;
  private final double maxDistanceFromSetpointMeters;
  private final Rotation2d maxAngleFromSetpoint;

  // <> this is here so that we can refill the
  // goal list each time the command runs
  private final ArrayList<Pose2d> absoluteGoalList;

  // <> current velocity of the robot
  Translation2d velocity = new Translation2d();

  // <> active list of goals to go to (always targets
  // the first one in the list)
  private ArrayList<Pose2d> goalList;

  // <> this should always be the same as the last element of the goal
  // list, and exists to avoid out of range exceptions
  private Pose2d finalGoal;

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
    this(subsystem, new ArrayList<>(List.of(destination)), maxDistanceFromSetpointMeters, maxAngleFromSetpoint);
  }

  /**
   * <> Crates a new {@link SwerveAutoMoveCommand}.
   *
   * @param subsystem                     the {@link DriveSubsystem} to control
   * @param goals                         points to drive the {@link DriveSubsystem} to (order is preserved)
   * @param maxDistanceFromSetpointMeters farthest the robot can be from a setpoint to move on to the next
   * @param maxAngleFromSetpoint          farthest the robot can be from a setpoint rotation to move on
   */
  public SwerveAutoMoveCommand(DriveSubsystem subsystem, ArrayList<Pose2d> goals,
                               double maxDistanceFromSetpointMeters, Rotation2d maxAngleFromSetpoint) {
    this.m_subsystem = subsystem;
    this.maxDistanceFromSetpointMeters = maxDistanceFromSetpointMeters;
    this.maxAngleFromSetpoint = maxAngleFromSetpoint;

    // <> the rotations must be clamped
    this.goalList = new ArrayList<>(List.of());
    for (Pose2d goal : goals) {
      this.goalList.add(new Pose2d(goal.getTranslation(), GeneralUtil.clampRotation2d(goal.getRotation())));
    }

    this.finalGoal = goalList.get(goalList.size() - 1);

    // <> deep-copy the final list just to be absolutely sure it doesn't get modified
    this.absoluteGoalList = new ArrayList<>(List.of());
    this.absoluteGoalList.addAll(goals);

    addRequirements(m_subsystem);
  }

  /**
   * <> Crates a new {@link SwerveAutoMoveCommand}.
   *
   * @param subsystem                     the {@link DriveSubsystem} to control
   * @param destination                   where to drive the {@link DriveSubsystem} to
   */
  public SwerveAutoMoveCommand(DriveSubsystem subsystem, Pose2d destination) {
    this(subsystem, destination, DriveConstants.AutoConstants.kMaxMetersFromGoal, DriveConstants.AutoConstants.kMaxRotationFromGoal);
  }

  /**
   * <> Crates a new {@link SwerveAutoMoveCommand}.
   *
   * @param subsystem                     the {@link DriveSubsystem} to control
   * @param goals                         points to drive the {@link DriveSubsystem} to (order is preserved)
   */
  public SwerveAutoMoveCommand(DriveSubsystem subsystem, ArrayList<Pose2d> goals) {
    this(subsystem, goals, DriveConstants.AutoConstants.kMaxMetersFromGoal, DriveConstants.AutoConstants.kMaxRotationFromGoal);
  }

  private Pose2d getGoal() {
    return goalList.isEmpty() ? finalGoal : goalList.get(0);
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
    double xError = subsystemPose.getX() - getGoal().getX();
    double yError = subsystemPose.getY() - getGoal().getY();

    return Math.sqrt(xError * xError + yError * yError);
  }

  /**
   * <> removes the current goal if we're at it
   */
  public void cleanUpGoalList() {
    if (goalList.isEmpty()) {
      return;
    }

    // <> figure out if we're at a valid position and rotation
    boolean distanceOk = getDistanceFromGoal() <= maxDistanceFromSetpointMeters;

    double rotationErrorDegrees = Math.abs(
      m_subsystem.getClampedHeading().getDegrees() - getGoal().getRotation().getDegrees());
    boolean rotationOk = rotationErrorDegrees <= maxAngleFromSetpoint.getDegrees();

    // <> and if we are remove the current goal
    if (distanceOk && rotationOk) {
      goalList.remove(0);
    }
  }

  @Override
  public void initialize() {
    // <> deep-copy absolute goal list into goal list
    goalList.clear();
    goalList.addAll(absoluteGoalList);

    velocity = new Translation2d();
  }

  @Override
  public void execute() {
    Pose2d subsystemPose = m_subsystem.getPose();

    // <> move on to the next goal if we're at the current one
    cleanUpGoalList();

    // <> find the angle to the goal
    double xError = subsystemPose.getX() - getGoal().getX();
    double yError = subsystemPose.getY() - getGoal().getY();
    Rotation2d driveAngle = Rotation2d.fromRadians(Math.atan2(yError, xError) + Math.PI);

    // <> extract constant so that the next line of code is shorter
    final double speed = DriveConstants.AutoConstants.kMaxVelocityMetersPerSecond;

    // <> we can easily figure out where we *should* be driving, so let this
    // function handle the acceleration limiting and actual driving
    Translation2d goalVelocity = new Translation2d(speed * driveAngle.getCos(), speed * driveAngle.getSin());
    driveWithGoalVelocity(goalVelocity);
  }

  @Override
  public void end(boolean interrupted) {
    // <> stop the robot from driving when the command ends
    m_subsystem.drive(0, 0, getGoal().getRotation(), false);
  }

  @Override
  public boolean isFinished() {
    // <> end if we have no more goals
    return goalList.isEmpty();
  }

  /**
   * <> using a goal velocity, changes the current velocity toward the goal velocity
   * and then drives the robot using that new velocity
   *
   * @param goalVelocity the velocity goal of the robot
   */
  private void driveWithGoalVelocity(Translation2d goalVelocity) {
    // <> find how much we'd need to change the velocity to get to the goal velocity
    Translation2d velocityDifference = goalVelocity.plus(velocity.times(-1)).times(0.8);
    double velocityDifferenceLength = velocityDifference.getNorm();

    // <> the constant is in seconds, but loop time is 20 ms, so extract
    // the constant in terms of the max velocity change per loop
    final double maxVelocityChange = DriveConstants.AutoConstants.kMaxAccelerationMetersPerSecondSq / 50;

    // if the change in velocity exceeds the max velocity change per 20 ms, slow it down
    if (velocityDifferenceLength >= maxVelocityChange) {
      // <> normalize velocity change
      velocityDifference = velocityDifference.div(velocityDifferenceLength);

      // <> and multiply it by the max velocity change
      velocityDifference = velocityDifference.times(maxVelocityChange);
    }

    // <> update velocity and drive
    velocity = velocity.plus(velocityDifference);
    m_subsystem.drive(velocity.getX(), velocity.getY(), getGoal().getRotation(), true);
  }

  /**
   * <> change the goal of the robot (doesn't affect intermediate points)
   */
  public void changeGoal(Pose2d newGoal) {
    // <> set the final goal variable and the last members of the goal lists
    // to the passed in new goal, ignoring all other existing goals
    finalGoal = newGoal;
    goalList.set(goalList.size() - 1, newGoal);
    absoluteGoalList.set(absoluteGoalList.size() - 1, newGoal);
  }

  /**
   * <> change the goal of the robot (will go back and travel to all points given)
   */
  public void changeGoal(ArrayList<Pose2d> newGoals) {
    // <> update the final goal variable
    finalGoal = newGoals.get(newGoals.size() - 1);

    // <> deep-copy the array lists
    goalList.clear();
    goalList.addAll(newGoals);

    absoluteGoalList.clear();
    absoluteGoalList.addAll(newGoals);
  }
}
