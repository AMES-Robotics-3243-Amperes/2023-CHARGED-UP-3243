// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveTrain.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utility_classes.GeneralUtil;

import java.util.ArrayList;
import java.util.List;

/**
 * A command used to drive a {@link DriveSubsystem} to a series Pose2d's
 * 
 * <p> If more than 1 point is given, all but the last points are more
 * lenient with when the robot will move on from that point
 */
public class SwerveAutoMoveCommand extends CommandBase {
  protected final DriveSubsystem m_subsystem;
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

  // <> takes the length of the velocity difference and outputs the magnited of acceleration
  private ProfiledPIDController velocityPidController;

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

    // <> create pid controller
    this.velocityPidController = new ProfiledPIDController(DriveConstants.AutoConstants.kP,
      DriveConstants.AutoConstants.kI, DriveConstants.AutoConstants.kD, DriveConstants.AutoConstants.kConstraints);

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

    // <> get the max distance and rotation from the goal
    // (more lenient if on an intermidiate point)
    double maxDistance = goalList.size() > 1 ? DriveConstants.AutoConstants.kMaxLenientMetersFromGoal : maxDistanceFromSetpointMeters;
    Rotation2d maxRotationOffset = goalList.size() > 1 ? DriveConstants.AutoConstants.kMaxLenientRotationFromGoal : maxAngleFromSetpoint;

    // <> figure out if we're at a valid position and rotation
    boolean distanceOk = getDistanceFromGoal() <= maxDistance;

    double rotationErrorDegrees = Math.abs(
      m_subsystem.getClampedHeading().getDegrees() - getGoal().getRotation().getDegrees());
    boolean rotationOk = rotationErrorDegrees <= maxRotationOffset.getDegrees();

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

    // <> find the translation from the robot to the goal
    Translation2d uncappedGoalVelocity = getGoal().getTranslation().minus(subsystemPose.getTranslation())
      .times(DriveConstants.AutoConstants.kGoalVelocityMagnitudeScalar);
    double uncappedGoalVelocityMagnitude = uncappedGoalVelocity.getNorm();

    // <> if the uncapped goal velocity is too long, limit the magnitude
    double maxVelocity = DriveConstants.AutoConstants.kMaxVelocityMetersPerSecond;
    Translation2d goalVelocity = (uncappedGoalVelocityMagnitude > maxVelocity) ?
      uncappedGoalVelocity.div(uncappedGoalVelocityMagnitude).times(maxVelocity) : uncappedGoalVelocity;
    
    // <> we can easily figure out where we *should* be driving, so let this
    // function handle the acceleration limiting and actual driving
    driveWithGoalVelocity(goalVelocity);

    SmartDashboard.putNumber("goalx", goalVelocity.getX());
    SmartDashboard.putNumber("goaly", goalVelocity.getY());
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

    // <> get magnitude of the velocity change
    double velocityChangeMagnitude = velocityPidController.calculate(0, velocityDifferenceLength);

    // <> get the angle of the velocity change
    Rotation2d velocityChangeAngle = Rotation2d.fromRadians(Math.atan2(velocityDifference.getY(), velocityDifference.getX()));

    // <> get the velocity change
    Translation2d velocityChange = new Translation2d(
      velocityChangeMagnitude * velocityChangeAngle.getCos(), velocityChangeMagnitude * velocityChangeAngle.getSin());

    // <> add velocity change to velocity
    velocity = velocity.plus(velocityChange);

    // <> drive using velocity
    m_subsystem.drive(velocity.getX(), velocity.getY(), getGoal().getRotation(), true);
  }

  /**
   * <> change the goal of the robot (doesn't affect intermediate points)
   */
  public void changeGoal(Pose2d newGoal) {
    // <> set the final goal variable and the last members of the goal lists
    // to the passed in new goal, ignoring all other existing goals
    finalGoal = newGoal;

    // <> make both of the lists contain just one element: newGoal
    goalList.clear();
    goalList.add(newGoal);

    absoluteGoalList.clear();
    absoluteGoalList.add(newGoal);
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
