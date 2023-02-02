// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.DriveTrain.DriveConstants.AutoConstants;
import frc.robot.Constants.DriveTrain.DriveConstants.ChasisKinematics;
import frc.robot.subsystems.DriveSubsystem;

/// <> just a wrapper for a SwerveControllerCommand with an edited constructor
public class SwerveTrajectoryFollowCommand extends SwerveControllerCommand {

  private final DriveSubsystem m_subsystem;

  /** Creates a new SwerveTrajectoryFollowCommand. */
  // <> requires a theta pid controller because enable
  // <> continuous output cannot be called statically
  public SwerveTrajectoryFollowCommand(
    DriveSubsystem subsystem,
    Trajectory trajectory,
    ProfiledPIDController thetaPidController
  ) {
    super(
      trajectory,
      subsystem::getPose,
      ChasisKinematics.kDriveKinematics,
      AutoConstants.movementPidController,
      AutoConstants.movementPidController,
      thetaPidController,
      subsystem::setModuleStates,
      subsystem
    );
    m_subsystem = subsystem;
  }
}
