// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.DriveTrain.DriveConstants;
import frc.robot.Constants.DriveTrain.DriveConstants.AutoConstants;
import frc.robot.Constants.DriveTrain.DriveConstants.ChasisKinematics;
import frc.robot.subsystems.DriveSubsystem;
import java.util.List;

/// <> just a wrapper for a SwerveControllerCommand with an edited constructor
public class SwerveAutoMoveCommand extends SwerveControllerCommand {

  /** Creates a new SwerveAutoMoveCommand.
   *
   * @param
   */
  // <> requires a theta pid controller because enable
  // <> continuous output cannot be called statically
  public SwerveAutoMoveCommand(
    DriveSubsystem subsystem,
    Trajectory trajectory,
    ProfiledPIDController thetaPidController
  ) {
    super(
      trajectory,
      subsystem::getPose,
      ChasisKinematics.kDriveKinematics,
      DriveConstants.AutoConstants.movementPidController,
      AutoConstants.movementPidController,
      thetaPidController,
      subsystem::setModuleStates,
      subsystem
    );
  }

  public SwerveAutoMoveCommand(
    DriveSubsystem subsystem,
    Pose2d pose,
    ProfiledPIDController thetaPidController
  ) {
    super(
      TrajectoryGenerator.generateTrajectory(
        subsystem.getPose(),
        List.of(),
        pose,
        DriveConstants.AutoConstants.trajectoryConfig
      ),
      subsystem::getPose,
      ChasisKinematics.kDriveKinematics,
      DriveConstants.AutoConstants.movementPidController,
      AutoConstants.movementPidController,
      thetaPidController,
      subsystem::setModuleStates,
      subsystem
    );
  }
}
