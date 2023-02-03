// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PlaceGamePieceCommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.commands.SwerveAutoMoveCommand;
import frc.robot.subsystems.DriveSubsystem;

public class MoveRobotToGrid extends SwerveAutoMoveCommand {
  private boolean isCube;
  private DriveSubsystem driveSubsystem;
  private Constants.Target target;

  /** Creates a new MoveRobotToGrid. */
  public MoveRobotToGrid(boolean isCube, Constants.Target target, DriveSubsystem driveSubsystem, ProfiledPIDController thetaPidController) {
    super(
      driveSubsystem,
      new Pose2d(), // H! TODO Make this actually be the correct pose
      thetaPidController
    );
    this.isCube = isCube;
    this.driveSubsystem = driveSubsystem;
    this.target = target; 
  }
}
