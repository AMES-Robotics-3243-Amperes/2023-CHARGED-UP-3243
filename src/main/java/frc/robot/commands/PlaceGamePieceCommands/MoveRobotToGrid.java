// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PlaceGamePieceCommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.JoyUtil;
import frc.robot.commands.SwerveAutoMoveCommand;
import frc.robot.subsystems.DriveSubsystem;

public class MoveRobotToGrid extends SwerveAutoMoveCommand {
  private DriveSubsystem driveSubsystem;
  private JoyUtil controller;
  private ProfiledPIDController thetaPidController;

  /** Creates a new MoveRobotToGrid. */
  public MoveRobotToGrid(Pose2d targetPose, DriveSubsystem driveSubsystem, JoyUtil controller, ProfiledPIDController thetaPidController) {
    super(
      driveSubsystem,
      targetPose,
      thetaPidController,
      false
    );
    this.driveSubsystem = driveSubsystem;
    this.controller = controller;
    this.thetaPidController = thetaPidController;
  }



  @Override
  public boolean isFinished() {
    // H! Only end the command if the a button is pressed to confirm it
    return super.isFinished() & controller.getAButton();
  }
  
  
}
