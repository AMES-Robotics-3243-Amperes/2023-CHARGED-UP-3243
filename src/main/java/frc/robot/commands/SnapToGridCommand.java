// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.DriveTrain.DriveConstants;
import frc.robot.FieldPosManager;
import frc.robot.JoyUtil;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class SnapToGridCommand extends CommandBase {
  private final SwerveAutoMoveCommand m_SwerveAutoMoveCommand;
  private final DriveSubsystem m_DriveSubsystem;
  private final FieldPosManager m_FieldPosManager;
  private final JoyUtil m_PrimaryController;
  private final JoyUtil m_SecondaryController;

  /** Creates a new SnapToGridCommand. 
   * 
   * This is a sequential command group that brings the robot to the alliance scoring zone and 'snaps' it to the 3 by 9 grid of placement positions.
   * 
   * @param driveSubsystem The {@link DriveSubsystem} used to move the robot to the grid
   * @param fieldPosManager The {@link FieldPosManager} to figure out where to go
  */
  public SnapToGridCommand(DriveSubsystem driveSubsystem, FieldPosManager fieldPosManager, JoyUtil primaryController, JoyUtil secondaryController) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DriveSubsystem = driveSubsystem;
    m_FieldPosManager = fieldPosManager;
    m_PrimaryController = primaryController;
    m_SecondaryController = secondaryController;


    m_SwerveAutoMoveCommand = new SwerveAutoMoveCommand(
      m_DriveSubsystem, 
      m_FieldPosManager.get2dFieldObjectPose(FieldPosManager.fieldSpot2d.scoringPosition, true, m_FieldPosManager.getNearestScoringZoneIndex()), 
      DriveConstants.AutoConstants.kDrivingPIDController, 
      DriveConstants.AutoConstants.kTurningPIDController,
      DriveConstants.AutoConstants.maxMetersFromSetpoint, 
      DriveConstants.AutoConstants.maxRotationFromSetpoint
    );

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}
