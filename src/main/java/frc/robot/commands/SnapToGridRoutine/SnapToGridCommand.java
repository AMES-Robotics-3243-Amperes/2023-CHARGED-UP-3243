// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SnapToGridRoutine;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.DriveTrain.DriveConstants;
import frc.robot.FieldPosManager.fieldSpot2d;
import frc.robot.FieldPosManager;
import frc.robot.JoyUtil;
import frc.robot.commands.DriveTrain.SwerveAutoMoveCommand;
import frc.robot.commands.SnapToGridRoutine.IndexLeftCommand;
import frc.robot.commands.SnapToGridRoutine.IndexRightCommand;
import frc.robot.subsystems.*;

public class SnapToGridCommand extends CommandBase {
  private final SwerveAutoMoveCommand m_SwerveAutoMoveCommand;
  private final DriveSubsystem m_DriveSubsystem;
  private final FieldPosManager m_FieldPosManager;
  private final IndexLeftCommand m_IndexLeftCommand;
  private final IndexRightCommand m_IndexRightCommand;
  private final JoyUtil m_PrimaryController;
  private final JoyUtil m_SecondaryController;
  public int index;

  /** Creates a new SnapToGridCommand. 
   * 
   * This is a command that will 'snap' the robot to the nearest of the 9 friendly scoring positions, and then allow t
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
    index = m_FieldPosManager.getNearestScoringZoneIndex();


    m_SwerveAutoMoveCommand = new SwerveAutoMoveCommand(
      m_DriveSubsystem, 
      m_FieldPosManager.get2dFieldObjectPose(FieldPosManager.fieldSpot2d.scoringPosition, true, index), 
      DriveConstants.AutoConstants.kMaxMetersFromGoal, 
      DriveConstants.AutoConstants.kMaxRotationFromGoal
    );

    m_IndexLeftCommand = new IndexLeftCommand(m_FieldPosManager, m_SwerveAutoMoveCommand, this);
    m_IndexRightCommand = new IndexRightCommand(m_FieldPosManager, m_SwerveAutoMoveCommand, this);

    m_SecondaryController.povLeft().onTrue(m_IndexLeftCommand);
    m_SecondaryController.povRight().onTrue(m_IndexRightCommand);

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_SwerveAutoMoveCommand.schedule();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // ss stops everything if the movement joysticks move. These functions apply deadzone automatically
    if (m_PrimaryController.getLeftX() > 0 || m_PrimaryController.getLeftY() > 0 || m_PrimaryController.getRightY() > 0) {
      m_SwerveAutoMoveCommand.cancel();
      return true;
    }
    return false;
  }
}
