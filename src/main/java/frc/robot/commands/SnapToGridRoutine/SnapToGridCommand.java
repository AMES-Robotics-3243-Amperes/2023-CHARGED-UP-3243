// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SnapToGridRoutine;

import frc.robot.Constants.DriveTrain.DriveConstants;
import frc.robot.FieldPosManager;
import frc.robot.JoyUtil;
import frc.robot.commands.DriveTrain.SwerveAutoMoveCommand;
import frc.robot.subsystems.*;

public class SnapToGridCommand extends SwerveAutoMoveCommand { // :D hi after looking over this, it looks like due to the fact that only one command
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
   * @param driveSubsystem The Drive Subsystem
   * @param fieldPosManager The Field Position Manager
   * @param JoyUtil The Primary Controller
   * @param JoyUtil The Secondary Controller
  */
  public SnapToGridCommand(DriveSubsystem driveSubsystem, FieldPosManager fieldPosManager, JoyUtil primaryController, JoyUtil secondaryController) {
    super(
      driveSubsystem, 
      fieldPosManager.get2dFieldObjectPose(FieldPosManager.fieldSpot2d.scoringPosition, true, fieldPosManager.getNearestScoringZoneIndex()), 
      DriveConstants.AutoConstants.kMaxMetersFromGoal, 
      DriveConstants.AutoConstants.kMaxRotationFromGoal
    );

    m_FieldPosManager = fieldPosManager;
    m_PrimaryController = primaryController;
    m_SecondaryController = secondaryController;
    
    index = m_FieldPosManager.getNearestScoringZoneIndex();

    // ss make an auto move command from the drive subsystem, the Pose2d corresponding to the index, and some constants
    

    // ss create the index commands from the field pos manager, auto move command, and this SnapToGridCommand
    m_IndexLeftCommand = new IndexLeftCommand(m_FieldPosManager, this);
    m_IndexRightCommand = new IndexRightCommand(m_FieldPosManager, this);

    // ss set the commands to activate when the dpad left and right are pressed
    m_SecondaryController.povLeft().onTrue(m_IndexLeftCommand);
    m_SecondaryController.povRight().onTrue(m_IndexRightCommand);

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
    // ss stops everything if the movement joysticks move. These functions apply deadzone automatically
    if (m_PrimaryController.getLeftX() > 0 || m_PrimaryController.getLeftY() > 0 || m_PrimaryController.getRightY() > 0) { // use Math.abs() and also check RightY joystick
    
      return true;
    }
    return false;
  }
}
