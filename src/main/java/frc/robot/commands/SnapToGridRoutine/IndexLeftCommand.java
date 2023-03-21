// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SnapToGridRoutine;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldPosManager;
import frc.robot.commands.DriveTrain.SwerveAutoMoveCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IndexLeftCommand extends InstantCommand {
  private final FieldPosManager m_FieldPosManager;
  private final SwerveAutoMoveCommand m_SwerveAutoMoveCommand;
  private final SnapToGridCommand m_SnapToGridCommand;

  /**
   * This command exists to handle what happens when dpad left is pressed on the secondary controller
   * 
   * @param fieldPosManager
   * @param swerveAutoMoveCommand
   * @param snapToGridCommand
   */
  public IndexLeftCommand(FieldPosManager fieldPosManager, SwerveAutoMoveCommand swerveAutoMoveCommand, SnapToGridCommand snapToGridCommand) {
    m_FieldPosManager = fieldPosManager;
    m_SwerveAutoMoveCommand = swerveAutoMoveCommand;
    m_SnapToGridCommand = snapToGridCommand;

    if (m_FieldPosManager.allianceColor == Alliance.Blue) {
      m_SnapToGridCommand.index++;
    }
    else if (m_FieldPosManager.allianceColor == Alliance.Red) {
      m_SnapToGridCommand.index--;
    }

    m_SwerveAutoMoveCommand.changeGoal(m_FieldPosManager.get2dFieldObjectPose(FieldPosManager.fieldSpot2d.scoringPosition, true, m_SnapToGridCommand.index));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
}
