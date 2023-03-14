// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.DriveTrain.DriveConstants;
import frc.robot.FieldPosManager;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SnapToGrid extends SequentialCommandGroup {
  private final SwerveAutoMoveCommand m_SwerveAutoMoveCommand;
  private final DriveSubsystem m_DriveSubsystem;
  private final FieldPosManager m_FieldPosManager;

  /** Creates a new SnapToGrid. */
  public SnapToGrid(DriveSubsystem driveSubsystem, FieldPosManager fieldPosManager) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_DriveSubsystem = driveSubsystem;
    m_FieldPosManager = fieldPosManager;


    m_SwerveAutoMoveCommand = new SwerveAutoMoveCommand(
      m_DriveSubsystem, 
      m_FieldPosManager.get2dFieldObjectPose(FieldPosManager.fieldSpot2d.scoringPosition, true, m_FieldPosManager.getNearestScoringZoneIndex()), 
      DriveConstants.AutoConstants.kDrivingPIDController, 
      DriveConstants.AutoConstants.kTurningPIDController,
      DriveConstants.AutoConstants.maxMetersFromSetpoint, 
      DriveConstants.AutoConstants.maxRotationFromSetpoint
    );
    

    addCommands(m_SwerveAutoMoveCommand);
  }
}
