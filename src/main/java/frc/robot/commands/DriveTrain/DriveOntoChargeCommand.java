// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain;

import frc.robot.FieldPosManager;
import frc.robot.subsystems.DriveSubsystem;

public class DriveOntoChargeCommand extends SwerveAutoMoveCommand {
  FieldPosManager m_posManager;

  /** Creates a new DriveOntoChargeCommand. */
  public DriveOntoChargeCommand(DriveSubsystem subsystem, FieldPosManager posManager) {
    super(subsystem, posManager.getChargePoint(true));

    m_posManager = posManager;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_posManager.togglePhotonVisionDisabled(true);
    super.initialize();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_posManager.togglePhotonVisionDisabled(false);
    super.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.isFinished();
  }
}
