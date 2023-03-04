// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveOntoCharge extends CommandBase {
  DriveSubsystem m_Subsystem;

  /** Creates a new DriveOntoCharge. */
  public DriveOntoCharge(DriveSubsystem subsystem) {
    m_Subsystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Subsystem.drive(-0.8, 0, 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_Subsystem.getChargeLevel().getDegrees()) > 5.5;
  }
}
