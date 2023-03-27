// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;

public class OpenToWidthCommand extends CommandBase {
  private final GrabberSubsystem m_GrabberSubsystem;
  /** Creates a new OpenToWidthCommand. */
  public OpenToWidthCommand(GrabberSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_GrabberSubsystem = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_GrabberSubsystem.openGrabberToWidth();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
