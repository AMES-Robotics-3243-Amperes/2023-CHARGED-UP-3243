// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Grabber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.GrabberSubsystem;

public class OpenToWidthCommand extends InstantCommand {

  private final GrabberSubsystem m_GrabberSubsystem;
  /** Creates a new OpenToWidthCommand. */
  public OpenToWidthCommand(GrabberSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_GrabberSubsystem = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_GrabberSubsystem.openGrabberToWidth();
    
  }

}
