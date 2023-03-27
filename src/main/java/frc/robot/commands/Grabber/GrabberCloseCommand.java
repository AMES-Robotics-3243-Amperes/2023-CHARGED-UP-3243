// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Grabber;

import frc.robot.subsystems.GrabberSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;

/** GrabberCommand controls the grabber. */
public class GrabberCloseCommand extends InstantCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  // ££ Defines the subsystem and the controller
  private final GrabberSubsystem m_GrabberSubsystem;

  /**
   * Creates a new GrabberOpenCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public GrabberCloseCommand(GrabberSubsystem subsystem) {
    // Assigns the subsystem and the controller values
    m_GrabberSubsystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // ++ Called when the command runs. It's an instant command, so init's really the only thing here
  @Override
  public void initialize() {
    
    m_GrabberSubsystem.closeGrabber();

  }

}
