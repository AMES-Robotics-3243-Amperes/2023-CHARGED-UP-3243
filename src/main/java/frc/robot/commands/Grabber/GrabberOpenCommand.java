// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Grabber;

import frc.robot.JoyUtil;
import frc.robot.subsystems.GrabberSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/** GrabberCommand controls the grabber. */
public class GrabberOpenCommand extends InstantCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  // ££ Defines the subsystem and the controller
  JoyUtil joy;
  private final GrabberSubsystem m_GrabberSubsystem;

  /**
   * Creates a new GrabberOpenCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public GrabberOpenCommand(GrabberSubsystem subsystem, JoyUtil joy) {
    // Assigns the subsystem and the controller values
    m_GrabberSubsystem = subsystem;
    this.joy = joy;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {


    if (joy.getPOVUp()) {
      m_GrabberSubsystem.ejectObject();
    } else if (joy.getPOVDown()) {
      m_GrabberSubsystem.openGrabberToWidth();
    } else {
      m_GrabberSubsystem.openGrabber();
    }
    
  }

}
