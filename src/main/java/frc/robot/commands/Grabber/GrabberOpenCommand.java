// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Grabber;

import frc.robot.subsystems.GrabberSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** GrabberCommand controls the grabber. */
public class GrabberOpenCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  // ££ Defines the subsystem and the controller
  private final GrabberSubsystem m_GrabberSubsystem;

  /**
   * Creates a new GrabberOpenCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public GrabberOpenCommand(GrabberSubsystem subsystem) {
    // Assigns the subsystem and the controller values
    m_GrabberSubsystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // ++ set the PID values of the grabber
    m_GrabberSubsystem.setGrabberPIDValues();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
    m_GrabberSubsystem.openGrabber();
      

  }

 
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_GrabberSubsystem.setGrabberWheelSpeeds(0.0);
    m_GrabberSubsystem.setGrabberWheelSpeeds(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
