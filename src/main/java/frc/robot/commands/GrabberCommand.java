// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.GrabberSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class GrabberCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  // ££ Defines the subsystem and the controller
  private final GrabberSubsystem m_GrabberSubsystem;
  private final XboxController m_controller;

  /**
   * Creates a new GrabberCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public GrabberCommand(GrabberSubsystem subsystem, XboxController controller) {
    // Assigns the subsystem and the controller values
    m_GrabberSubsystem = subsystem;
    m_controller = controller;

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

    // ++ set speeds based on controller readings TEMPORARY, FOR TESTING
    m_GrabberSubsystem.setGrabberPosition( m_controller.getLeftX() );
    m_GrabberSubsystem.setGrabberWheelSpeeds( m_controller.getRightTriggerAxis() * 0.05);
    

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
    return false;
  }
}
