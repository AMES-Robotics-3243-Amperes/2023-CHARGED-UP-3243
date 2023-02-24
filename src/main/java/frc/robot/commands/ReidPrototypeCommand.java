// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ReidPrototypeSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class ReidPrototypeCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  // ££ Defines the subsystem and the controller
  private final ReidPrototypeSubsystem m_reidPrototypeSubsystem;
  private final XboxController m_controller;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ReidPrototypeCommand(ReidPrototypeSubsystem subsystem, XboxController controller) {
    // Assigns the subsystem and the controller values
    m_reidPrototypeSubsystem = subsystem;
    m_controller = controller;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  // ££ In brevity: If the X button is pressed the compliant motors will spin, if the B button is pressed the grabber will close, if the Y button is pressed the grabber will open, and if the A button is pressed PID tuning can begin. Tune by changing the values that are passed to setPIDValues()
  @Override
  public void execute() {
    if (m_controller.getLeftBumperPressed()) {
      m_reidPrototypeSubsystem.closeGrabber();
    } else if (m_controller.getRightBumperPressed()) {
      m_reidPrototypeSubsystem.openGrabber();
    }
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
