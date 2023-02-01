// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.JoyUtil;
import frc.robot.subsystems.LegAnkleSubsystem;

public class WristCommand extends CommandBase {
  /** Creates a new Wrist. */
  private final LegAnkleSubsystem m_subsystem;
  private final JoyUtil m_controller;

    public WristCommand(LegAnkleSubsystem subsystem, JoyUtil controller){
      m_subsystem = subsystem;
      m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.moveByXYTheta(m_controller.getLeftX(),m_controller.getLeftY() , m_controller.getRightY(), m_controller.getRightX());

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
