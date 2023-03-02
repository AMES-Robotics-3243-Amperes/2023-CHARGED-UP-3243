// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LegAnkleSubsystem;

public class MoveLegAnkleToPickupPositionCommand extends InstantCommand {
  LegAnkleSubsystem m_subsystem;

  public MoveLegAnkleToPickupPositionCommand(LegAnkleSubsystem subsystem) {
    m_subsystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.moveToXYTheta(-0.889, 0, -Math.PI / 2, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
