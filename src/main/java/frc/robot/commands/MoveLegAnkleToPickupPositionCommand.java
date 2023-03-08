// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LegAnkleSubsystem;

public class MoveLegAnkleToPickupPositionCommand extends CommandBase {
  LegAnkleSubsystem m_LegAnkleSubsystem;

  public MoveLegAnkleToPickupPositionCommand(LegAnkleSubsystem subsystem) {
    m_LegAnkleSubsystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_LegAnkleSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // ++ set values to measured constants
    m_LegAnkleSubsystem.changeMotorPositions(
      Constants.WristAndArm.pivotPickupPos, 
      Constants.WristAndArm.extensionPickupPos,
      Constants.WristAndArm.pitchPickupPos, 
      Constants.WristAndArm.rollPickupPos);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_LegAnkleSubsystem.isArmPositioned();
  }
}
