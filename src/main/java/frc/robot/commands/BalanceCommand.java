// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveTrain.DriveConstants.BalanceConstants;
import frc.robot.subsystems.DriveSubsystem;

public class BalanceCommand extends CommandBase {
  private final DriveSubsystem m_subsystem;
  private final ProfiledPIDController m_pidController = BalanceConstants.PIDController;

  // keeps track of how long the robot has been balanced
  private final Timer m_timer = new Timer();

  /**
   * Creates a new BalanceCommand.
   */
  public BalanceCommand(DriveSubsystem subsystem) {
    m_subsystem = subsystem;
    m_pidController.setGoal(0);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double chargeLevelDegrees = m_subsystem.getChargeLevel().getDegrees();
    boolean isBalanced = Math.abs(chargeLevelDegrees) < BalanceConstants.kMaxBalanceLeniency.getDegrees();

    if (!isBalanced) {
      m_timer.restart();
    }

    double pidOutput = m_pidController.calculate(m_subsystem.getChargeLevel().getDegrees());
    m_subsystem.drive(-pidOutput, 0, 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(BalanceConstants.kBalanceTimeSeconds);
  }
}
