// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.JoyUtil;
import frc.robot.Constants.JoyUtilConstants;
import frc.robot.subsystems.LegAnkleSubsystem;
import frc.robot.utility_classes.LegAnklePosition;

public class ManualLegAnkleCommand extends CommandBase {
  /**
   * Creates a new Wrist.
   */
  private final LegAnkleSubsystem m_subsystem;
  private final JoyUtil m_controller;

  public ManualLegAnkleCommand(LegAnkleSubsystem subsystem, JoyUtil controller) {
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
    // m_subsystem.moveByXYTheta(JoyUtil.posWithDeadzone( m_controller.getLeftX() ), JoyUtil.posWithDeadzone(
    // -m_controller.getLeftY() ), JoyUtil.posWithDeadzone( m_controller.getRightY() ), JoyUtil.posWithDeadzone(
    // -m_controller.getRightX()));
    LegAnklePosition currentTargets = m_subsystem.getManualSetpoints();

    /*m_subsystem.setMotorPositions(
      currentTargets.pivot, 
      currentTargets.extension, 
      m_controller.getPOVYAxis() > 0.6 ? (0.55) : ( m_controller.getPOVYAxis() < -0.6 ? (0.8) : currentTargets.pitch), 
      currentTargets.roll
    );*/
    //System.out.println(m_controller.getDPadY());


    //&& Fast/slow mode for the LegAnkle using analog triggers
    m_subsystem.changeMotorPositions(
      MathUtil.applyDeadband(m_controller.getRightTriggerAxis() - m_controller.getLeftTriggerAxis(), JoyUtilConstants.kDeadzone) / 200,
      m_controller.getLeftY() / -150,
      m_controller.getRightY() / -150,
      (m_controller.getRightX()) / 200
    );
    
    // H! IK control
    // m_subsystem.moveByXYTheta(
    //   JoyUtil.fastMode(JoyUtil.posWithDeadzone(m_controller.getLeftX()) , m_controller.getLeftTriggerAxis(), m_controller.getRightTriggerAxis()),
    //   JoyUtil.fastMode(JoyUtil.posWithDeadzone(m_controller.getLeftY()) , m_controller.getLeftTriggerAxis(), m_controller.getRightTriggerAxis()),
    //   JoyUtil.fastMode(JoyUtil.posWithDeadzone(m_controller.getRightY()), m_controller.getLeftTriggerAxis(), m_controller.getRightTriggerAxis()),
    //   JoyUtil.fastMode(JoyUtil.posWithDeadzone(m_controller.getRightX()), m_controller.getLeftTriggerAxis(), m_controller.getRightTriggerAxis())
    //   );

    m_subsystem.deleteThis_doSetpoint = true;
    //m_subsystem.resetRoll();

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
