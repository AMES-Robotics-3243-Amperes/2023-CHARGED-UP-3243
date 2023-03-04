// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LegAnkleSubsystem;

public class MoveLegAnkleToPosition extends CommandBase {
  public double targetPivot;
  public double targetPitch;
  public double targetRoll;
  public LegAnkleSubsystem legAnkleSubsystem;

  public double targetExtension;
  

  /** Creates a new MoveLegAnkleToExtension. */
  public MoveLegAnkleToPosition(LegAnkleSubsystem legAnkleSubsystem, double targetPivot, double targetPitch, double targetRoll) {
    this.targetPivot = targetPivot;
    this.targetPitch = targetPitch;
    this.targetRoll = targetRoll;
    this.legAnkleSubsystem = legAnkleSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(legAnkleSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LegAnkleSubsystem.MotorPos currentTargets = legAnkleSubsystem.getManualSetpoints();

    targetExtension = currentTargets.extension;

    legAnkleSubsystem.setManualSetpoints(targetPivot, targetExtension, targetPitch, targetRoll);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    legAnkleSubsystem.setManualSetpoints(targetPivot, targetExtension, targetPitch, targetRoll);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return legAnkleSubsystem.nearTargetPos();
  }
}
