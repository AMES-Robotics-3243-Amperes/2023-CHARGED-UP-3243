// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LegAnkleSubsystem;

public class MoveLegAnkleToPivot extends CommandBase {
  public double targetPivot;
  public LegAnkleSubsystem legAnkleSubsystem;

  public double targetExtension;
  public double targetPitch;
  public double targetRoll;

  /** Creates a new MoveLegAnkleToExtension. */
  public MoveLegAnkleToPivot(LegAnkleSubsystem legAnkleSubsystem, double targetPivot) {
    this.targetPivot = targetPivot;
    this.legAnkleSubsystem = legAnkleSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(legAnkleSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LegAnkleSubsystem.MotorPos currentTargets = legAnkleSubsystem.getManualSetpoints();

    targetExtension = currentTargets.extension;
    targetPitch = currentTargets.pitch;
    targetRoll = currentTargets.roll;

    legAnkleSubsystem.setMotorPositions(targetPivot, targetExtension, targetPitch, targetRoll);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    legAnkleSubsystem.setMotorPositions(targetPivot, targetExtension, targetPitch, targetRoll);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return legAnkleSubsystem.isArmPositioned();
  }
}
