// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LegAnkleSubsystem;
import frc.robot.utility_classes.LegAnklePosition;

public class MoveLegAnkleToPositionCommand extends CommandBase {
  public Double targetPivot;
  public Double targetPitch;
  public Double targetRoll;
  public Double targetExtension;

  public LegAnkleSubsystem legAnkleSubsystem;

  
  

  /** Creates a new MoveLegAnkleToExtension. */
  public MoveLegAnkleToPositionCommand(LegAnkleSubsystem legAnkleSubsystem, double targetExtension, double targetPivot, double targetPitch, double targetRoll) {
    this.targetExtension = targetExtension;
    this.targetPivot = targetPivot;
    this.targetPitch = targetPitch;
    this.targetRoll = targetRoll;
    this.legAnkleSubsystem = legAnkleSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(legAnkleSubsystem);
  }

  public MoveLegAnkleToPositionCommand(LegAnkleSubsystem legAnkleSubsystem, LegAnklePosition target) {
    this(legAnkleSubsystem, target.extension, target.pivot, target.pitch, target.roll);
  }

  protected MoveLegAnkleToPositionCommand(LegAnkleSubsystem legAnkleSubsystem) {
    this.legAnkleSubsystem = legAnkleSubsystem;

    addRequirements(legAnkleSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
