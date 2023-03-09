// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.LegAnkleSubsystem;
import frc.robot.utility_classes.LegAnklePosition;

public class MoveLegAnkleToPartialPositionCommand extends MoveLegAnkleToPositionCommand {
  /** Creates a new MoveLegAnkleToPartialAxes. */
  public MoveLegAnkleToPartialPositionCommand(LegAnkleSubsystem legAnkleSubsystem, Double targetExtension, Double targetPivot, Double targetPitch, Double targetRoll) {
    super(legAnkleSubsystem);

    // H! For each axis, if a value is passed in, then use that value.
    if (targetExtension == null) {
      this.targetExtension = targetExtension;
    }

    if (targetPivot == null) {
      this.targetPivot = targetPivot;
    }

    if (targetPitch == null) {
      this.targetPitch = targetPitch;
    }

    if (targetRoll == null) {
      this.targetRoll = targetRoll;
    }
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LegAnklePosition currentTargets = legAnkleSubsystem.getManualSetpoints();

    // H! For each axis, if a value is passed in, then assume that value.
    if (targetExtension == null) {
      targetExtension = currentTargets.extension;
    }

    if (targetPivot == null) {
      targetPivot = currentTargets.pivot;
    }

    if (targetPitch == null) {
      targetPitch = currentTargets.pitch;
    }

    if (targetRoll == null) {
      targetRoll = currentTargets.roll;
    }

    super.initialize();
  }
}
