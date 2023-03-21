// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LegAnkle;

import frc.robot.subsystems.LegAnkleSubsystem;
import frc.robot.utility_classes.LegAnklePosition;

public class MoveLegAnkleToPartialPositionCommand extends MoveLegAnkleToPositionCommand {

  /**Creates a new MoveLegAnkleToPositionCommand. 
   * H!
   * 
   * This will move the leg ankle to the positions entered when scheduled. It will end when it is in an acceptable margin of the setpoints. You can also enter null for the values and it will not affect that axis.
   * 
   * @param legAnkleSubsystem The {@link LegAnkleSubsystem} that this is controlling.
   * @param targetExtension The extension value to go to, or null if you'd like it to not affect this axis.
   * @param targetPivot The pivot value to go to, or null if you'd like it to not affect this axis.
   * @param targetPitch The pitch value to go to, or null if you'd like it to not affect this axis.
   * @param targetRoll The roll value to go to, or null if you'd like it to not affect this axis.
  */
  public MoveLegAnkleToPartialPositionCommand(LegAnkleSubsystem legAnkleSubsystem, Double targetExtension, Double targetPivot, Double targetPitch, Double targetRoll) {
    super(legAnkleSubsystem);

    // H! For each axis, if a value is passed in, then use that value.
    // :D I think you mean only use the values that we don't pass in lmao
    if (targetExtension != null) {
      this.targetExtension = targetExtension;
    }

    if (targetPivot != null) {
      this.targetPivot = targetPivot;
    }

    if (targetPitch != null) {
      this.targetPitch = targetPitch;
    }

    if (targetRoll != null) {
      this.targetRoll = targetRoll;
    }
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LegAnklePosition currentTargets = legAnkleSubsystem.getManualSetpoints();

    // H! For each axis, if a value is not passed in, then assume that value.
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

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }
}
