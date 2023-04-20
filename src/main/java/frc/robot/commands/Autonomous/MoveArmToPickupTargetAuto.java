// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import frc.robot.commands.LegAnkle.PickupPosition.MoveLegAnkleToPickupPositionCommandSweep;
import frc.robot.subsystems.LegAnkleSubsystem;

public class MoveArmToPickupTargetAuto extends MoveLegAnkleToPickupPositionCommandSweep {
  LegAnkleSubsystem legAnkleSubsystem;
  public MoveArmToPickupTargetAuto(LegAnkleSubsystem legAnkleSubsystem) {
    super(legAnkleSubsystem);
  }
}