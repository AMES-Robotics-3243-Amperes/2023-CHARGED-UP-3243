// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import frc.robot.JoyUtil;
import frc.robot.commands.LegAnkle.MoveLegAnkleToPlacementPositionCommand;
import frc.robot.subsystems.LegAnkleSubsystem;

public class MoveArmToPlaceTargetAuto extends MoveLegAnkleToPlacementPositionCommand {

  MoveArmToPlaceTargetAuto(LegAnkleSubsystem legAnkleSubsystem, JoyUtil secondaryJoy) {
    super(legAnkleSubsystem, secondaryJoy);
  }

  @Override
  public void initialize() {
    high();
  }
}