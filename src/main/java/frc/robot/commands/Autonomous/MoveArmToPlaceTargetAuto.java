// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import frc.robot.FieldPosManager;
import frc.robot.JoyUtil;
import frc.robot.commands.LegAnkle.MoveLegAnkleToPlacementPositionCommand;
import frc.robot.commands.LegAnkle.MoveLegAnkleToPositionCommand;
import frc.robot.subsystems.LegAnkleSubsystem;
import frc.robot.utility_classes.LegAnklePosition;

public class MoveArmToPlaceTargetAuto extends MoveLegAnkleToPlacementPositionCommand {

  MoveArmToPlaceTargetAuto(LegAnkleSubsystem legAnkleSubsystem, JoyUtil secondaryJoy) {
    super(legAnkleSubsystem, secondaryJoy);
  }

  @Override
  public void initialize() {
    high();
  }
}