// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import frc.robot.FieldPosManager;
import frc.robot.commands.LegAnkle.MoveLegAnkleToPositionCommand;
import frc.robot.subsystems.LegAnkleSubsystem;
import frc.robot.utility_classes.LegAnklePosition;

public class MoveArmToPickupTargetAuto extends MoveLegAnkleToPositionCommand {
  private int pickupIndex;
  private FieldPosManager fieldPosManager;

  /** Creates a new MoveArmToTarget. */
  public MoveArmToPickupTargetAuto(int pickupIndex, LegAnkleSubsystem legAnkleSubsystem, FieldPosManager fieldPosManager) {
    super(legAnkleSubsystem);
    this.fieldPosManager = fieldPosManager;
    this.pickupIndex = pickupIndex;
  }


  public void setTargetPositionsByIndex(int pickupIndex) {
    // H! Get the pickup target positions
    Pose3d pickupPos = fieldPosManager.get3dFieldObjectPose(FieldPosManager.fieldSpot3d.centerFieldGamePieces, true, pickupIndex);
    Translation3d robotPosToScoringPos = pickupPos.getTranslation().minus(new Pose3d(fieldPosManager.getRobotPose()).getTranslation());
    Translation3d pivotToScoringPos = robotPosToScoringPos.plus(Constants.WristAndArm.pivotOffset);
    

    // ++ targets x, y, and z are the components of the translation3D between the arm pivot and the scoring position
    // ++ the X direction is in the length of the field 
    double targetX = pivotToScoringPos.getX();
    double targetY = pivotToScoringPos.getZ();

    LegAnklePosition newTargetPosition = LegAnkleSubsystem.IK(targetX, targetY, 0, 0);

    setTargets(newTargetPosition);
  }

  @Override
  public void initialize() {
    setTargetPositionsByIndex(pickupIndex);
    super.initialize();
  }
}