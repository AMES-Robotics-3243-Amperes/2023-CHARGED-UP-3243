// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PlacementRoutineStuff;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.FieldPosManager;
import frc.robot.JoyUtil;
import frc.robot.commands.MoveLegAnkleToPositionCommand;
import frc.robot.subsystems.LegAnkleSubsystem;
import frc.robot.utility_classes.LegAnklePosition;

public class MoveArmToTarget extends MoveLegAnkleToPositionCommand {
  private boolean isCube;
  private FieldPosManager.fieldSpot3d target;

  private int targetIndex;
  private FieldPosManager fieldPosManager;
  private JoyUtil controller;

  /** Creates a new MoveArmToTarget. */
  public MoveArmToTarget(int targetIndex, boolean isCube, FieldPosManager.fieldSpot3d target, LegAnkleSubsystem legAnkleSubsystem, FieldPosManager fieldPosManager, JoyUtil controller) {
    super(legAnkleSubsystem);
    this.isCube = isCube;
    this.legAnkleSubsystem = legAnkleSubsystem;
    this.target = target;
    this.targetIndex = targetIndex;
    this.fieldPosManager = fieldPosManager;
    this.controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(legAnkleSubsystem);

    setTargetPositions();
  }


  public void setTargetPositions() {
    // H! Get the scoring target positions
    Pose3d scoringPos = fieldPosManager.get3dFieldObjectPose(target, isCube, targetIndex);
    Translation3d robotPosToScoringPos = scoringPos.getTranslation().minus(new Pose3d(fieldPosManager.getRobotPose()).getTranslation());
    Translation3d pivotToScoringPos = robotPosToScoringPos.plus(Constants.WristAndArm.pivotOffset);
    

    // ++ targets x, y, and z are the components of the translation3D between the arm pivot and the scoring position
    // ++ the X direction is in the length of the field 
    double targetX = pivotToScoringPos.getX();
    double targetY = pivotToScoringPos.getZ();

    LegAnklePosition newTargetPosition = LegAnkleSubsystem.IK(targetX, targetY, 0, 0);

    targetExtension = newTargetPosition.extension;
    targetPivot = newTargetPosition.pivot;
    targetPitch = newTargetPosition.pitch;
    targetRoll = newTargetPosition.roll;
  }

  
  @Override
  public void execute() {
    // H! If the DPAD is up, go to the high goal. If it's in the middle, go to the middle goal. If it's down, go to the low goal.
    if (controller.getPOVUp() ||  controller.getPOVUpRight() || controller.getPOVUpLeft()) {
      target = FieldPosManager.fieldSpot3d.highGrabberScoring;      
    } else if (controller.getPOVDown() ||  controller.getPOVDownRight() || controller.getPOVDownLeft()) {
      target = FieldPosManager.fieldSpot3d.lowGrabberScoring;      
    } else {
      target = FieldPosManager.fieldSpot3d.middleGrabberScoring;
    }

    setTargetPositions();
    
    super.execute();
  }
}