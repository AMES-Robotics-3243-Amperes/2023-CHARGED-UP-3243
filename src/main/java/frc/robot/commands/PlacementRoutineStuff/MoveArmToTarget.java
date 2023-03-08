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
import frc.robot.subsystems.LegAnkleSubsystem;

public class MoveArmToTarget extends CommandBase {
  private boolean isCube;
  private LegAnkleSubsystem legAnkleSubsystem;
  private FieldPosManager.fieldSpot3d target;

  private double targetX;
  private double targetY;
  private double targetPitch = 0;
  private double targetRoll = 0;
  private int targetIndex;
  private FieldPosManager fieldPosManager;
  private JoyUtil controller;

  private boolean isDone;


  /** Creates a new MoveArmToTarget. */
  public MoveArmToTarget(int targetIndex, boolean isCube, FieldPosManager.fieldSpot3d target, LegAnkleSubsystem legAnkleSubsystem, FieldPosManager fieldPosManager, JoyUtil controller) {
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
    targetX = pivotToScoringPos.getX();
    targetY = pivotToScoringPos.getZ();
  }

  



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    legAnkleSubsystem.setKinematicPositions(targetX, targetY, targetPitch, targetRoll);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return legAnkleSubsystem.isArmPositioned();
  }
}