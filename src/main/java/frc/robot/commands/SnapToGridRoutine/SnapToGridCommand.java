// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SnapToGridRoutine;

import frc.robot.Constants.DriveTrain.DriveConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldPosManager;
import frc.robot.JoyUtil;
import frc.robot.commands.DriveTrain.SwerveAutoMoveCommand;
import frc.robot.subsystems.*;

public class SnapToGridCommand extends SwerveAutoMoveCommand { // :D hi after looking over this, it looks like due to the fact that only one command
  private final FieldPosManager m_FieldPosManager;
  private final JoyUtil m_PrimaryController;
  public int index;
  private boolean lastPOVLeft = false;
  private boolean lastPOVRight = false;

  /** Creates a new SnapToGridCommand. 
   * 
   * This is a command that will 'snap' the robot to the nearest of the 9 friendly scoring positions, and then allow t
   * 
   * @param driveSubsystem The Drive Subsystem
   * @param fieldPosManager The Field Position Manager
   * @param JoyUtil The Primary Controller
   * @param JoyUtil The Secondary Controller
  */
  public SnapToGridCommand(DriveSubsystem driveSubsystem, FieldPosManager fieldPosManager, JoyUtil primaryController, JoyUtil secondaryController) {
    super(
      driveSubsystem, 
      fieldPosManager.get2dFieldObjectPose(FieldPosManager.fieldSpot2d.scoringPosition, true, fieldPosManager.getNearestScoringZoneIndex()), 
      DriveConstants.AutoConstants.kMaxMetersFromGoal, 
      DriveConstants.AutoConstants.kMaxRotationFromGoal
    );

    m_FieldPosManager = fieldPosManager;
    m_PrimaryController = primaryController;

    // ss make an auto move command from the drive subsystem, the Pose2d corresponding to the index, and some constants
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    index = m_FieldPosManager.getNearestScoringZoneIndex();
    changeGoal(m_FieldPosManager.get2dFieldObjectPose(FieldPosManager.fieldSpot2d.scoringPosition, true, index));
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_PrimaryController.getPOVRight() && !lastPOVRight){
      // ss if blue, right means subtract 1
      if (m_FieldPosManager.allianceColor == Alliance.Blue) {
        index--;
      } // ss if red, right means add 1
      else if (m_FieldPosManager.allianceColor == Alliance.Red) {
        index++;
      }
      index = (int)MathUtil.inputModulus(index, 0, 8);

      // ss change the goal to the new index
      changeGoal(m_FieldPosManager.get2dFieldObjectPose(FieldPosManager.fieldSpot2d.scoringPosition, true, index));
    }
    if(m_PrimaryController.getPOVLeft() && !lastPOVLeft){
      // ss if blue, left means add 1
      if (m_FieldPosManager.allianceColor == Alliance.Blue) {
        index++;
      } // ss if red, left means subtract 1
      else if (m_FieldPosManager.allianceColor == Alliance.Red) {
        index--;
      }
      index = (int)MathUtil.inputModulus(index, 0, 8);

      // ss change the goal to the new index
      changeGoal(m_FieldPosManager.get2dFieldObjectPose(FieldPosManager.fieldSpot2d.scoringPosition, true, index));
    }
    lastPOVRight = m_PrimaryController.getPOVRight();
    lastPOVLeft = m_PrimaryController.getPOVLeft();

    super.execute();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // ss stops everything if the movement joysticks move. These functions apply deadzone automatically
    if (m_PrimaryController.getLeftX() != 0 || m_PrimaryController.getLeftY() != 0 || m_PrimaryController.getRightX() != 0 || m_PrimaryController.getRightY() != 0) { // use Math.abs() and also check RightY joystick
      return true;
    }

    return false;
  }
}
