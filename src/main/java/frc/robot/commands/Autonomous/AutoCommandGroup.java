// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveTrain.DriveConstants;
import frc.robot.commands.DriveTrain.BalanceCommand;
import frc.robot.commands.DriveTrain.LockSwerveWheelsCommand;
import frc.robot.commands.DriveTrain.SwerveAutoMoveCommand;
import frc.robot.FieldPosManager;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LegAnkleSubsystem;
import frc.robot.subsystems.ShuffleboardSubsystem;

import java.util.ArrayList;
import java.util.List;

/**
 * <> The command that will be executed during the autonomous period
 */
 
public class AutoCommandGroup extends SequentialCommandGroup {
  protected final DriveSubsystem m_driveSubsystem;
  protected final LegAnkleSubsystem m_legAnkleSubsystem;
  protected final FieldPosManager m_posManager;
  protected final ShuffleboardSubsystem m_shuffleboardSubsystem;

  /**
   * <> creates a new {@link AutoCommandGroup}
   *
   * @param driveSubsystem the {@link DriveSubsystem} to control for driving
   */
   
  public AutoCommandGroup(DriveSubsystem driveSubsystem, LegAnkleSubsystem legAnkleSubsystem,
                          ShuffleboardSubsystem shuffleboardSubsystem, FieldPosManager posManager) {
    m_driveSubsystem = driveSubsystem;
    m_legAnkleSubsystem = legAnkleSubsystem;
    addRequirements(m_driveSubsystem, legAnkleSubsystem);

    m_posManager = posManager;
    m_shuffleboardSubsystem = shuffleboardSubsystem;

    ArrayList<Command> autoCommands = new ArrayList<Command>();

    boolean bottom = m_shuffleboardSubsystem.ShuffleBoardBooleanInput(
      ShuffleboardSubsystem.ShuffleBoardInput.goLowerRoute);
    boolean charge = m_shuffleboardSubsystem.ShuffleBoardBooleanInput(
      ShuffleboardSubsystem.ShuffleBoardInput.goChargeStation);

    Pose2d nearChargeAvoidIntermediatePoint;
    Pose2d farChargeAvoidIntermediatePoint;

    // <> figure out the intermediate points that will be used to avoid
    // the charge station when going from one side of it to the other
    if (bottom) {
      nearChargeAvoidIntermediatePoint = m_posManager.getAutoPose(FieldPosManager.autoPath.lowerPath, true, 0);
      farChargeAvoidIntermediatePoint = m_posManager.getAutoPose(FieldPosManager.autoPath.lowerPath, true, 1);
    } else {
      nearChargeAvoidIntermediatePoint = m_posManager.getAutoPose(FieldPosManager.autoPath.upperPath, true, 0);
      farChargeAvoidIntermediatePoint = m_posManager.getAutoPose(FieldPosManager.autoPath.upperPath, true, 1);
    }

    // <> get the pickup and drop off locations of all the pieces
    int piece0DropOffID = (int) m_shuffleboardSubsystem.ShuffleBoardNumberInput(
      ShuffleboardSubsystem.ShuffleBoardInput.piece0Place);
    int piece1ID = (int) m_shuffleboardSubsystem.ShuffleBoardNumberInput(
      ShuffleboardSubsystem.ShuffleBoardInput.piece1Pickup);
    int piece1DropOffID = (int) m_shuffleboardSubsystem.ShuffleBoardNumberInput(
      ShuffleboardSubsystem.ShuffleBoardInput.piece1Place);
    int piece2ID = (int) m_shuffleboardSubsystem.ShuffleBoardNumberInput(
      ShuffleboardSubsystem.ShuffleBoardInput.piece2Pickup);
    int piece2DropOffID = (int) m_shuffleboardSubsystem.ShuffleBoardNumberInput(
      ShuffleboardSubsystem.ShuffleBoardInput.piece2Place);

    // TODO: shift these so that the robot doesn't drive into things
    Pose2d dropOffPiece0Destination = m_posManager.get3dFieldObjectPose(FieldPosManager.fieldSpot3d.highGrabberScoring,
      true, piece0DropOffID).toPose2d();

    Pose2d pickupPiece1Position = m_posManager.get3dFieldObjectPose(FieldPosManager.fieldSpot3d.centerFieldGamePieces,
      true, piece1ID).toPose2d();
    Pose2d dropOffPiece1Destination = m_posManager.get3dFieldObjectPose(FieldPosManager.fieldSpot3d.highGrabberScoring,
      true, piece1DropOffID).toPose2d();

    Pose2d pickupPiece2Position = m_posManager.get3dFieldObjectPose(FieldPosManager.fieldSpot3d.centerFieldGamePieces,
      true, piece2ID).toPose2d();
    Pose2d dropOffPiece2Destination = m_posManager.get3dFieldObjectPose(FieldPosManager.fieldSpot3d.highGrabberScoring,
      true, piece2DropOffID).toPose2d();

    // <> only add all the commands if neither of the ids are negative
    if (piece0DropOffID >= 0) {
      SwerveAutoMoveCommand goToPieceDropOffCommand = new SwerveAutoMoveCommand(m_driveSubsystem, dropOffPiece0Destination);
      autoCommands.add(goToPieceDropOffCommand);

      // TODO: add drop-off command (talk to hale i think idk who's doing this)
    }

    // <> only add all the commands if neither of the ids are negative
    if (piece1ID >= 0 && piece1DropOffID >= 0) {
      SwerveAutoMoveCommand goToPieceCommand = new SwerveAutoMoveCommand(m_driveSubsystem, new ArrayList<Pose2d>(List.of(nearChargeAvoidIntermediatePoint, farChargeAvoidIntermediatePoint, pickupPiece1Position)));
      autoCommands.add(goToPieceCommand);

      // TODO: add pickup command (talk to hale i think idk who's doing this)

      SwerveAutoMoveCommand goToPieceDropOffCommand = new SwerveAutoMoveCommand(m_driveSubsystem, new ArrayList<>(List.of(farChargeAvoidIntermediatePoint, nearChargeAvoidIntermediatePoint, dropOffPiece1Destination)));
      autoCommands.add(goToPieceDropOffCommand);

      // TODO: add drop-off command (talk to hale i think idk who's doing this)
    }

    // <> only add all the commands if neither of the ids are negative
    if (piece2ID >= 0 && piece2DropOffID >= 0) {
      SwerveAutoMoveCommand goToPieceCommand = new SwerveAutoMoveCommand(m_driveSubsystem, new ArrayList<>(List.of(nearChargeAvoidIntermediatePoint, farChargeAvoidIntermediatePoint, pickupPiece2Position)));
      autoCommands.add(goToPieceCommand);

      // TODO: add pickup command (talk to hale i think idk who's doing this)

      SwerveAutoMoveCommand goToPieceDropOffCommand = new SwerveAutoMoveCommand(m_driveSubsystem, new ArrayList<>(List.of(farChargeAvoidIntermediatePoint, nearChargeAvoidIntermediatePoint, dropOffPiece2Destination)));
      autoCommands.add(goToPieceDropOffCommand);

      // TODO: add drop-off command (talk to hale i think idk who's doing this)
    }

    if (charge) {
      // TODO: get these correct

      Pose2d bottomNearChargeCorner = m_posManager.getAutoPose(FieldPosManager.autoPath.lowerPath, true, 0);
      Pose2d topFarChargeCorner = m_posManager.getAutoPose(FieldPosManager.autoPath.upperPath, true, 1);
      Pose2d chargePoint =  bottomNearChargeCorner.plus(new Transform2d(bottomNearChargeCorner, topFarChargeCorner)).div(2);

      Pose2d intermediatePoint = new Pose2d();
      boolean doBalance = true;

      SwerveAutoMoveCommand getToPosCommand = new SwerveAutoMoveCommand(m_driveSubsystem, new ArrayList<>(List.of(intermediatePoint, chargePoint)),
        DriveConstants.AutoConstants.kMaxLenientMetersFromGoal, DriveConstants.AutoConstants.kMaxLenientRotationFromGoal);

      autoCommands.add(getToPosCommand);

      if (doBalance) {
        autoCommands.add(new BalanceCommand(m_driveSubsystem));
      }

      autoCommands.add(new LockSwerveWheelsCommand(m_driveSubsystem));
    }

    addCommands((Command[]) autoCommands.toArray());
  }
}
