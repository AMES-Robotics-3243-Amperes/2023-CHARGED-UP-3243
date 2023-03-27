// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveTrain.DriveConstants;
import frc.robot.FieldPosManager;
import frc.robot.JoyUtil;
import frc.robot.commands.DriveTrain.BalanceCommand;
import frc.robot.commands.DriveTrain.LockSwerveWheelsCommand;
import frc.robot.commands.DriveTrain.SwerveAutoMoveCommand;
import frc.robot.commands.LegAnkle.MoveLegAnkleToNeutralPositionCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
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
  protected final GrabberSubsystem m_grabberSubsystem;
  protected final JoyUtil m_secondaryJoy;

  /**
   * <> creates a new {@link AutoCommandGroup}
   *
   * @param driveSubsystem the {@link DriveSubsystem} to control for driving
   */

  public AutoCommandGroup(DriveSubsystem driveSubsystem, LegAnkleSubsystem legAnkleSubsystem,
                          ShuffleboardSubsystem shuffleboardSubsystem, GrabberSubsystem grabberSubsystem,
                          FieldPosManager posManager, JoyUtil secondaryJoy) {
    m_driveSubsystem = driveSubsystem;
    m_legAnkleSubsystem = legAnkleSubsystem;
    m_grabberSubsystem = grabberSubsystem;
    addRequirements(m_driveSubsystem, legAnkleSubsystem, grabberSubsystem, shuffleboardSubsystem);

    m_posManager = posManager;
    m_shuffleboardSubsystem = shuffleboardSubsystem;
    m_secondaryJoy = secondaryJoy;

    ArrayList<Command> autoCommands = new ArrayList<>();

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

    // TODO: make the pickup correct (be sure to shift them)
    Pose2d dropOffPiece0Destination = m_posManager.get2dFieldObjectPose(FieldPosManager.fieldSpot2d.scoringPosition,
      true, piece0DropOffID);

    Pose2d pickupPiece1Position = m_posManager.get2dFieldObjectPose(FieldPosManager.fieldSpot2d.scoringPosition, true,
      piece1ID);
    Pose2d dropOffPiece1Destination = m_posManager.get2dFieldObjectPose(FieldPosManager.fieldSpot2d.scoringPosition,
      true, piece1DropOffID);

    Pose2d pickupPiece2Position = m_posManager.get2dFieldObjectPose(FieldPosManager.fieldSpot2d.scoringPosition, true,
      piece2ID);
    Pose2d dropOffPiece2Destination = m_posManager.get2dFieldObjectPose(FieldPosManager.fieldSpot2d.scoringPosition,
      true, piece2DropOffID);

    // <> only add all the commands if neither of the ids are negative
    if (piece0DropOffID >= 0) {
      SwerveAutoMoveCommand goToPieceDropOffCommand = new SwerveAutoMoveCommand(m_driveSubsystem,
        dropOffPiece0Destination);
      autoCommands.add(goToPieceDropOffCommand);

      addNewPlacementRoutine(legAnkleSubsystem, autoCommands);
    }

    // <> only add all the commands if neither of the ids are negative
    if (piece1ID >= 0 && piece1DropOffID >= 0) {
      SwerveAutoMoveCommand goToPieceCommand = new SwerveAutoMoveCommand(m_driveSubsystem, new ArrayList<>(
        List.of(nearChargeAvoidIntermediatePoint, farChargeAvoidIntermediatePoint, pickupPiece1Position)));
      autoCommands.add(goToPieceCommand);

      addNewPickupRoutine(legAnkleSubsystem, grabberSubsystem, autoCommands);

      SwerveAutoMoveCommand goToPieceDropOffCommand = new SwerveAutoMoveCommand(m_driveSubsystem, new ArrayList<>(
        List.of(farChargeAvoidIntermediatePoint, nearChargeAvoidIntermediatePoint, dropOffPiece1Destination)));
      autoCommands.add(goToPieceDropOffCommand);

      addNewPlacementRoutine(legAnkleSubsystem, autoCommands);
    }

    // <> only add all the commands if neither of the ids are negative
    if (piece2ID >= 0 && piece2DropOffID >= 0) {
      SwerveAutoMoveCommand goToPieceCommand = new SwerveAutoMoveCommand(m_driveSubsystem, new ArrayList<>(
        List.of(nearChargeAvoidIntermediatePoint, farChargeAvoidIntermediatePoint, pickupPiece2Position)));
      autoCommands.add(goToPieceCommand);

      addNewPickupRoutine(legAnkleSubsystem, grabberSubsystem, autoCommands);

      SwerveAutoMoveCommand goToPieceDropOffCommand = new SwerveAutoMoveCommand(m_driveSubsystem, new ArrayList<>(
        List.of(farChargeAvoidIntermediatePoint, nearChargeAvoidIntermediatePoint, dropOffPiece2Destination)));
      autoCommands.add(goToPieceDropOffCommand);

      addNewPlacementRoutine(legAnkleSubsystem, autoCommands);
    }

    if (charge) {
      Pose2d bottomNearChargeCorner = m_posManager.getAutoPose(FieldPosManager.autoPath.lowerPath, true, 0);
      Pose2d topFarChargeCorner = m_posManager.getAutoPose(FieldPosManager.autoPath.upperPath, true, 1);
      Pose2d chargePoint = bottomNearChargeCorner.plus(new Transform2d(bottomNearChargeCorner, topFarChargeCorner))
        .div(2);

      Pose2d intermediatePoint = new Pose2d(
        new Translation2d(nearChargeAvoidIntermediatePoint.getX(), chargePoint.getY()),
        nearChargeAvoidIntermediatePoint.getRotation());

      SwerveAutoMoveCommand getToPosCommand = new SwerveAutoMoveCommand(m_driveSubsystem,
        new ArrayList<>(List.of(intermediatePoint, chargePoint)),
        DriveConstants.AutoConstants.kMaxLenientMetersFromGoal,
        DriveConstants.AutoConstants.kMaxLenientRotationFromGoal);

      autoCommands.add(getToPosCommand);
      autoCommands.add(new BalanceCommand(m_driveSubsystem));
      autoCommands.add(new LockSwerveWheelsCommand(m_driveSubsystem));
    }

    Command[] commandArray = new Command[autoCommands.size()];
    commandArray = autoCommands.toArray(commandArray);
    addCommands(commandArray);
  }

  private static void addNewPickupRoutine(LegAnkleSubsystem legAnkleSubsystem, GrabberSubsystem grabberSubsystem,
                                          ArrayList<Command> autoCommands) {
    SequentialCommandGroup pickupCommand = new SequentialCommandGroup(new MoveArmToPickupTargetAuto(legAnkleSubsystem),
      new PickupGameObjectAuto(grabberSubsystem));

    autoCommands.add(pickupCommand);
  }

  private void addNewPlacementRoutine(LegAnkleSubsystem legAnkleSubsystem, ArrayList<Command> autoCommands) {
    SequentialCommandGroup dropOffCommand = new SequentialCommandGroup(
      new MoveArmToPlaceTargetAuto(legAnkleSubsystem), new ReleaseGameObjectAuto(m_grabberSubsystem, m_secondaryJoy), new MoveLegAnkleToNeutralPositionCommand(legAnkleSubsystem));

    autoCommands.add(dropOffCommand);
  }
}
