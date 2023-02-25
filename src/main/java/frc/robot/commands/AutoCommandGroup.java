// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrain.DriveConstants;
import frc.robot.FieldPosManager;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShuffleboardSubsystem;

import java.util.ArrayList;
import java.util.List;

/**
 * <> The command that will be executed during the autonomous period
 */
public class AutoCommandGroup extends SequentialCommandGroup {
  protected final DriveSubsystem m_driveSubsystem;
  protected final FieldPosManager m_posManager;
  protected final ShuffleboardSubsystem m_shuffleboardSubsystem;

  /**
   * <> creates a new {@link AutoCommandGroup}
   *
   * @param driveSubsystem the {@link DriveSubsystem} to control for driving
   */
  public AutoCommandGroup(DriveSubsystem driveSubsystem, ShuffleboardSubsystem shuffleboardSubsystem,
                          FieldPosManager posManager) {
    m_driveSubsystem = driveSubsystem;
    addRequirements(m_driveSubsystem);

    m_posManager = posManager;

    m_shuffleboardSubsystem = shuffleboardSubsystem;

    // <> this is for all the auto move commands
    ProfiledPIDController thetaPidController = new ProfiledPIDController(
      Constants.DriveTrain.DriveConstants.AutoConstants.kTurningP,
      Constants.DriveTrain.DriveConstants.AutoConstants.kTurningI,
      Constants.DriveTrain.DriveConstants.AutoConstants.kTurningD,
      Constants.DriveTrain.DriveConstants.AutoConstants.kThetaControllerConstraints);
    thetaPidController.enableContinuousInput(-Math.PI, Math.PI);

    ArrayList<Command> autoCommands = new ArrayList<Command>();

    boolean bottom = m_shuffleboardSubsystem.ShuffleBoardBooleanInput(
      ShuffleboardSubsystem.ShuffleBoardInput.goLowerRoute);
    boolean charge = m_shuffleboardSubsystem.ShuffleBoardBooleanInput(
      ShuffleboardSubsystem.ShuffleBoardInput.goChargeStation);

    Translation2d nearChargeAvoidIntermediatePoint;
    Translation2d farChargeAvoidIntermediatePoint;

    // <> figure out the intermediate points that will be used to avoid
    // the charge station when going from one side of it to the other
    if (bottom) {
      nearChargeAvoidIntermediatePoint = m_posManager.getAutoPose(FieldPosManager.autoPath.lowerPath, true, 0)
        .getTranslation();
      farChargeAvoidIntermediatePoint = m_posManager.getAutoPose(FieldPosManager.autoPath.lowerPath, true, 1)
        .getTranslation();
    } else {
      nearChargeAvoidIntermediatePoint = m_posManager.getAutoPose(FieldPosManager.autoPath.upperPath, true, 0)
        .getTranslation();
      farChargeAvoidIntermediatePoint = m_posManager.getAutoPose(FieldPosManager.autoPath.upperPath, true, 1)
        .getTranslation();
    }

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

    // <> get the pickup and drop off locations of all the pieces
    // TODO: shift these so that the robot doesn't drive over the piece
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
      SwerveAutoMoveCommand goToPieceDropOffCommand = new SwerveAutoMoveCommand(m_driveSubsystem,
        TrajectoryGenerator.generateTrajectory(m_driveSubsystem.getPose(),
          List.of(farChargeAvoidIntermediatePoint, nearChargeAvoidIntermediatePoint), dropOffPiece0Destination,
          DriveConstants.AutoConstants.trajectoryConfig), thetaPidController, false);
      autoCommands.add(goToPieceDropOffCommand);

      // TODO: add drop-off command (talk to hale i think idk who's doing this)
    }

    // <> only add all the commands if neither of the ids are negative
    if (!(piece1ID < 0 || piece1DropOffID < 0)) {
      SwerveAutoMoveCommand goToPieceCommand = new SwerveAutoMoveCommand(m_driveSubsystem,
        TrajectoryGenerator.generateTrajectory(m_driveSubsystem.getPose(),
          List.of(nearChargeAvoidIntermediatePoint, farChargeAvoidIntermediatePoint), pickupPiece1Position,
          DriveConstants.AutoConstants.trajectoryConfig), thetaPidController, false);
      autoCommands.add(goToPieceCommand);

      // TODO: add pickup command (talk to hale i think idk who's doing this)

      SwerveAutoMoveCommand goToPieceDropOffCommand = new SwerveAutoMoveCommand(m_driveSubsystem,
        TrajectoryGenerator.generateTrajectory(m_driveSubsystem.getPose(),
          List.of(farChargeAvoidIntermediatePoint, nearChargeAvoidIntermediatePoint), dropOffPiece1Destination,
          DriveConstants.AutoConstants.trajectoryConfig), thetaPidController, false);
      autoCommands.add(goToPieceDropOffCommand);

      // TODO: add drop-off command (talk to hale i think idk who's doing this)
    }

    // <> only add all the commands if neither of the ids are negative
    if (!(piece2ID < 0 || piece2DropOffID < 0)) {
      SwerveAutoMoveCommand goToPieceCommand = new SwerveAutoMoveCommand(m_driveSubsystem,
        TrajectoryGenerator.generateTrajectory(m_driveSubsystem.getPose(),
          List.of(nearChargeAvoidIntermediatePoint, farChargeAvoidIntermediatePoint), pickupPiece2Position,
          DriveConstants.AutoConstants.trajectoryConfig), thetaPidController, false);
      autoCommands.add(goToPieceCommand);

      // TODO: add pickup command (talk to hale i think idk who's doing this)

      SwerveAutoMoveCommand goToPieceDropOffCommand = new SwerveAutoMoveCommand(m_driveSubsystem,
        TrajectoryGenerator.generateTrajectory(m_driveSubsystem.getPose(),
          List.of(farChargeAvoidIntermediatePoint, nearChargeAvoidIntermediatePoint), dropOffPiece2Destination,
          DriveConstants.AutoConstants.trajectoryConfig), thetaPidController, false);
      autoCommands.add(goToPieceDropOffCommand);

      // TODO: add drop-off command (talk to hale i think idk who's doing this)
    }

    if (charge) {
      // TODO: get these correct

      Pose2d chargePoint = new Pose2d(new Translation2d(40, 40), new Rotation2d(0));
      Translation2d intermediatePoint = new Translation2d(20, 40);
      boolean doBalance = true;

      SwerveAutoMoveCommand getToPosCommand = new SwerveAutoMoveCommand(m_driveSubsystem,
        TrajectoryGenerator.generateTrajectory(m_driveSubsystem.getPose(), List.of(intermediatePoint), chargePoint,
          DriveConstants.AutoConstants.trajectoryConfig), thetaPidController, false);

      autoCommands.add(getToPosCommand);

      if (doBalance) {
        autoCommands.add(new BalanceCommand(m_driveSubsystem));
      }

      autoCommands.add(Commands.runOnce(m_driveSubsystem::setX));
    }

    addCommands((Command[]) autoCommands.toArray());
  }
}
