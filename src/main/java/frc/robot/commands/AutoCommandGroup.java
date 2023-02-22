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

import java.util.ArrayList;
import java.util.List;

/**
 * <> The command that will be executed during the autonomous period
 */
public class AutoCommandGroup extends SequentialCommandGroup {
  protected final DriveSubsystem m_driveSubsystem;
  protected final FieldPosManager m_posManager;

  /**
   * <> creates a new {@link AutoCommandGroup}
   *
   * @param driveSubsystem the {@link DriveSubsystem} to control for driving
   */
  public AutoCommandGroup(DriveSubsystem driveSubsystem, FieldPosManager posManager) {
    m_driveSubsystem = driveSubsystem;
    addRequirements(m_driveSubsystem);

    m_posManager = posManager;

    // <> this is for all the auto move commands
    ProfiledPIDController thetaPidController = new ProfiledPIDController(
      Constants.DriveTrain.DriveConstants.AutoConstants.kTurningP,
      Constants.DriveTrain.DriveConstants.AutoConstants.kTurningI,
      Constants.DriveTrain.DriveConstants.AutoConstants.kTurningD,
      Constants.DriveTrain.DriveConstants.AutoConstants.kThetaControllerConstraints);
    thetaPidController.enableContinuousInput(-Math.PI, Math.PI);

    ArrayList<Command> autoCommands = new ArrayList<Command>();

    // TODO: get these correct
    boolean bottom = true;
    boolean charge = true;

    Translation2d nearChargeAvoidIntermediatePoint;
    Translation2d farChargeAvoidIntermediatePoint;

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

    // TODO: get ids and stuff from ShuffleBoard

    int piece0ID = 0;
    int piece1ID = 0;
    int piece1DropOffID = 0;
    int piece2ID = 1;
    int piece2DropOffID = 1;

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
    } else {
      // TODO: get these correct

      int finalPieceID = 0;
      int finalPieceDropOffID = 0;

      Pose2d pickupPieceDestination = m_posManager.get3dFieldObjectPose(
        FieldPosManager.fieldSpot3d.centerFieldGamePieces, true, finalPieceID).toPose2d();
      Pose2d dropOffPieceDestination = m_posManager.get3dFieldObjectPose(FieldPosManager.fieldSpot3d.highGrabberScoring,
        true, finalPieceDropOffID).toPose2d();

      // <> only add all the commands if neither of the ids are negative
      if (!(finalPieceID < 0 || finalPieceDropOffID < 0)) {
        SwerveAutoMoveCommand goToPieceCommand = new SwerveAutoMoveCommand(m_driveSubsystem,
          TrajectoryGenerator.generateTrajectory(m_driveSubsystem.getPose(),
            List.of(nearChargeAvoidIntermediatePoint, farChargeAvoidIntermediatePoint), pickupPieceDestination,
            DriveConstants.AutoConstants.trajectoryConfig), thetaPidController, false);
        autoCommands.add(goToPieceCommand);

        // TODO: add pickup command (talk to hale i think idk who's doing this)

        SwerveAutoMoveCommand goToDropOffCommand = new SwerveAutoMoveCommand(m_driveSubsystem,
          TrajectoryGenerator.generateTrajectory(m_driveSubsystem.getPose(),
            List.of(farChargeAvoidIntermediatePoint, nearChargeAvoidIntermediatePoint), dropOffPieceDestination,
            DriveConstants.AutoConstants.trajectoryConfig), thetaPidController, false);
        autoCommands.add(goToDropOffCommand);

        // TODO: add drop-off command (talk to hale i think idk who's doing this)
      }
    }

    addCommands((Command[]) autoCommands.toArray());
  }
}
