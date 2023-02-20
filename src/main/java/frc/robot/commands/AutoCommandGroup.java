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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrain.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.ArrayList;
import java.util.List;

/**
 * <> The command that will be executed during the autonomous period
 */
public class AutoCommandGroup extends SequentialCommandGroup {
  protected final DriveSubsystem m_driveSubsystem;

  /**
   * <> creates a new {@link AutoCommandGroup}
   *
   * @param driveSubsystem the {@link DriveSubsystem} to control for driving
   */
  public AutoCommandGroup(DriveSubsystem driveSubsystem) {
    m_driveSubsystem = driveSubsystem;
    addRequirements(m_driveSubsystem);

    // <> this is for all the auto move commands
    ProfiledPIDController thetaPidController = new ProfiledPIDController(
      Constants.DriveTrain.DriveConstants.AutoConstants.kTurningP,
      Constants.DriveTrain.DriveConstants.AutoConstants.kTurningI,
      Constants.DriveTrain.DriveConstants.AutoConstants.kTurningD,
      Constants.DriveTrain.DriveConstants.AutoConstants.kThetaControllerConstraints);
    thetaPidController.enableContinuousInput(-Math.PI, Math.PI);

    ArrayList<Command> autoCommands = new ArrayList<Command>();

    // TODO: get these correct
    boolean left = true;
    boolean charge = true;

    Translation2d chargeStationBottomIntermediatePoint;
    Translation2d chargeStationTopIntermediatePoint;

    if (left) {
      chargeStationBottomIntermediatePoint = new Translation2d(10, 10);
      chargeStationTopIntermediatePoint = new Translation2d(10, 20);
    } else {
      chargeStationBottomIntermediatePoint = new Translation2d(30, 10);
      chargeStationTopIntermediatePoint = new Translation2d(30, 10);
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
    } else {
      // TODO: get these correct

      int finalPieceID = 0;
      int finalPieceDropOffID = 0;

      Pose2d pickupPieceDestination = new Pose2d();
      Pose2d dropOffPieceDestination = new Pose2d();

      if (!(finalPieceID < 0 || finalPieceDropOffID < 0)) {
        SwerveAutoMoveCommand goToPieceCommand = new SwerveAutoMoveCommand(m_driveSubsystem,
          TrajectoryGenerator.generateTrajectory(m_driveSubsystem.getPose(),
            List.of(chargeStationBottomIntermediatePoint, chargeStationTopIntermediatePoint), pickupPieceDestination,
            DriveConstants.AutoConstants.trajectoryConfig), thetaPidController, false);
        autoCommands.add(goToPieceCommand);

        // TODO: add pickup command (talk to hale i think idk who's doing this)

        SwerveAutoMoveCommand goToDropOffCommand = new SwerveAutoMoveCommand(m_driveSubsystem,
          TrajectoryGenerator.generateTrajectory(m_driveSubsystem.getPose(),
            List.of(chargeStationTopIntermediatePoint, chargeStationBottomIntermediatePoint), dropOffPieceDestination,
            DriveConstants.AutoConstants.trajectoryConfig), thetaPidController, false);
        autoCommands.add(goToDropOffCommand);

        // TODO: add drop-off command (talk to hale i think idk who's doing this)
      }
    }

    addCommands((Command[]) autoCommands.toArray());
  }
}
