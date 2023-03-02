// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrain.DriveConstants;
import frc.robot.FieldPosManager;
import frc.robot.FieldPosManager.fieldSpot2d;
import frc.robot.subsystems.DriveSubsystem;

import java.util.List;

public class TempAutoRoutine extends SequentialCommandGroup {
  /**
   * Creates a new TempAutoRoutine.
   */
  public TempAutoRoutine(FieldPosManager posManager, DriveSubsystem subsystem, ProfiledPIDController pidController) {
    Translation2d fieldCenter = posManager.get2dFieldObjectPose(fieldSpot2d.chargeStationBottomLeft, true, 0)
      .getTranslation()
      .plus(posManager.get2dFieldObjectPose(fieldSpot2d.chargeStationTopRight, true, 0).getTranslation())
      .plus(posManager.get2dFieldObjectPose(fieldSpot2d.chargeStationBottomLeft, false, 0).getTranslation())
      .plus(posManager.get2dFieldObjectPose(fieldSpot2d.chargeStationTopRight, false, 0).getTranslation()).times(0.25);

    SwerveAutoMoveCommand m_MoveCommand = new SwerveAutoMoveCommand(subsystem,
      TrajectoryGenerator.generateTrajectory(Constants.robotStartingPosition, List.of(new Translation2d(
        (posManager.get2dFieldObjectPose(fieldSpot2d.chargeStationBottomLeft, true, 0).getTranslation()
          .getX() + posManager.get2dFieldObjectPose(fieldSpot2d.chargeStationTopRight, true, 0).getTranslation()
          .getX()) / 2, posManager.get2dFieldObjectPose(fieldSpot2d.chargeStationBottomLeft, true, 0).getTranslation()
        .plus(new Translation2d(0, -1)).getY()), fieldCenter.plus(new Translation2d(0, -1.5))), new Pose2d(
        posManager.get2dFieldObjectPose(fieldSpot2d.chargeStationBottomLeft, true, 0).getTranslation()
          .plus(posManager.get2dFieldObjectPose(fieldSpot2d.chargeStationTopRight, true, 0).getTranslation())
          .times(0.5), Constants.robotStartingPosition.getRotation()), DriveConstants.AutoConstants.trajectoryConfig),
      pidController, isFinished());

    InstantCommand m_LockWheelsCommand = new InstantCommand(subsystem::setX);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(m_MoveCommand, m_LockWheelsCommand);
  }
}
