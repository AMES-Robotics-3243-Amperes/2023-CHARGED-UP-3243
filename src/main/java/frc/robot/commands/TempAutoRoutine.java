// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.FieldPosManager;
import frc.robot.Constants.DriveTrain.DriveConstants;
import frc.robot.FieldPosManager.fieldSpot2d;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;

public class TempAutoRoutine extends SequentialCommandGroup {
  /** Creates a new TempAutoRoutine. */
  public TempAutoRoutine(FieldPosManager posManager, DriveSubsystem subsystem, ProfiledPIDController thetaPidController, PhotonVisionSubsystem photonSub) {

    // boolean gotCorrectPosition = true;
    // Timer timer = new Timer();
    // timer.restart();

    //addCommands(new InstantCommand(() -> subsystem.drive(-0.15, 0, 0, false), subsystem), new WaitCommand(1), new InstantCommand(() -> subsystem.drive(0, 0, 0, false), subsystem));
    addCommands(new DriveOntoCharge(subsystem), new BalanceCommand(subsystem));

    // while (!photonSub.seeingApriltag()) {
    //   if (timer.hasElapsed(3)) YU8
    //     gotCorrectPosition = false;
    //     break;
    //   }
    // }

    // Translation2d fieldCenter = posManager.get2dFieldObjectPose(fieldSpot2d.chargeStationBottomLeft, true, 0)
    //   .getTranslation()
    //   .plus(posManager.get2dFieldObjectPose(fieldSpot2d.chargeStationTopRight, true, 0).getTranslation())
    //   .plus(posManager.get2dFieldObjectPose(fieldSpot2d.chargeStationBottomLeft, false, 0).getTranslation())
    //   .plus(posManager.get2dFieldObjectPose(fieldSpot2d.chargeStationTopRight, false, 0).getTranslation().times(0.25));

    // Translation2d chargeCenter = posManager.get2dFieldObjectPose(fieldSpot2d.chargeStationBottomLeft, true, 0).getTranslation()
    //   .plus(posManager.get2dFieldObjectPose(fieldSpot2d.chargeStationTopRight, true, 0).getTranslation()).times(0.5);

    // // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(subsystem.getPose(), List.of(
    // //   new Translation2d((posManager.get2dFieldObjectPose(fieldSpot2d.chargeStationBottomLeft, true, 0).getTranslation()
    // //     .getX() + posManager.get2dFieldObjectPose(fieldSpot2d.chargeStationTopRight, true, 0).getTranslation()
    // //     .getX()) / 2, posManager.get2dFieldObjectPose(fieldSpot2d.chargeStationBottomLeft, true, 0).getTranslation()
    // //     .plus(new Translation2d(0, -1)).getY()), fieldCenter.plus(new Translation2d(0, -1))), new Pose2d(
    // //   posManager.get2dFieldObjectPose(fieldSpot2d.chargeStationTopRight, true, 0).getTranslation()
    // //     .plus(posManager.get2dFieldObjectPose(fieldSpot2d.chargeStationTopRight, true, 0).getTranslation()).times(0.5),
    // //   subsystem.getPose().getRotation()), DriveConstants.AutoConstants.trajectoryConfig);

    // SwerveAutoMoveCommand m_MoveCommand = new SwerveAutoMoveCommand(subsystem, new Pose2d(chargeCenter, subsystem.getHeading()), thetaPidController, false);
    // InstantCommand m_LockWheelCommand = new InstantCommand(subsystem::setX);

    // // Add your commands in the addCommands() call, e.g.
    // // addCommands(new FooCommand(), new BarCommand());
    // if (gotCorrectPosition) {
    //   addCommands(m_MoveCommand, m_LockWheelCommand);
    // } else {
    //   addCommands();
    // }
  }
}
