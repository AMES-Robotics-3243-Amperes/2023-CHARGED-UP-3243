// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveTrain.DriveConstants;
import frc.robot.FieldPosManager;
import frc.robot.commands.DriveTrain.DriveOntoChargeCommand;
import frc.robot.commands.DriveTrain.SwerveAutoMoveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShuffleboardSubsystem;

import java.util.ArrayList;
import java.util.List;

/**
 * <> a basic command that drives out of the community and to
 * the back of the charge station in a positition safe
 * to schedule a {@link DriveOntoChargeCommand} from
 */
public class DriveBehindChargeCommand extends SequentialCommandGroup {
  /**
   * Creates a new DriveBehindChargeCommand.
   */
  public DriveBehindChargeCommand(DriveSubsystem driveSubsystem, ShuffleboardSubsystem shuffleboardSubsystem,
                                  FieldPosManager posManager) {
    // <> figure out if lower route is true from shuffleboard subsystem
    boolean bottom = shuffleboardSubsystem.ShuffleBoardBooleanInput(
      ShuffleboardSubsystem.ShuffleBoardInput.goLowerRoute);

    Pose2d nearChargeAvoidIntermediatePoint;
    Pose2d farChargeAvoidIntermediatePoint;

    // <> figure out the intermediate points that will be used to avoid
    // the charge station when going from one side of it to the other
    if (bottom) {
      nearChargeAvoidIntermediatePoint = posManager.getAutoPose(FieldPosManager.autoPath.lowerPath, true, 0);
      farChargeAvoidIntermediatePoint = posManager.getAutoPose(FieldPosManager.autoPath.lowerPath, true, 1);
    } else {
      nearChargeAvoidIntermediatePoint = posManager.getAutoPose(FieldPosManager.autoPath.upperPath, true, 0);
      farChargeAvoidIntermediatePoint = posManager.getAutoPose(FieldPosManager.autoPath.upperPath, true, 1);
    }

    ArrayList<Pose2d> destinations = new ArrayList<>(
      List.of(nearChargeAvoidIntermediatePoint, farChargeAvoidIntermediatePoint,
        new Pose2d(new Translation2d(farChargeAvoidIntermediatePoint.getX(), posManager.getChargePoint(true).getY()),
          nearChargeAvoidIntermediatePoint.getRotation())));

    // <> we only add a single command but that's fine because doing
    // this is much easier than extending swerve auto move command
    addCommands(
      new SwerveAutoMoveCommand(driveSubsystem, destinations, DriveConstants.AutoConstants.kMaxLenientMetersFromGoal,
        DriveConstants.AutoConstants.kMaxRotationFromGoal));
  }
}
