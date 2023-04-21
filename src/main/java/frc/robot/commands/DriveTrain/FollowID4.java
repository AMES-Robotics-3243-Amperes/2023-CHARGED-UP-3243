// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldPosManager;
import frc.robot.FieldPosManager.fieldSpot2d;
import frc.robot.subsystems.DriveSubsystem;

public class FollowID4 extends SwerveAutoMoveCommand {
  /** Creates a new FollowID4. */
  public FollowID4(DriveSubsystem driveSubsystem, FieldPosManager posManager) {
    super(driveSubsystem, new Pose2d(posManager.get2dFieldObjectPose(fieldSpot2d.doubleLoadingZone, DriverStation.getAlliance().equals(Alliance.Red), 0).getTranslation().plus(new Translation2d(-0.2, 0)), Rotation2d.fromDegrees(180)));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
