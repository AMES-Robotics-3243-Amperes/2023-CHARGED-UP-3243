// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveTrain.DriveConstants;
import frc.robot.JoyUtil;
import frc.robot.subsystems.DriveSubsystem;

public class SwerveTeleopCommand extends CommandBase {

  // <> subsystem
  private final DriveSubsystem m_driveSubsystem;

  // <> driver joyutil
  private final JoyUtil controller;

  // <> driving needs to be reversed if on red alliance
  private boolean reverse;

  /**
   * Creates a new SwerveTeleopCommand.
   */
  public SwerveTeleopCommand(DriveSubsystem subsystem, JoyUtil controller) {
    m_driveSubsystem = subsystem;
    this.controller = controller;

    reverse = false;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveSubsystem.resetFieldRelativeTurningPid();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = -controller.getLeftY() * (reverse ? -1 : 1) * DriveConstants.kDrivingSpeedDamper;
    double ySpeed = -controller.getLeftX() * (reverse ? -1 : 1) * DriveConstants.kDrivingSpeedDamper;

    boolean fieldRelativeDriving = controller.getRightBumper() != DriveConstants.kDrivingFieldRelative;

    double controllerRightY = controller.getRightY();
    double controllerRightX = controller.getRightX();

    if (fieldRelativeDriving && Math.sqrt(controllerRightX * controllerRightX + controllerRightY * controllerRightY) <= 0.8) {
      // <> field relative driving, but no angle specified
      m_driveSubsystem.drive(xSpeed, ySpeed, m_driveSubsystem.getHeading(), true);
    } else if (fieldRelativeDriving) {
      // <> field relative turning, so get an angle
      Rotation2d fieldRelativeTurningGoal = Rotation2d.fromRadians(Math.atan2(-controllerRightX, -controllerRightY));

      m_driveSubsystem.drive(xSpeed, ySpeed, fieldRelativeTurningGoal, true);
    } else {
      // <> not field relative turning, so get raw rotation speed
      double rotationSpeed = controllerRightX * DriveConstants.kAngularSpeedDamper;

      m_driveSubsystem.drive(xSpeed, ySpeed, rotationSpeed, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * <> sets the reverse value (true if red)
   */
  public void setReverse(boolean value) {
    reverse = value;
  }
}
