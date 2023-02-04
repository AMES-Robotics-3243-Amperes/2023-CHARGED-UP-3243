// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PlaceGamePieceCommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.Constants;
import frc.robot.JoyUtil;
import frc.robot.commands.SwerveAutoMoveCommand;
import frc.robot.subsystems.DriveSubsystem;

public class MoveRobotToGrid extends SwerveAutoMoveCommand {
  private boolean isCube;
  private DriveSubsystem driveSubsystem;
  private Constants.Target target;
  private JoyUtil controller;
  private EventLoop rightPOVBindingEventLoop;
  private EventLoop leftPOVBindingEventLoop;
  private int targetPoseIndex;
  private boolean isDone;
  private ProfiledPIDController thetaPidController;

  /** Creates a new MoveRobotToGrid. */
  public MoveRobotToGrid(boolean isCube, Constants.Target target, DriveSubsystem driveSubsystem, JoyUtil controller, ProfiledPIDController thetaPidController) {
    super(
      driveSubsystem,
      new Pose2d(), // H! TODO Make this actually be the correct pose
      thetaPidController
    );
    this.isCube = isCube;
    this.driveSubsystem = driveSubsystem;
    this.target = target; 
    this.controller = controller;
    this.thetaPidController = thetaPidController;

    // H! Binds onPOVRight to starting pov right
    rightPOVBindingEventLoop = new EventLoop();
    controller.povRight(rightPOVBindingEventLoop).rising().ifHigh(this::onPOVRight);

    // H! Binds onPOVLeft to starting pov left
    leftPOVBindingEventLoop = new EventLoop();
    controller.povLeft(leftPOVBindingEventLoop).rising().ifHigh(this::onPOVLeft);
  }


  @Override
  public void execute() {
    super.execute();

    // H! WPI Lib says to do this
    rightPOVBindingEventLoop.poll();
    leftPOVBindingEventLoop.poll();
  }


  @Override
  public boolean isFinished() {
    if (isDone) {return true;}
    return super.isFinished();
  }


  public void onPOVRight() {
    changeTrajectory() // H! TODO Configure this once changeTrajectory is implemented in SwerveAutoMoveCommand
  }
  
  public void onPOVLeft() {
  
  }
  
  
}
