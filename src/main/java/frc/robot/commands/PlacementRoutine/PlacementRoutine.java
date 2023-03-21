// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PlacementRoutine;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.FieldPosManager;
import frc.robot.JoyUtil;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LegAnkleSubsystem;
import frc.robot.utility_classes.GeneralUtil;
import frc.robot.subsystems.GrabberSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlacementRoutine extends SequentialCommandGroup {

  public boolean isCube;
  public FieldPosManager.fieldSpot3d target;
  public DriveSubsystem driveSubsystem;
  public LegAnkleSubsystem legAnkleSubsystem;
  public GrabberSubsystem grabberSubsystem;
  public ProfiledPIDController thetaPidController;
  public JoyUtil controller;
  public FieldPosManager fieldPositionManager;
  public int poseIndex;
  public Pose2d targetPose;

  private EventLoop rightPOVBindingEventLoop;
  private EventLoop leftPOVBindingEventLoop;

  /**
   * Creates a new PlaceGamePiece.
   */
  public PlacementRoutine(FieldPosManager fieldPosManager, DriveSubsystem driveSubsystem,
                        LegAnkleSubsystem legAnkleSubsystem, GrabberSubsystem grabberSubsystem,
                        ProfiledPIDController thetaPidController, JoyUtil controller, int poseIndex) {
    this.fieldPositionManager = fieldPosManager;
    this.driveSubsystem = driveSubsystem;
    this.legAnkleSubsystem = legAnkleSubsystem;
    this.grabberSubsystem = grabberSubsystem;
    this.thetaPidController = thetaPidController;
    this.controller = controller;
    this.poseIndex = poseIndex;

    /* 
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands( // H! TODO Make the target pose based on the index passed in and the FieldPositionManager pose array
      new MoveRobotToGrid(
        fieldPosManager.get2dFieldObjectPose(FieldPosManager.fieldSpot2d.scoringPosition, true, poseIndex),
        driveSubsystem, controller, thetaPidController),
      new MoveArmToTarget(poseIndex, isCube, target, legAnkleSubsystem, fieldPosManager, controller),
      new ReleaseGameObject(isCube, target, grabberSubsystem));


    // H! Binds onPOVRight to starting pov right
    rightPOVBindingEventLoop = new EventLoop();
    controller.povRight().onTrue(new InstantCommand(this::onPOVRight));

    // H! Binds onPOVLeft to starting pov left
    leftPOVBindingEventLoop = new EventLoop();
    controller.povLeft().onTrue(new InstantCommand(this::onPOVLeft));
    */
  }


  public PlacementRoutine(FieldPosManager fieldPosManager, DriveSubsystem driveSubsystem,
                        LegAnkleSubsystem legAnkleSubsystem, GrabberSubsystem grabberSubsystem,
                        ProfiledPIDController thetaPidController, JoyUtil controller) {
    this(fieldPosManager, driveSubsystem, legAnkleSubsystem, grabberSubsystem, thetaPidController, controller,
      fieldPosManager.getNearestScoringZoneIndex() /* H! TODO Needs to be integrated with finding the closest pose */);
  }

  public void onPOVRight() {
    this.cancel();
    new PlacementRoutine(fieldPositionManager, driveSubsystem, legAnkleSubsystem, grabberSubsystem, thetaPidController,
      controller, GeneralUtil.negativeSafeMod(poseIndex + 1, 9)).schedule();
  }

  /*private static int loopIndex(int min, int max, int x) {
    if (x > max) {
      return negativeSafeMod(x - min,  max - min) + max;
    } else if (x < min) {
      return negativeSafeMod(x - min,  max - min) + max;
    }
    return x;
  }*/

  public void onPOVLeft() {
    this.cancel();
    new PlacementRoutine(fieldPositionManager, driveSubsystem, legAnkleSubsystem, grabberSubsystem, thetaPidController,
      controller, GeneralUtil.negativeSafeMod(poseIndex - 1, 9)).schedule();
  }
}
