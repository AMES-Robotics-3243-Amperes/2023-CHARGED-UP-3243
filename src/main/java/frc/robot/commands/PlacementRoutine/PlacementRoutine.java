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
import frc.robot.commands.SnapToGridRoutine.SnapToGridCommand;
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
  public JoyUtil primaryController;
  public JoyUtil secondaryController;
  public FieldPosManager fieldPositionManager;
  public int poseIndex;
  public Pose2d targetPose;

  /**
   * Creates a new PlaceGamePiece.
   */
  public PlacementRoutine(FieldPosManager fieldPosManager, DriveSubsystem driveSubsystem,
                        LegAnkleSubsystem legAnkleSubsystem, GrabberSubsystem grabberSubsystem,
                        ProfiledPIDController thetaPidController, JoyUtil secondaryController, JoyUtil primaryController) {
    this.fieldPositionManager = fieldPosManager;
    this.driveSubsystem = driveSubsystem;
    this.legAnkleSubsystem = legAnkleSubsystem;
    this.grabberSubsystem = grabberSubsystem;
    this.thetaPidController = thetaPidController;
    this.primaryController = primaryController;
    this.secondaryController = secondaryController;

    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SnapToGridCommand(driveSubsystem, fieldPosManager, primaryController, secondaryController),
      new MoveArmToTarget(poseIndex, isCube, target, legAnkleSubsystem, fieldPosManager, secondaryController), //H! this needs to be changed so it no longer needs posIndex, or that needs to be gotten from snap to grid once it ends
      new ReleaseGameObject(grabberSubsystem, secondaryController)
    );


    
  }


  /*private static int loopIndex(int min, int max, int x) {
    if (x > max) {
      return negativeSafeMod(x - min,  max - min) + max;
    } else if (x < min) {
      return negativeSafeMod(x - min,  max - min) + max;
    }
    return x;
  }*/
}
