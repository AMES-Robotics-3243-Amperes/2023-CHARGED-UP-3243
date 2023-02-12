// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.JoyUtil;
import frc.robot.commands.PlaceGamePieceCommands.MoveArmToTarget;
import frc.robot.commands.PlaceGamePieceCommands.MoveRobotToGrid;
import frc.robot.commands.PlaceGamePieceCommands.ReleaseGameObject;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LegAnkleSubsystem;
import frc.robot.subsystems.ReidPrototypeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceGamePiece extends SequentialCommandGroup {

  public boolean isCube;
  public Constants.Target target;
  public DriveSubsystem driveSubsystem;
  public LegAnkleSubsystem legAnkleSubsystem;
  public ReidPrototypeSubsystem grabberSubsystem;
  public ProfiledPIDController thetaPidController;
  public JoyUtil controller;

  private EventLoop rightPOVBindingEventLoop;
  private EventLoop leftPOVBindingEventLoop;

  /** Creates a new PlaceGamePiece. */
  public PlaceGamePiece(DriveSubsystem driveSubsystem, LegAnkleSubsystem legAnkleSubsystem, ReidPrototypeSubsystem grabberSubsystem, ProfiledPIDController thetaPidController, JoyUtil controller, int poseIndex) {
    this.driveSubsystem = driveSubsystem;
    this.legAnkleSubsystem = legAnkleSubsystem;
    this.grabberSubsystem = grabberSubsystem;
    this.thetaPidController = thetaPidController;
    this.controller = controller;
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands( // H! TODO Make the target pose based on the index passed in and the FieldPositionManager pose array
      new MoveRobotToGrid(null, driveSubsystem, controller, thetaPidController),
      new MoveArmToTarget(isCube, target, legAnkleSubsystem),
      new ReleaseGameObject(isCube, target, grabberSubsystem)
    );

    
    // H! Binds onPOVRight to starting pov right
    rightPOVBindingEventLoop = new EventLoop();
    controller.povRight(rightPOVBindingEventLoop).rising().ifHigh(this::onPOVRight);

    // H! Binds onPOVLeft to starting pov left
    leftPOVBindingEventLoop = new EventLoop();
    controller.povLeft(leftPOVBindingEventLoop).rising().ifHigh(this::onPOVLeft);
  }



  public PlaceGamePiece(DriveSubsystem driveSubsystem, LegAnkleSubsystem legAnkleSubsystem, ReidPrototypeSubsystem grabberSubsystem, ProfiledPIDController thetaPidController, JoyUtil controller) {
    this(driveSubsystem, legAnkleSubsystem, grabberSubsystem, thetaPidController, controller, -1 /* H! TODO Needs to be integrated with finding the closest pose */);
  }


  public void onPOVRight() {
    this.cancel();
    new PlaceGamePiece(driveSubsystem, legAnkleSubsystem, grabberSubsystem, thetaPidController, controller).schedule();
  }
  
  public void onPOVLeft() {
    this.cancel();
    new PlaceGamePiece(driveSubsystem, legAnkleSubsystem, grabberSubsystem, thetaPidController, controller).schedule();
  }
}
