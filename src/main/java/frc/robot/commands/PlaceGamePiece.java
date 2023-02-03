// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
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

  /** Creates a new PlaceGamePiece. */
  public PlaceGamePiece(DriveSubsystem driveSubsystem, LegAnkleSubsystem legAnkleSubsystem, ReidPrototypeSubsystem grabberSubsystem, ProfiledPIDController thetaPidController) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveRobotToGrid(isCube, target, driveSubsystem, thetaPidController),
      new MoveArmToTarget(isCube, target, legAnkleSubsystem),
      new ReleaseGameObject(isCube, target, grabberSubsystem)
    );
  }
}
