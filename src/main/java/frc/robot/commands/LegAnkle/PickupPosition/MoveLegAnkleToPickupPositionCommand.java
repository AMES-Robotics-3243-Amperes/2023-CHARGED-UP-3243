// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LegAnkle.PickupPosition;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.JoyUtil;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveLegAnkleToPickupPositionCommand extends InstantCommand {
  public JoyUtil controller;
  public MoveLegAnkleToPickupPositionCommandLOW lowCommand;
  public MoveLegAnkleToPickupPositionCommandNormal normalCommand;
  public MoveLegAnkleToPickupPositionCommandDoubleLoading doubleLoadingCommand;

  public MoveLegAnkleToPickupPositionCommand(MoveLegAnkleToPickupPositionCommandLOW lowCommand, MoveLegAnkleToPickupPositionCommandNormal normalCommand, MoveLegAnkleToPickupPositionCommandDoubleLoading doubleLoadingCommand, JoyUtil controller) {
    this.controller = controller;
    this.lowCommand = lowCommand;
    this.doubleLoadingCommand = doubleLoadingCommand;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (controller.getPOVUp()) {
      doubleLoadingCommand.schedule();
    } else if (controller.getPOVDown()) {
      normalCommand.schedule();
    } else {
      lowCommand.schedule();
    }
  }
}
