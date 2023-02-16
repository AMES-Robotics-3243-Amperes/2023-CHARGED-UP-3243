// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PlaceGamePieceCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ReidPrototypeSubsystem;

public class ReleaseGameObject extends CommandBase {
  private boolean isCube;
  private ReidPrototypeSubsystem grabberSubsystem;
  private Constants.Target target;

  /** Creates a new ReleaseGameObject. */
  public ReleaseGameObject(boolean isCube, Constants.Target target, ReidPrototypeSubsystem grabberSubsystem) {
    this.isCube = isCube;
    this.grabberSubsystem = grabberSubsystem;
    this.target = target;

    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(grabberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // H! TODO Implement finding out whether the grabber is open and make this dependent on it
  }

  protected void onConfirmed() {
    grabberSubsystem.openGrabber(0.1); // H! TODO Put a speed that makes sense in here
  }
}
