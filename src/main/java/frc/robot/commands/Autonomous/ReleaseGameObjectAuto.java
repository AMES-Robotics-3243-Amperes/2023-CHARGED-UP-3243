// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.Grabber.GrabberOpenCommand;
import frc.robot.subsystems.GrabberSubsystem;

public class ReleaseGameObjectAuto extends GrabberOpenCommand {
  
  protected Timer timeoutTimer = new Timer();
  protected double timeoutDuration = 5;
  protected boolean confirmed = false;

  /** Creates a new ReleaseGameObject. */
  public ReleaseGameObjectAuto(GrabberSubsystem grabberSubsystem) {
    super(grabberSubsystem);
  }

  @Override
  public void initialize() {
    timeoutTimer.restart();
    super.initialize();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timeoutTimer.hasElapsed(timeoutDuration); // H! TODO Implement finding out whether the grabber is open and make this dependent on it
  }

}
