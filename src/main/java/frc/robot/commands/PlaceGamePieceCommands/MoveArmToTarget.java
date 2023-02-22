// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PlaceGamePieceCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LegAnkleSubsystem;

public class MoveArmToTarget extends CommandBase {
  private boolean isCube;
  private LegAnkleSubsystem legAnkleSubsystem;
  private Constants.Target target;

  private double targetX;
  private double targetY;
  private double targetPitch;
  private double targetRoll;

  private boolean isDone;


  /** Creates a new MoveArmToTarget. */
  public MoveArmToTarget(boolean isCube, Constants.Target target, LegAnkleSubsystem legAnkleSubsystem) {
    this.isCube = isCube;
    this.legAnkleSubsystem = legAnkleSubsystem;
    this.target = target;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(legAnkleSubsystem);


    // H! Get the target positions
    if (isCube) {
      switch (target) {
        case HIGH_TARGET:
          targetX = Constants.AutomationConfigure.Cube.HighTarget.armX;
          targetY = Constants.AutomationConfigure.Cube.HighTarget.armY;
          targetPitch = Constants.AutomationConfigure.Cube.HighTarget.armPitch;
          targetRoll = Constants.AutomationConfigure.Cube.HighTarget.armRoll;
          break;
          
        case MID_TARGET:
          targetX = Constants.AutomationConfigure.Cube.MidTarget.armX;
          targetY = Constants.AutomationConfigure.Cube.MidTarget.armY;
          targetPitch = Constants.AutomationConfigure.Cube.MidTarget.armPitch;
          targetRoll = Constants.AutomationConfigure.Cube.MidTarget.armRoll;
          break;

        case LOW_TARGET:
          targetX = Constants.AutomationConfigure.Cube.LowTarget.armX;
          targetY = Constants.AutomationConfigure.Cube.LowTarget.armY;
          targetPitch = Constants.AutomationConfigure.Cube.LowTarget.armPitch;
          targetRoll = Constants.AutomationConfigure.Cube.LowTarget.armRoll;
          break;
      }
    } else {
      switch (target) {
        case HIGH_TARGET:
          targetX = Constants.AutomationConfigure.Cone.HighTarget.armX;
          targetY = Constants.AutomationConfigure.Cone.HighTarget.armY;
          targetPitch = Constants.AutomationConfigure.Cone.HighTarget.armPitch;
          targetRoll = Constants.AutomationConfigure.Cone.HighTarget.armRoll;
          break;
          
        case MID_TARGET:
          targetX = Constants.AutomationConfigure.Cone.MidTarget.armX;
          targetY = Constants.AutomationConfigure.Cone.MidTarget.armY;
          targetPitch = Constants.AutomationConfigure.Cone.MidTarget.armPitch;
          targetRoll = Constants.AutomationConfigure.Cone.MidTarget.armRoll;
          break;

        case LOW_TARGET:
          targetX = Constants.AutomationConfigure.Cone.LowTarget.armX;
          targetY = Constants.AutomationConfigure.Cone.LowTarget.armY;
          targetPitch = Constants.AutomationConfigure.Cone.LowTarget.armPitch;
          targetRoll = Constants.AutomationConfigure.Cone.LowTarget.armRoll;
          break;
      }
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    isDone = legAnkleSubsystem.moveToXYTheta(targetX, targetY, targetPitch, targetRoll);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}