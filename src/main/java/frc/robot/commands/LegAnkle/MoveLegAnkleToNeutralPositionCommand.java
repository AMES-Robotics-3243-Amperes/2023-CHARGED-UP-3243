// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LegAnkle;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.LegAnkleSubsystem;

public class MoveLegAnkleToNeutralPositionCommand extends SequentialCommandGroup {

  /** Creates a new MoveLegAnkleToNeutralPositionCommand. 
   * 
   * 
   * @param legAnkleSubsystem The {@link LegAnkleSubsystem} that this is controlling.
  */
  public MoveLegAnkleToNeutralPositionCommand(LegAnkleSubsystem legAnkleSubsystem) {
    addCommands(
      new MoveLegAnkleToPositionCommand(legAnkleSubsystem, Constants.AutomationConfiguration.initialLegAnklePositonMovement.extension, null, Constants.AutomationConfiguration.initialLegAnklePositonMovement.pitch, null),
      new MoveLegAnkleToPositionCommand(legAnkleSubsystem, Constants.AutomationConfiguration.initialLegAnklePositonMovement)
    );
  }
}