// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LegAnkle;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.LegAnkleSubsystem;

public class MoveLegAnkleToPlacementPositionLOW extends SequentialCommandGroup {

  /** Creates a new MoveLegAnkleToPlacementPositionHIGH. 
   * 
   * 
   * @param legAnkleSubsystem The {@link LegAnkleSubsystem} that this is controlling.
  */
  public MoveLegAnkleToPlacementPositionLOW(LegAnkleSubsystem legAnkleSubsystem) {
    addCommands(
      new MoveLegAnkleToPositionCommand(legAnkleSubsystem, 
        Constants.AutomationConfiguration.legAnklePlacementPositionLOW.extension,
        null, 
        Constants.AutomationConfiguration.legAnklePlacementPositionLOW.pitch, 
        null
      ),
      new MoveLegAnkleToPositionCommand(legAnkleSubsystem, 
        Constants.AutomationConfiguration.legAnklePlacementPositionLOW.extension,
        Constants.AutomationConfiguration.legAnklePlacementPositionLOW.pivot,
        Constants.AutomationConfiguration.legAnklePlacementPositionLOW.pitch,
        null
      )
    );
  }
}
