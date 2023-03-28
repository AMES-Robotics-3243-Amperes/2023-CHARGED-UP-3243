// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LegAnkle;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.LegAnkleSubsystem;

public class MoveLegAnkleToPlacementPositionHIGH extends SequentialCommandGroup {

  /** Creates a new MoveLegAnkleToPlacementPositionHIGH. 
   * 
   * 
   * @param legAnkleSubsystem The {@link LegAnkleSubsystem} that this is controlling.
  */
  public MoveLegAnkleToPlacementPositionHIGH(LegAnkleSubsystem legAnkleSubsystem) {
    addCommands(
      new MoveLegAnkleToPositionCommand(legAnkleSubsystem, 
        null, 
        Constants.AutomationConfiguration.legAnklePlacementPositionHIGH.pivot, 
        Constants.AutomationConfiguration.legAnklePlacementPositionHIGH.pitch, 
        null
      ),
      new MoveLegAnkleToPositionCommand(legAnkleSubsystem, 
        Constants.AutomationConfiguration.legAnklePlacementPositionHIGH.extension, 
        Constants.AutomationConfiguration.legAnklePlacementPositionHIGH.pivot, 
        Constants.AutomationConfiguration.legAnklePlacementPositionHIGH.pitch, 
        null
      )
    );
  }
}