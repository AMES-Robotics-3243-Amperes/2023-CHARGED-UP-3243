// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LegAnkle.PickupPosition;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.JoyUtil;
import frc.robot.commands.LegAnkle.MoveLegAnkleToPositionCommand;
import frc.robot.subsystems.LegAnkleSubsystem;

public class MoveLegAnkleToPickupPositionCommandDoubleLoading extends SequentialCommandGroup {

  /** Creates a new MoveLegAnkleToPickupPositionCommand. 
   * 
   * This will move the leg ankle to the double loading station pickup position. It will end when it is in an acceptable margin of the setpoints.
   * 
   * @param legAnkleSubsystem The {@link LegAnkleSubsystem} that this is controlling.
  */
  public MoveLegAnkleToPickupPositionCommandDoubleLoading(LegAnkleSubsystem legAnkleSubsystem) {
    addCommands(
      new MoveLegAnkleToPositionCommand(legAnkleSubsystem, 
        Constants.AutomationConfiguration.legAnkleDoubleLoadingPosition.extension, 
        null, 
        null, 
        null
      ),
      new MoveLegAnkleToPositionCommand(legAnkleSubsystem, 
        Constants.AutomationConfiguration.legAnkleDoubleLoadingPosition.extension, 
        null, 
        Constants.AutomationConfiguration.legAnkleDoubleLoadingPosition.pitch, 
        null
      ),
      new MoveLegAnkleToPositionCommand(legAnkleSubsystem, 
        Constants.AutomationConfiguration.legAnkleDoubleLoadingPosition.extension, 
        Constants.AutomationConfiguration.legAnkleDoubleLoadingPosition.pivot, 
        Constants.AutomationConfiguration.legAnkleDoubleLoadingPosition.pitch, 
        null
      )
    );
  }
}
