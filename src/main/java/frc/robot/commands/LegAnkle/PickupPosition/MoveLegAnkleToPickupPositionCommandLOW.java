// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LegAnkle.PickupPosition;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.JoyUtil;
import frc.robot.commands.LegAnkle.MoveLegAnkleToPositionCommand;
import frc.robot.subsystems.LegAnkleSubsystem;

public class MoveLegAnkleToPickupPositionCommandLOW extends SequentialCommandGroup {

  /** Creates a new MoveLegAnkleToPickupPositionCommand. 
   * 
   * This will move the leg ankle to the low pickup position. It will end when it is in an acceptable margin of the setpoints.
   * First, it will move the wrist to the correct angle, and then move the whole assembly to the corret position.
   * 
   * @param legAnkleSubsystem The {@link LegAnkleSubsystem} that this is controlling.
  */
  public MoveLegAnkleToPickupPositionCommandLOW(LegAnkleSubsystem legAnkleSubsystem) {
    addCommands(
      new MoveLegAnkleToPositionCommand(legAnkleSubsystem, null, null, Constants.AutomationConfiguration.legAnkleDoubleLoadingPosition.pitch, Constants.AutomationConfiguration.legAnkleDoubleLoadingPosition.roll),
      new MoveLegAnkleToPositionCommand(legAnkleSubsystem, Constants.AutomationConfiguration.legAnkleDoubleLoadingPosition)
    );
  }
}
