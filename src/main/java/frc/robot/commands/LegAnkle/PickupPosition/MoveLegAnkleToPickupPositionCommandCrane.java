// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LegAnkle.PickupPosition;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.JoyUtil;
import frc.robot.commands.LegAnkle.MoveLegAnkleToPositionCommand;
import frc.robot.subsystems.LegAnkleSubsystem;

public class MoveLegAnkleToPickupPositionCommandCrane extends SequentialCommandGroup { // :D TODO: this should move pivot first, then wrist

  // <> public class Dog extends AxiomOfCompleteness {}

  /** Creates a new MoveLegAnkleToPickupPositionCommand. 
   * 
   * This will move the leg ankle to the normal pickup position. It will end when it is in an acceptable margin of the setpoints.
   * 
   * @param legAnkleSubsystem The {@link LegAnkleSubsystem} that this is controlling.
  */
  public MoveLegAnkleToPickupPositionCommandCrane(LegAnkleSubsystem legAnkleSubsystem) {
    addCommands(
      new MoveLegAnkleToPositionCommand(legAnkleSubsystem, 
       Constants.AutomationConfiguration.legAnklePickupPositionSweep.extension, 
        null, 
        null, 
        null
      ),
      new MoveLegAnkleToPositionCommand(legAnkleSubsystem, 
        Constants.AutomationConfiguration.legAnklePickupPositionSweep.extension, 
        Constants.AutomationConfiguration.legAnklePickupPositionSweep.pivot, 
        null, 
        null
      ),
      new MoveLegAnkleToPositionCommand(legAnkleSubsystem, 
        Constants.AutomationConfiguration.legAnklePickupPositionSweep.extension, 
        Constants.AutomationConfiguration.legAnklePickupPositionSweep.pivot, 
        Constants.AutomationConfiguration.legAnklePickupPositionSweep.pitch, 
        null
      )
    );
  }
}
