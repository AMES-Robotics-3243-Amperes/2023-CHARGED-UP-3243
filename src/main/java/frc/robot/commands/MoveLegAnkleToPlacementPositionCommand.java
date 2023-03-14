// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.JoyUtil;
import frc.robot.subsystems.LegAnkleSubsystem;

public class MoveLegAnkleToPlacementPositionCommand extends MoveLegAnkleToPositionCommand {
  /** Creates a new MoveLegAnkleToPickupPositionCommand. 
   * 
   * This will move the leg ankle to the pickup position. It will end when it is in an acceptable margin of the setpoints.
   * 
   * @param legAnkleSubsystem The {@link LegAnkleSubsystem} that this is controlling.
  */
  public MoveLegAnkleToPlacementPositionCommand(LegAnkleSubsystem legAnkleSubsystem, JoyUtil joy) {
    super(legAnkleSubsystem);
    // :D I don't think it's physically possible to hold the POV up and down at the same time so this should be ok
    if(joy.getPOVDown()){
      setTargets(Constants.AutomationConfiguration.legAnklePlacementPositionLOW);
    } else if(joy.getPOVUp()){
      setTargets(Constants.AutomationConfiguration.legAnklePlacementPositionHIGH);
    } else {
      setTargets(Constants.AutomationConfiguration.legAnklePlacementPositionMIDDLE);
    }
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  

}
