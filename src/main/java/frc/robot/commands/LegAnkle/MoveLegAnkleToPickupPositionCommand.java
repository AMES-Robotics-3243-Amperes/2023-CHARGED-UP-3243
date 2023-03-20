// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LegAnkle;

import frc.robot.Constants;
import frc.robot.JoyUtil;
import frc.robot.subsystems.LegAnkleSubsystem;

public class MoveLegAnkleToPickupPositionCommand extends MoveLegAnkleToPositionCommand {

  JoyUtil joy;

  /** Creates a new MoveLegAnkleToPickupPositionCommand. 
   * 
   * This will move the leg ankle to the pickup position. It will end when it is in an acceptable margin of the setpoints.
   * 
   * @param legAnkleSubsystem The {@link LegAnkleSubsystem} that this is controlling.
  */
  public MoveLegAnkleToPickupPositionCommand(LegAnkleSubsystem legAnkleSubsystem, JoyUtil joy) {
    super(legAnkleSubsystem);

    this.joy = joy;
    
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize(){
    // :D I don't think it's physically possible to hold the POV up and down at the same time so this should be ok
    if(joy.getPOVDown()){
      setTargets(Constants.AutomationConfiguration.legAnklePickupPositionLOW);
    } else if(joy.getPOVUp()){
      setTargets(Constants.AutomationConfiguration.legAnkleDoubleLoadingPosition);
    } else {
      setTargets(Constants.AutomationConfiguration.legAnklePickupPosition);
    }
    legAnkleSubsystem.setMotorPositions(targetExtension, targetPivot, targetPitch, targetRoll);
  }
}
