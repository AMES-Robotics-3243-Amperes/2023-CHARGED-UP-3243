// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LegAnkle;

import frc.robot.Constants;
import frc.robot.subsystems.LegAnkleSubsystem;

public class MoveLegAnkleToInitialPositionCommand extends MoveLegAnkleToPositionCommand {
  /** Creates a new MoveLegAnkleToPickupPositionCommand. 
   * 
   * This will move the leg ankle to the inital position, to disengage the pawl. It will end when it is in an acceptable margin of the setpoints.
   * 
   * @param legAnkleSubsystem The {@link LegAnkleSubsystem} that this is controlling.
  */
  public MoveLegAnkleToInitialPositionCommand(LegAnkleSubsystem legAnkleSubsystem) {
    super(legAnkleSubsystem, Constants.AutomationConfiguration.initialLegAnklePositonMovement);
    // Use addRequirements() here to declare subsystem dependencies.
  }
}