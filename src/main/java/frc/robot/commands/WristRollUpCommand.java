package frc.robot.commands;

import frc.robot.subsystems.LegAnkleSubsystem;

public class WristRollUpCommand extends MoveLegAnkleToPositionCommand {

    public WristRollUpCommand(LegAnkleSubsystem legAnkleSubsystem) {
       super(legAnkleSubsystem, null, null, null, 1.0);
    }   
}