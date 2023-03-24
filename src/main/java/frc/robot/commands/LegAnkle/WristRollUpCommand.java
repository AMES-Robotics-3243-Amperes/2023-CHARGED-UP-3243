package frc.robot.commands.LegAnkle;

import frc.robot.subsystems.LegAnkleSubsystem;

public class WristRollUpCommand extends MoveLegAnkleToPositionCommand {

    public WristRollUpCommand(LegAnkleSubsystem legAnkleSubsystem) {
       super(legAnkleSubsystem, null, null, null, 0.25);
    }   
}