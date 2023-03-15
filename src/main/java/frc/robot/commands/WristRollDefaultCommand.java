package frc.robot.commands;

import frc.robot.subsystems.LegAnkleSubsystem;

public class WristRollDefaultCommand extends MoveLegAnkleToPartialPositionCommand {

    public WristRollDefaultCommand(LegAnkleSubsystem legAnkleSubsystem) {
        super(legAnkleSubsystem, null, null, null, 0.0);
    }
}