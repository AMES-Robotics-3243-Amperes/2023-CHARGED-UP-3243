package frc.robot.commands.LegAnkle;

import frc.robot.subsystems.LegAnkleSubsystem;

public class WristRollDefaultCommand extends MoveLegAnkleToPositionCommand {

    public WristRollDefaultCommand(LegAnkleSubsystem legAnkleSubsystem) {
        super(legAnkleSubsystem, null, null, null, -0.25);
    }
}