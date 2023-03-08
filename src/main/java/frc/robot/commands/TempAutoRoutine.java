// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldPosManager;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;

public class TempAutoRoutine extends SequentialCommandGroup {
  /** Creates a new TempAutoRoutine. */
  public TempAutoRoutine(FieldPosManager posManager, DriveSubsystem subsystem, ProfiledPIDController thetaPidController, PhotonVisionSubsystem photonSub) {
    //addCommands(new InstantCommand(() -> subsystem.drive(-0.15, 0, 0, false), subsystem), new WaitCommand(1), new InstantCommand(() -> subsystem.drive(0, 0, 0, false), subsystem));
    addCommands(new DriveOntoCharge(subsystem), new BalanceCommand(subsystem));
  }
}
