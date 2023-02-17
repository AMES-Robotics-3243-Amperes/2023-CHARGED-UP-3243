// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.FieldPosManager;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShuffleboardSubsystem extends SubsystemBase {

  //&&Tabs
  ShuffleboardTab driverFeedback;

  final Field2d field = new Field2d();

  /** Creates a new ShuffleboardSubsystem. */
  public ShuffleboardSubsystem() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //field.setRobotPose(fieldPoseManager);

  }
}
