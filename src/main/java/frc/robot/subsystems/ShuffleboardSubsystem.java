// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.FieldPosManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShuffleboardSubsystem extends SubsystemBase {

  //&& Widget objects
  static ComplexWidget field2dWidget;

  static SimpleWidget legAnkleCommandWidget;
  static SimpleWidget driveTrainWidget;

  final Field2d field = new Field2d();
  private FieldPosManager fieldPoseManager;

  private LegAnkleSubsystem legAnkleSubsystem;
  private DriveSubsystem driveTrainSubsystem;
  private boolean[] photonVisionSubsystem;


  /** Creates a new ShuffleboardSubsystem. */
  public ShuffleboardSubsystem(FieldPosManager posManager, LegAnkleSubsystem legAnkle, DriveSubsystem driveTrain, boolean[] photonVision) {

    //&& set fieldPoseManager equal to posManager for Field2D widget
    fieldPoseManager = posManager;
    
    //&& Sets subsystems used in the widgets equal to their names
    legAnkleSubsystem = legAnkle;
    driveTrainSubsystem = driveTrain;
    photonVisionSubsystem = photonVision;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //&& Field 2D widget
    SmartDashboard.putData(field);
    field.setRobotPose(fieldPoseManager.getRobotPose());

    //&& Names the widgets that get displayed in shuffleboard
    SmartDashboard.putString("legAnkleCommandWidget", legAnkleSubsystem.getClass().getName());
    SmartDashboard.putString("driveTrainCommandWidget", driveTrainSubsystem.getClass().getName());

    SmartDashboard.putBoolean("motorOkTemp", driveTrainSubsystem.getMotorsOkTemperature());

    //&& TODO: Once Jasper merges into dev, finish creating widget for whether grabber is closed or not
    SmartDashboard.putBoolean("grabberClosing", false);

    // SmartDashboard.putBooleanArray("seeingApriltag", photonVisionSubsystem);



  }
}
