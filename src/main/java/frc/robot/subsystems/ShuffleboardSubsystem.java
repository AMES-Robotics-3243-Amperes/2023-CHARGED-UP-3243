// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LegAnkleSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.IMUSubsystem;

import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.FieldPosManager;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


public class ShuffleboardSubsystem extends SubsystemBase {

  private SendableChooser<Pose2d> m_autoFirstComponent = new SendableChooser<>();

  //m_autoFirstComponent.setDefaultOption("Do Nothing", )

  //&& Widget objects
  static ComplexWidget field2dWidget;

  static SimpleWidget legAnkleCommandWidget;
  static SimpleWidget driveTrainWidget;
  static SimpleWidget chargeStationAngleWidget;

  final Field2d field = new Field2d();
  private FieldPosManager fieldPoseManager;

  private LegAnkleSubsystem legAnkleSubsystem;
  private DriveSubsystem driveTrainSubsystem;
  private PhotonVisionSubsystem photonVisionSubsystem;
  private IMUSubsystem imuSubsystem;
  


  /** Creates a new ShuffleboardSubsystem. */
  public ShuffleboardSubsystem(FieldPosManager posManager, LegAnkleSubsystem legAnkle, DriveSubsystem driveTrain, PhotonVisionSubsystem photonVision, IMUSubsystem IMU) {

    //&& Set fieldPoseManager equal to posManager for Field2D widget
    fieldPoseManager = posManager;
    
    //&& Sets the subsystems used in the widgets equal to their names
    legAnkleSubsystem = legAnkle;
    driveTrainSubsystem = driveTrain;
    photonVisionSubsystem = photonVision;
    imuSubsystem = IMU;

    //SmartDashboard.putNumber(, 0)

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //&& Field 2D widget
    SmartDashboard.putData(field);
    field.setRobotPose(fieldPoseManager.getRobotPose());

    //&& -----------------Titles of the widgets that get displayed in shuffleboard------------------

    //&& Shows the command running for LegAnkleSubsystem
    SmartDashboard.putString("legAnkleCommandWidget", legAnkleSubsystem.getCurrentCommand().getName());

    //&& Shows the command running for DriveTrainSubsystem
    SmartDashboard.putString("driveTrainCommandWidget", driveTrainSubsystem.getCurrentCommand().getName());

    //&& Shows whether any of the drivetrain motors are overheating
    SmartDashboard.putBoolean("motorTooHot", driveTrainSubsystem.getMotorsOkTemperature());

    //&& TODO: Once Jasper merges into dev, finish creating widget for whether grabber is closed or not
    SmartDashboard.putBoolean("grabberClosing", false);

    //&& Shows whether PhotonVision is registering an Apriltag
    SmartDashboard.putBoolean("seeingApriltag", photonVisionSubsystem.seeingApriltag());

    //&& Shows the angle of the charge station as measured by the gyro
    SmartDashboard.putNumber("chargeStationAngleWidget", imuSubsystem.getChargeLevel().getDegrees());

    //&&---------------------------------------------------------------------------------------------

  }
}
