// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldPosManager;

public class ShuffleboardSubsystem extends SubsystemBase {

  //&& Widget objects
  static ComplexWidget field2dWidget;

  //m_autoFirstComponent.setDefaultOption("Do Nothing", )
  static SimpleWidget legAnkleCommandWidget;
  static SimpleWidget driveTrainWidget;
  static SimpleWidget chargeStationAngleWidget;
  private SendableChooser<Pose2d> m_autoFirstComponent = new SendableChooser<>();
  private FieldPosManager fieldPoseManager;

  private LegAnkleSubsystem legAnkleSubsystem;
  private DriveSubsystem driveTrainSubsystem;
  private PhotonVisionSubsystem photonVisionSubsystem;
  private IMUSubsystem imuSubsystem;
  private ReidPrototypeSubsystem reidGrabberSubsystem;


  /**
   * Creates a new ShuffleboardSubsystem.
   */
  public ShuffleboardSubsystem(FieldPosManager posManager, LegAnkleSubsystem legAnkle, DriveSubsystem driveTrain,
                               PhotonVisionSubsystem photonVision, IMUSubsystem IMU, ReidPrototypeSubsystem grabber) {

    //&& Set fieldPoseManager equal to posManager for Field2D widget
    fieldPoseManager = posManager;

    //&& Sets the subsystems used in the widgets equal to their names
    legAnkleSubsystem = legAnkle;
    driveTrainSubsystem = driveTrain;
    photonVisionSubsystem = photonVision;
    imuSubsystem = IMU;
    reidGrabberSubsystem = grabber;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //&& Field 2D widget
    SmartDashboard.putData(fieldPoseManager.getField2d());


    //&& -----------------Titles of the widgets that get displayed in shuffleboard------------------

    SmartDashboard.putBoolean("doCharge", true);
    SmartDashboard.putBoolean("doUpperRoute", true);
    SmartDashboard.putBoolean("doLowerRoute", true);

    SmartDashboard.putNumber("placePiece0", 0);
    SmartDashboard.putNumber("pickupPiece1", 0);
    SmartDashboard.putNumber("placePiece1", 0);
    SmartDashboard.putNumber("pickupPiece2", 0);
    SmartDashboard.putNumber("placePiece2", 0);
    SmartDashboard.putNumber("pickupPiece3", 0);
    SmartDashboard.putNumber("placePiece3", 0);
    SmartDashboard.putNumber("chargePosition", 0);

    //&& Shows the command running for LegAnkleSubsystem
    //SmartDashboard.putString("legAnkleCommandWidget", legAnkleSubsystem.getCurrentCommand().getName());

    //&& Shows the command running for DriveTrainSubsystem
    //SmartDashboard.putString("driveTrainCommandWidget", driveTrainSubsystem.getCurrentCommand().getName());

    //&& Shows whether any of the drivetrain motors are overheating
    SmartDashboard.putBoolean("motorTooHot", driveTrainSubsystem.getMotorsOkTemperature());

    //&& TODO: Once Jasper merges into dev, finish creating widget for whether grabber is closed or not
    SmartDashboard.putBoolean("grabberClosing", reidGrabberSubsystem.getGrabberClosing());

    SmartDashboard.putBoolean("grabberOpening", reidGrabberSubsystem.getGrabberOpening());

    //&& Shows whether PhotonVision is registering an Apriltag
    SmartDashboard.putBoolean("seeingApriltag", photonVisionSubsystem.seeingApriltag());

    //&& Shows the angle of the charge station as measured by the gyro
    //SmartDashboard.putNumber("chargeStationAngleWidget", imuSubsystem.getChargeLevel().getDegrees());

    //&&---------------------------------------------------------------------------------------------

  }

  public boolean ShuffleBoardBooleanInput(ShuffleBoardInput shuffleInput) {
    switch (shuffleInput) {
      case goChargeStation:

        return SmartDashboard.getBoolean("doCharge", true);

      case goUpperRoute:

        return SmartDashboard.getBoolean("doUpperRoute", true);

      case goLowerRoute:

        return SmartDashboard.getBoolean("doLowerRoute", true);

      default:
        return false;
    }
  }

  public double ShuffleBoardNumberInput(ShuffleBoardInput shuffleInput) {
    switch (shuffleInput) {
      case piece0Place:

        return SmartDashboard.getNumber("placePiece0", 0);

      case piece1Pickup:

        return SmartDashboard.getNumber("pickupPiece1", 0);

      case piece1Place:

        return SmartDashboard.getNumber("placePiece1", 0);

      case piece2Pickup:

        return SmartDashboard.getNumber("pickupPiece2", 0);

      case piece2Place:

        return SmartDashboard.getNumber("placePiece2", 0);

      case piece3Pickup:

        return SmartDashboard.getNumber("pickupPiece3", 0);

      case piece3Place:

        return SmartDashboard.getNumber("placePiece3", 0);

      default:

        return -1;

    }

  }

  public enum ShuffleBoardInput {
    piece0Place, piece1Pickup, piece1Place, piece2Pickup, piece2Place, piece3Pickup, piece3Place, goChargeStation,
    chargeStationPosition, goUpperRoute, goLowerRoute
  }
}
