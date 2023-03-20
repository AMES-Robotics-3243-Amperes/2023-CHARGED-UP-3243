// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveTrain.DriveConstants;
import frc.robot.commands.DriveTrain.BalanceCommand;
import frc.robot.commands.DriveTrain.LockSwerveWheelsCommand;
import frc.robot.commands.DriveTrain.SwerveAutoMoveCommand;
import frc.robot.commands.DriveTrain.SwerveTeleopCommand;
import frc.robot.commands.Grabber.GrabberCloseCommand;
import frc.robot.commands.Grabber.GrabberCommand;
import frc.robot.commands.Grabber.GrabberOpenCommand;
import frc.robot.commands.LegAnkle.ManualLegAnkleCommand;
import frc.robot.commands.LegAnkle.MoveLegAnkleToNeutralPositionCommand;
import frc.robot.commands.LegAnkle.MoveLegAnkleToPickupPositionCommand;
import frc.robot.commands.LegAnkle.MoveLegAnkleToPlacementPositionCommand;
import frc.robot.commands.LegAnkle.WristRollDefaultCommand;
import frc.robot.commands.LegAnkle.WristRollUpCommand;
import frc.robot.subsystems.*;

import java.util.ArrayList;
import java.util.List;
// import edu.wpi.first.cscore.CameraServer;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // ++ CONTROLLER STUFF ---------------------
  public static JoyUtil primaryController = new JoyUtil(Constants.JoyUtilConstants.primaryControllerID);
  public static JoyUtil secondaryController = new JoyUtil(Constants.JoyUtilConstants.secondaryControllerID);

  // <> --- FIELD POS MANAGER ---
  public static FieldPosManager fieldPosManager = new FieldPosManager();

  // The robot's subsystems and commands are defined here...
  // ++ ----- SUBSYSTEMS -----------
  public final PhotonVisionSubsystem m_photonVisionSubsystem = new PhotonVisionSubsystem(fieldPosManager);
  public final LegAnkleSubsystem m_legAnkleSubsystem = new LegAnkleSubsystem();
  public final DriveSubsystem m_driveSubsystem = new DriveSubsystem(fieldPosManager,
    DriveConstants.FieldRelativeTurningConstants.kPidController);

  // ++ ----- COMMANDS -------------
  public final SwerveTeleopCommand m_SwerveTeleopCommand = new SwerveTeleopCommand(m_driveSubsystem, primaryController);
  public final BalanceCommand m_BalanceCommand = new BalanceCommand(m_driveSubsystem);
  // public final MoveLegAnkleToPickupPositionCommand m_legAnkleToPickupCommand = new
  // MoveLegAnkleToPickupPositionCommand(
  //   m_legAnkleSubsystem);// :D duplicate??
  public final MoveLegAnkleToPickupPositionCommand m_moveLegAnkleToPickupPositionCommand =
    new MoveLegAnkleToPickupPositionCommand(
    m_legAnkleSubsystem, secondaryController);
  public final MoveLegAnkleToPlacementPositionCommand m_moveLegAnkleToPlacementPositionCommand =
    new MoveLegAnkleToPlacementPositionCommand(
    m_legAnkleSubsystem, secondaryController);
  public final MoveLegAnkleToNeutralPositionCommand m_moveLegAnkleToNeutralPositionCommand =
    new MoveLegAnkleToNeutralPositionCommand(
    m_legAnkleSubsystem);


  private final GrabberSubsystem m_GrabberSubsystem = new GrabberSubsystem();
  public final GrabberCommand m_GrabberCommand = new GrabberCommand(m_GrabberSubsystem, secondaryController);
  private final ShuffleboardSubsystem m_shuffleboardSubsystem = new ShuffleboardSubsystem(fieldPosManager,
    m_legAnkleSubsystem, m_driveSubsystem, m_photonVisionSubsystem, null, m_GrabberSubsystem);
  // <> this is required for creating new swerve trajectory follow commands
  private final ManualLegAnkleCommand m_manualLegAnkleCommand = new ManualLegAnkleCommand(m_legAnkleSubsystem,
    secondaryController);
  private final GrabberCloseCommand m_grabCloseCommand = new GrabberCloseCommand(m_GrabberSubsystem);
  private final GrabberOpenCommand m_grabOpenCommand = new GrabberOpenCommand(m_GrabberSubsystem);
  private final WristRollDefaultCommand m_wristRollDefaultCommand = new WristRollDefaultCommand(m_legAnkleSubsystem);
  private final WristRollUpCommand m_wristRollUpCommand = new WristRollUpCommand(m_legAnkleSubsystem);

  //private final PlaceGamePiece m_placeGamePieceCommand;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    //m_driveSubsystem.setDefaultCommand(m_SwerveTeleopCommand); // :D TODO: uncomment this

    m_legAnkleSubsystem.setDefaultCommand(m_manualLegAnkleCommand);

    //m_GrabberSubsystem.setDefaultCommand(m_grabCloseCommand);

    // H! This command is here because it needs thetaPidController to be created for it to be created
    //m_placeGamePieceCommand = new PlaceGamePiece(m_driveSubsystem, m_legAnkleSubsystem, GrabberSubsystem,
    //  thetaPidController);


    // ++ driver camera
    CameraServer.startAutomaticCapture();

    // Configure the trigger bindings
    configureBindings();
  }
  //maya stop letting me steal your computer (++ wow, you wrote that in front of me. bold) i know :)

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link JoyUtil}
   */
  public void configureBindings() {
    primaryController.x().toggleOnTrue(new LockSwerveWheelsCommand(m_driveSubsystem));
    primaryController.a().toggleOnTrue(new SwerveAutoMoveCommand(m_driveSubsystem,
      new ArrayList<Pose2d>(List.of(new Pose2d(), new Pose2d(new Translation2d(1, 0), new Rotation2d(45)),
        new Pose2d(new Translation2d(0, 0.5), new Rotation2d(-90)),
        new Pose2d(new Translation2d(1.5, -0.2), new Rotation2d(180)))), DriveConstants.AutoConstants.kMaxMetersFromGoal,
      DriveConstants.AutoConstants.kMaxRotationFromGoal));

    secondaryController.rightBumper().onTrue(m_grabOpenCommand);
    secondaryController.rightBumper().onFalse(m_grabCloseCommand);
    secondaryController.leftBumper().onTrue(m_wristRollUpCommand);
    secondaryController.leftBumper().onFalse(m_wristRollDefaultCommand);
    //secondaryController.x().onTrue(m_legAnkleToPickupCommand);
    // :D whats the deal with this? there are two pickup thingies? I commented the other one out and changed this one
    // to use the x button
    secondaryController.x().onTrue(m_moveLegAnkleToPickupPositionCommand);
    secondaryController.y().onTrue(m_moveLegAnkleToPlacementPositionCommand); // :D DONE: test this at some point soon
    secondaryController.a().onTrue(m_moveLegAnkleToNeutralPositionCommand);
  }

  public void teleopInit() {}

  public Command getAutonomousCommand() {
    return null;
  }

  public void testPeriodic() {
    m_legAnkleSubsystem.testPeriodic();
  }
}
