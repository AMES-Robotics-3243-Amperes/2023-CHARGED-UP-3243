// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveTrain.DriveConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
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
  public final DriveSubsystem m_driveSubsystem = new DriveSubsystem(fieldPosManager);
  // ++ ----- COMMANDS -------------
  public final SwerveTeleopCommand m_SwerveTeleopCommand = new SwerveTeleopCommand(m_driveSubsystem, primaryController);
  public final BalanceCommand m_BalanceCommand = new BalanceCommand(m_driveSubsystem);
  public final MoveLegAnkleToPickupPositionCommand m_legAnkleToPickupCommand = new MoveLegAnkleToPickupPositionCommand(
    m_legAnkleSubsystem);
  public final TempAutoRoutine m_auto;
  private final GrabberSubsystem m_GrabberSubsystem = new GrabberSubsystem();
  public final GrabberCommand m_GrabberCommand = new GrabberCommand(m_GrabberSubsystem, secondaryController);
  private final ShuffleboardSubsystem m_shuffleboardSubsystem = new ShuffleboardSubsystem(fieldPosManager,
    m_legAnkleSubsystem, m_driveSubsystem, m_photonVisionSubsystem, null, m_GrabberSubsystem);
  // <> this is required for creating new swerve trajectory follow commands
  private final ProfiledPIDController thetaPidController;
  private final WristCommand m_WristCommand = new WristCommand(m_legAnkleSubsystem, secondaryController);
  private final GrabberCloseCommand m_grabCloseCommand = new GrabberCloseCommand(m_GrabberSubsystem);
  private final GrabberOpenCommand m_grabOpenCommand = new GrabberOpenCommand(m_GrabberSubsystem);

  //private final PlaceGamePiece m_placeGamePieceCommand;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // <> this is required for creating new swerve trajectory follow commands
    thetaPidController = new ProfiledPIDController(DriveConstants.AutoConstants.kTurningP,
      DriveConstants.AutoConstants.kTurningI, DriveConstants.AutoConstants.kTurningD,
      DriveConstants.AutoConstants.kThetaControllerConstraints);
    thetaPidController.enableContinuousInput(-Math.PI, Math.PI);

    m_auto = new TempAutoRoutine(fieldPosManager, m_driveSubsystem, thetaPidController, m_photonVisionSubsystem);

    m_driveSubsystem.setDefaultCommand(m_SwerveTeleopCommand);

    m_legAnkleSubsystem.setDefaultCommand(m_WristCommand);

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
    primaryController.a().toggleOnTrue(
      new SwerveAutoMoveCommand(m_driveSubsystem, new Pose2d(), DriveConstants.AutoConstants.kDrivingPIDController,
        DriveConstants.AutoConstants.kDrivingPIDController, DriveConstants.AutoConstants.kTurningPIDController,
        DriveConstants.AutoConstants.maxMetersFromSetpoint, DriveConstants.AutoConstants.maxRotationFromSetpoint));

    secondaryController.leftBumper().onTrue(m_grabOpenCommand);
    secondaryController.rightBumper().onTrue(m_grabCloseCommand);
    secondaryController.x().onTrue(m_legAnkleToPickupCommand);
  }

  public void teleopInit() {}

  public Command getAutonomousCommand() {
    return m_auto;
  }
}
