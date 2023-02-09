// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveTrain.DriveConstants;
import frc.robot.commands.PlaceGamePiece;
import frc.robot.commands.ReidPrototypeCommand;
import frc.robot.commands.SwerveAutoMoveCommand;
import frc.robot.commands.SwerveTeleopCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LegAnkleSubsystem;
import frc.robot.subsystems.ReidPrototypeSubsystem;

import java.util.List;

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
  public static JoyUtil primaryController = new JoyUtil(Constants.Joysticks.primaryControllerID);
  public static JoyUtil secondaryController = new JoyUtil(Constants.Joysticks.secondaryControllerID);

  // The robot's subsystems and commands are defined here...
  // ++ ----- SUBSYSTEMS -----------
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final LegAnkleSubsystem m_legAnkleSubsystem = new LegAnkleSubsystem();
  private final ReidPrototypeSubsystem m_reidPrototypeSubsystem = new ReidPrototypeSubsystem();

  // <> this is required for creating new swerve trajectory follow commands
  private final ProfiledPIDController thetaPidController;

  // ++ ----- COMMANDS -------------
  //private final SwerveTrajectoryFollowCommand m_SwerveTrajectoryFollowCommand;
  private final SwerveTeleopCommand m_SwerveTeleopCommand = new SwerveTeleopCommand(m_driveSubsystem,
    primaryController);

  //private final PlaceGamePiece m_placeGamePieceCommand;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    thetaPidController = new ProfiledPIDController(DriveConstants.AutoConstants.kTurningP,
      DriveConstants.AutoConstants.kTurningI, DriveConstants.AutoConstants.kTurningD,
      DriveConstants.AutoConstants.kThetaControllerConstraints);
    thetaPidController.enableContinuousInput(-Math.PI, Math.PI);

    m_driveSubsystem.setDefaultCommand(m_SwerveTeleopCommand);
    m_driveSubsystem.resetOdometry(new Pose2d(new Translation2d(), m_driveSubsystem.getHeading()));

    // H! This command is here because it needs thetaPidController to be created for it to be created
    //m_placeGamePieceCommand = new PlaceGamePiece(m_driveSubsystem, m_legAnkleSubsystem, m_reidPrototypeSubsystem,
    //  thetaPidController);

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
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  public void configureBindings() {
    JoystickButton primaryAButton = new JoystickButton(primaryController, Constants.Joysticks.A);
    primaryAButton.onTrue(new SwerveAutoMoveCommand(m_driveSubsystem,
      TrajectoryGenerator.generateTrajectory(new Pose2d(), List.of(), new Pose2d(new Translation2d(-1.2, 0.3),
        Rotation2d.fromDegrees(90)), DriveConstants.AutoConstants.trajectoryConfig), thetaPidController, false));

    // H! Make it so the X button activates the PlaceGamePiece Routine
    Trigger xButton = new JoystickButton(primaryController, XboxController.Button.kX.value);
    //xButton.onTrue(m_placeGamePieceCommand);
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
