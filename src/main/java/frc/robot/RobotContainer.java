// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.PhotonVisionCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.JoyUtil; 

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // ++ CONTROLLER STUFF ---------------------
  public static JoyUtil primaryController = new JoyUtil(Constants.Joysticks.primaryControllerID);
  public static JoyUtil secondaryController = new JoyUtil(Constants.Joysticks.secondaryControllerID);
  public final PhotonVisionSubsystem m_photonVisionSubsystem = new PhotonVisionSubsystem();
  public final PhotonVisionCommand m_photonVisionCommand = new PhotonVisionCommand(m_photonVisionSubsystem);

  
  // The robot's subsystems and commands are defined here...
  // ++ ----- SUBSYSTEMS -----------
  public final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  // ++ ----- COMMANDS -------------
  public final TeleopDriveCommand m_TeleopDriveCommand = new TeleopDriveCommand(m_driveSubsystem, primaryController);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    m_photonVisionSubsystem.setDefaultCommand(m_photonVisionCommand);

    m_driveSubsystem.setDefaultCommand(m_TeleopDriveCommand);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

  }


}
