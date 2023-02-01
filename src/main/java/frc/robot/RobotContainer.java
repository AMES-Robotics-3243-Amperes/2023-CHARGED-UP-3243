// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.PlaceGamePiece;
import frc.robot.commands.ReidPrototypeCommand;
import frc.robot.subsystems.ReidPrototypeSubsystem;
import frc.robot.subsystems.LegAnkleSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
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
  // ++ ----- COMMANDS -------------
  private final PlaceGamePiece m_placeGamePieceCommand = new PlaceGamePiece(m_driveSubsystem, m_legAnkleSubsystem, m_reidPrototypeSubsystem);
  private final ReidPrototypeCommand m_prototypeCommand = new ReidPrototypeCommand(m_reidPrototypeSubsystem, secondaryController);
  
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_reidPrototypeSubsystem.setDefaultCommand(m_prototypeCommand);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // H! Make it so the X button activates the PlaceGamePiece Routine
    Trigger xButton = new JoystickButton(primaryController, XboxController.Button.kX.value);
    xButton.onTrue(m_placeGamePieceCommand);
  }

}
