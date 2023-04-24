// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LegAnkle;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LegAnkleSubsystem;
import frc.robot.utility_classes.LegAnklePosition;

public class MoveLegAnkleToPositionCommand extends CommandBase {
  public Double targetPivot;
  public Double targetPitch;
  public Double targetRoll;
  public Double targetExtension;

  public LegAnkleSubsystem legAnkleSubsystem;

  public Timer timeoutTimer = new Timer();
  public double timeoutDuration = Constants.WristAndArm.movementTimeoutDuration;

  
  

  /**Creates a new MoveLegAnkleToPositionCommand. 
   * H!
   * 
   * This will move the leg ankle to the positions entered when scheduled. It will end when it is in an acceptable margin of the setpoints.
   * 
   * @param legAnkleSubsystem The {@link LegAnkleSubsystem} that this is controlling.
   * @param targetExtension The extension value to go to.
   * @param targetPivot The pivot value to go to.
   * @param targetPitch The pitch value to go to.
   * @param targetRoll The roll value to go to.
  */
  public MoveLegAnkleToPositionCommand(LegAnkleSubsystem legAnkleSubsystem, Double targetExtension, Double targetPivot, Double targetPitch, Double targetRoll) {
    setTargets(targetExtension, targetPivot, targetPitch, targetRoll);
    this.legAnkleSubsystem = legAnkleSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(legAnkleSubsystem);
  }

  /**Creates a new MoveLegAnkleToPositionCommand. 
   * H!
   * 
   * This will move the leg ankle to the positions entered when scheduled. It will end when it is in an acceptable margin of the setpoints.
   * 
   * @param legAnkleSubsystem The {@link LegAnkleSubsystem} that this is controlling.
   * @param target A {@link LegAnklePosition} containing the positions to move to.
  */
  public MoveLegAnkleToPositionCommand(LegAnkleSubsystem legAnkleSubsystem, LegAnklePosition target) {
    this(legAnkleSubsystem, target.extension, target.pivot, target.pitch, target.roll);
  }

  /**Creates a new MoveLegAnkleToPositionCommand. 
   * H!
   * 
   * This constructor will not set the target positions, so use it at your own risk!
   * 
   * @param legAnkleSubsystem The {@link LegAnkleSubsystem} that this is controlling.
  */
  protected MoveLegAnkleToPositionCommand(LegAnkleSubsystem legAnkleSubsystem) {
    this.legAnkleSubsystem = legAnkleSubsystem;

    addRequirements(legAnkleSubsystem);
  }

  /**Sets the target positions for each degree of freedom
   * :D
   * 
   * I made this to set the target positions in the {@link MoveLegAnkleToPlacementPositionCommand} command, so that the positions could
   * be set based on stacked controller inputs (such as Y+POV-UP vs Y vs Y+POV-DOWN, all doing very similar things)
   * 
   * @param targetExtension The extension value to go to.
   * @param targetPivot The pivot value to go to.
   * @param targetPitch The pitch value to go to.
   * @param targetRoll The roll value to go to.
   */
  public void setTargets(Double targetExtension, Double targetPivot, Double targetPitch, Double targetRoll){
    /*System.out.println("~~~~~~~~~~~~~~~~~~~~~");
    System.out.println(targetExtension);
    System.out.println(targetPivot);
    System.out.println(targetPitch);
    System.out.println(targetRoll);
    System.out.println("~~~~~~~~~~~~~~~~~~~~~");*/
    this.targetExtension = targetExtension;
    this.targetPivot = targetPivot;
    this.targetPitch = targetPitch;
    this.targetRoll = targetRoll;
  }

  /**Sets the target positions for each degree of freedom
   * :D
   * 
   * I made this to set the target positions in the {@link MoveLegAnkleToPlacementPositionCommand} command, so that the positions could
   * be set based on stacked controller inputs (such as Y+POV-UP vs Y vs Y+POV-DOWN, all doing very similar things)
   * 
   * @param target A {@link LegAnklePosition} containing the positions to move to.
   */
  public void setTargets(LegAnklePosition target){
    setTargets(target.extension, target.pivot, target.pitch, target.roll);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeoutTimer.restart();
    System.out.println("~~~~~~~~~~~~~~~~~~~~~");
    System.out.println(targetExtension);
    System.out.println(targetPivot);
    System.out.println(targetPitch);
    System.out.println(targetRoll);
    System.out.println("~~~~~~~~~~~~~~~~~~~~~");
    legAnkleSubsystem.setMotorPositions(targetExtension, targetPivot, targetPitch, targetRoll);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // H! When the command is done, set the setpoints to the current position so it dosen't keep going.
    LegAnklePosition currentPosition = legAnkleSubsystem.getMotorPosition();
    legAnkleSubsystem.setMotorPositions(currentPosition.extension, currentPosition.pivot, currentPosition.pitch, currentPosition.roll);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return legAnkleSubsystem.isArmPositioned() || timeoutTimer.hasElapsed(timeoutDuration);
    
    //fortnite battle pass
  }
}
