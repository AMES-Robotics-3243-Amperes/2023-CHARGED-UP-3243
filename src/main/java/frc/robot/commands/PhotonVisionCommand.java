package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PhotonVisionSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class PhotonVisionCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final PhotonVisionSubsystem m_subsystem;

  Pose3d botPose;
  double botX;
  double botY;
  double botSpin;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PhotonVisionCommand(PhotonVisionSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!PhotonVisionSubsystem.targets.isEmpty()) {
      System.out.print(botPose);
      botX = PhotonVisionSubsystem.checkRobotPosition().getX();
      botY = PhotonVisionSubsystem.checkRobotPosition().getY();
      botSpin = PhotonVisionSubsystem.checkRobotPosition().getRotation().getAngle();


      SmartDashboard.putNumber("robot X", botX);
      SmartDashboard.putNumber("robot Y", botY);
      SmartDashboard.putNumber("robot rotation", botSpin);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}