package frc.robot.commands;

import frc.robot.subsystems.PhotonVisionSubsystem;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** An example command that uses an example subsystem. */
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
    if (m_subsystem.targets.isEmpty() != true){
      System.out.print(botPose);
      botX = m_subsystem.checkRobotPosition().getX();
      botY = m_subsystem.checkRobotPosition().getY();
      botSpin = m_subsystem.checkRobotPosition().getRotation().getAngle();


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