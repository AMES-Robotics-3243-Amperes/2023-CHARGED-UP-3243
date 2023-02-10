package frc.robot.commands;

import frc.robot.subsystems.PhotonVisionSubsystem;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class PhotonVisionCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final PhotonVisionSubsystem m_subsystem;

  double targX;
  double targY;
  double targZ;
  int id;
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
    if (m_subsystem.returnBestTarget() != null){
      botPose = m_subsystem.checkRobotPosition();
      System.out.print(botPose);
      SmartDashboard.putNumber("BotPosePVCX", botPose.getX());
      targX = m_subsystem.returnBestTarget().getBestCameraToTarget().getX();
      targY = m_subsystem.returnBestTarget().getBestCameraToTarget().getY();
      targZ = m_subsystem.returnBestTarget().getBestCameraToTarget().getZ();
      id = m_subsystem.returnBestTarget().getFiducialId();
      botX = botPose.getX();
      botY = botPose.getY();
      botSpin = botPose.getRotation().getAngle();


      SmartDashboard.putNumber("target X dist", targX);
      SmartDashboard.putNumber("target Y dist", targY);
      SmartDashboard.putNumber("target Z dist", targZ);
      SmartDashboard.putNumber("robot X", botX);
      SmartDashboard.putNumber("robot Y", botY);
      SmartDashboard.putNumber("robot rotation", botSpin);
      SmartDashboard.putNumber("target dist", Math.sqrt((targX*targX)+(targY*targY)+(targZ*targZ)));
      SmartDashboard.putNumber("target ID", id);
    }
    SmartDashboard.putNumber("Robot Pose X", botPose.getX());
    SmartDashboard.putNumber("Robot Pose Y", botPose.getY());
    SmartDashboard.putNumber("Robot Pose Z", botPose.getZ());
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