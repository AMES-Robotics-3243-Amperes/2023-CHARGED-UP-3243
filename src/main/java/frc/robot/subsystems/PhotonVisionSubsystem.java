package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.PhotonVisionCommand;

public class PhotonVisionSubsystem extends SubsystemBase {
    
PhotonCamera m_camera;

  private final Field2d m_field2d = new Field2d();

  // :D these are values that should be in constants after testing
  public static final String cameraName = "Global_Shutter_Camera";
  // :> BIG NOTE: Arducam_OV9281_MMN2 is the name of the camera but that is not what it looks for. It is looking for what photonvision reads

  //public static final string cameraName = "Microsoft_LifeCam_HD-3000";
  // :D this is a Transform3d that tracks the transformation from the camera to the robot
  public static final Transform3d camToBot = new Transform3d(
    new Pose3d(

      Units.inchesToMeters(14),
      0,
      Units.inchesToMeters(6),
      new Rotation3d(0,0,0)
    
    ),
    
    new Pose3d()
  );

  // :D end constants

  // :D a field layout stores all the vision targets on the field
  AprilTagFieldLayout m_aprilTagFieldLayout;



  public PhotonVisionSubsystem() {
    

    // :D instantiate the camera and load the field layout
    m_camera = new PhotonCamera(cameraName);
    try{
      m_aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException err){
      throw new RuntimeException();
    }
  }

  /**
   * Get the target data from camera.
   *
   * @return best PhotonTrackedTarget seen by m_camera or null if no targets are found
   */
  public PhotonTrackedTarget returnBestTarget(){
    // :D getting an object (of type var, eugh) which contains the data for all the photon targets visible by the camera
    var results = m_camera.getLatestResult();
    // :D now getting the "best" target, as ruled by photonlib, to pull data from later
    if (results.hasTargets()) {
      PhotonTrackedTarget target = results.getBestTarget();
      return target;
    }
    return null;
  }


  /**
   * Get the estimated camera position based on photon target data.
   *
   * @return Pose3d representing the position of the camera on the field, or null if no valid targets are found
   */
  public Pose3d checkRobotPosition(){
    PhotonTrackedTarget seenTarget = returnBestTarget();
    if (seenTarget != null){
      Transform3d cameraToTarget = seenTarget.getBestCameraToTarget();
      
      // :D Optional allows us to account for the case when the robot sees a fiducial that is not on the field layout file (ids 1-8)
      Optional<Pose3d> tagPose = m_aprilTagFieldLayout.getTagPose(seenTarget.getFiducialId());
    
      if (tagPose.isPresent()){
        Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(cameraToTarget, tagPose.get(), camToBot);
        SmartDashboard.putNumber("robotposePVSX", robotPose.getX());
        return robotPose;
      }
    }
    return null;
  }

 


 /**
   * Get the Pose3d associated with a given target location
   *
   * @param targetID is the selected target to find the Pose3d of, as an integer from 1 to 8
   * @param offset is the offset as a Pose3d where positive offset is from the target towards the center of the field
   * 
   * @return Pose3d representing the position of a scoring position on the field
   */

   public Pose3d getScoringPose(int targetID, Transform3d offset){
    Optional<Pose3d> tagPose = m_aprilTagFieldLayout.getTagPose(targetID);;
    Pose3d scorePose;
    if (targetID>4){
      scorePose = tagPose.get().plus(offset.times(-1));
    } else {
      scorePose = tagPose.get().plus(offset);
    }
    return scorePose;
}

@Override
public void periodic() {
    // :D create a SmartDashboard widget for the field positions
    SmartDashboard.putData("field", m_field2d);

    // This method will be called once per scheduler run
    if (returnBestTarget()!= null){
      m_field2d.setRobotPose(checkRobotPosition().toPose2d());
    }
  }

@Override
public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}