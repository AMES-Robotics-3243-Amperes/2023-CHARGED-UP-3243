package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldPosManager;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import java.io.IOException;
import java.util.Optional;

public class PhotonVisionSubsystem extends SubsystemBase {

  // :D these are values that should be in constants after testing
  public static final String camera1Name = Constants.PhotonVision.cameraName1; // :> These are not needed  and can be
  // cleaned up
  
  public static final String camera2Name = Constants.PhotonVision.cameraName2;

  //public static final string cameraName = "Microsoft_LifeCam_HD-3000";
  // :D this is a Transform3d that tracks the transformation from the camera to the robot
  public static final Transform3d camToBot = new Transform3d(
    new Pose3d(Units.inchesToMeters(-14), 0, Units.inchesToMeters(22.5),
      new Rotation3d(0, 0, Units.degreesToRadians(180))), new Pose3d());


  // :D a field layout stores all the vision targets on the field
  static AprilTagFieldLayout m_aprilTagFieldLayout;
  private PhotonCamera camera = new PhotonCamera(Constants.PhotonVision.cameraName1);
  
  private PhotonPoseEstimator backCamPoseEstimator = new PhotonPoseEstimator(m_aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP, camera, camToBot);

  // :> BIG NOTE: Arducam_OV9281_MMN2 is the name of the camera but that is not what it looks for. It is looking for
  // what photonvision reads


  // :D end constants
  FieldPosManager m_field;


  public PhotonVisionSubsystem(FieldPosManager field) {
    m_field = field;
    // :D instantiate the camera and load the field layout
    // :> The reason why we catch this error just to throw a runtime exception is just because for loadfromresource
    // can't handle an IO exception but can
    /** handle a runtimeexception
     */
    try {
      m_aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException err) {
      throw new RuntimeException(err);
    }

    backCamPoseEstimator.setFieldTags(m_aprilTagFieldLayout);

    // backCamPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_LAST_POSE);
  }

  /**
   * Get the estimated camera position based on photon target data.
   * This is an extremely important function that is the main purpose of this subsystem
   *
   * @return {@link Pose3d} representing the position of the camera on the field, or null if no valid targets are found
   */
  public Pose3d checkRobotPosition() {

      // backCamPoseEstimator.setLastPose(m_field.getRobotPose());
      
      Optional<EstimatedRobotPose> backEstPose = backCamPoseEstimator.update();

      if(backEstPose.isPresent()){
        return backEstPose.get().estimatedPose;
      }

    return null;
  }


  @Override
  public void periodic() {

    PhotonPipelineResult result = camera.getLatestResult();

    // This method will be called once per scheduler run
    // :> Checks if targets is empty as a precaution to make sure that it isn't getting values from targets that
    // don't exist
    if (result.hasTargets() && result.getBestTarget().getPoseAmbiguity() < 0.07) {
      Pose3d robotPose = checkRobotPosition();
      // :> As a precuation this makes sure that the field pos mangager isn't getting updated with null data that way
      // we don't get a null pointer exception
      if (robotPose != null) {
        m_field.updateFieldPosWithPhotonVisionPose(robotPose.toPose2d());
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  // :> This method checks to see if it can see an apriltag to update Shuffleboard
  public boolean seeingApriltag() {
    return camera.getLatestResult().hasTargets();
  }
}