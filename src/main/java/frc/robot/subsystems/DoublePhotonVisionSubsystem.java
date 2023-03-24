package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldPosManager;
import frc.robot.utility_classes.GeneralUtil;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

public class DoublePhotonVisionSubsystem extends SubsystemBase {

  // :D these are values that should be in constants after testing
  public static final String camera1Name = Constants.PhotonVision.cameraName1; // :> These are not needed  and can be
  // cleaned up
  
  public static final String camera2Name = Constants.PhotonVision.cameraName2;

  //public static final string cameraName = "Microsoft_LifeCam_HD-3000";
  // :D this is a Transform3d that tracks the transformation from the camera to the robot
  public static final Transform3d camToBot1 = new Transform3d(
    new Pose3d(Units.inchesToMeters(-14), 0, Units.inchesToMeters(22.5),
      new Rotation3d(0, 0, Units.degreesToRadians(180))), new Pose3d());

  public static final Transform3d camToBot2 = new Transform3d(
    new Pose3d(Units.inchesToMeters(2), Units.inchesToMeters(0), Units.inchesToMeters(14.5),
      new Rotation3d(0, 0, Units.degreesToRadians(0))), new Pose3d());

  public static ArrayList<PhotonTrackedTarget> targets = new ArrayList<PhotonTrackedTarget>();
  public static ArrayList<Optional<Pose3d>> tagPoses = new ArrayList<Optional<Pose3d>>();

  // :> the = new ArrayList is because the code interprets the array as null otherwise.
  static List<Transform3d> camsToBot = Arrays.asList(camToBot1, camToBot2);
  static ArrayList<Transform3d> cameraToTargets = new ArrayList<Transform3d>();

  // :D a field layout stores all the vision targets on the field
  static AprilTagFieldLayout m_aprilTagFieldLayout;
  ArrayList<PhotonPipelineResult> results = new ArrayList<PhotonPipelineResult>();
  List<PhotonCamera> cameras = Arrays.asList(new PhotonCamera(Constants.PhotonVision.cameraName1),
    new PhotonCamera(Constants.PhotonVision.cameraName2));
  
  private PhotonPoseEstimator frontCamPoseEstimator = new PhotonPoseEstimator(m_aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP, cameras.get(1), camToBot2);
  private PhotonPoseEstimator backCamPoseEstimator = new PhotonPoseEstimator(m_aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP, cameras.get(0), camToBot1);

  // :> BIG NOTE: Arducam_OV9281_MMN2 is the name of the camera but that is not what it looks for. It is looking for
  // what photonvision reads


  // :D end constants
  FieldPosManager m_field;


  public DoublePhotonVisionSubsystem(FieldPosManager field) {
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

    frontCamPoseEstimator.setFieldTags(m_aprilTagFieldLayout);
    backCamPoseEstimator.setFieldTags(m_aprilTagFieldLayout);

    frontCamPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_LAST_POSE);
    backCamPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_LAST_POSE);
  }

  /**
   * Get the estimated camera position based on photon target data.
   * This is an extremely important function that is the main purpose of this subsystem
   *
   * @return {@link Pose3d} representing the position of the camera on the field, or null if no valid targets are found
   */
  public Pose3d checkRobotPosition() {
    // :> I'm so sorry for all of the for loops it is necessary for the two cameras.
    //if (!targets.isEmpty()) {
      //for (int i = 0; i < targets.size(); i++) {
      //cameraToTargets.add(targets.get(i).getBestCameraToTarget());
      //}

      // for (int i = 0; i < targets.size(); i++) {
      // tagPoses.add(m_aprilTagFieldLayout.getTagPose(targets.get(i).getFiducialId()));
      // }

      // :> This next set of code in this function is the main logic to how we are getting our postional data.

      // :> Defines the robot poses array to be used in the function
      ArrayList<Pose3d> robotPoses = new ArrayList<Pose3d>();

      frontCamPoseEstimator.setLastPose(m_field.getRobotPose());
      backCamPoseEstimator.setLastPose(m_field.getRobotPose());
      
      Optional<EstimatedRobotPose> frontEstPose = frontCamPoseEstimator.update();
      Optional<EstimatedRobotPose> backEstPose = backCamPoseEstimator.update();

      if(frontEstPose.isPresent()){
        robotPoses.add(frontEstPose.get().estimatedPose);
      }
      if(backEstPose.isPresent()){
        robotPoses.add(backEstPose.get().estimatedPose);
      }

      // :> Makes a for loop based on how many cameras are picking up targets
      // for (int i = 0; i < targets.size(); i++) {
      //   // :> Safety precuation that makes sure the that inside the cameras targets it isn't null that way it doesn't
      //   // return a null pointer error
      //   if (targets.get(i) != null) {
      //     // :> Gets the distance from the cameras to the targets it see's based off of which camera it's getting
      //     // data from
      //     Transform3d cameraToTarget = targets.get(i).getBestCameraToTarget();
      //     //:> Gets the position of the apriltags on the field that it see's from the cameras.
      //     Optional<Pose3d> tagPose = m_aprilTagFieldLayout.getTagPose(targets.get(i).getFiducialId());

      //     if (!tagPose.isPresent()) {
      //       continue;
      //     }

      //     // :> Uses the official Photonvision function to take in all of the previous data and get a field position
      //     // from it.
      //     /* It does this for both cameras that way it can get the most accurate position possible. The reason why
      //       this system is used is because you can't estimate any data except for Pose3Ds which is essential for
      //       getting good positonal data
      //      */
          
      //     robotPoses.add(PhotonUtils.estimateFieldToRobotAprilTag(cameraToTarget, tagPose.get(), camsToBot.get(i)));
      //     // if (tagPoses.get(i).isPresent()) {
      //     //   for (int k = 0; k < cameraToTargets.size(); k++) {
      //     //     robotPoses.add(
      //     //       PhotonUtils.estimateFieldToRobotAprilTag(cameraToTargets.get(k), tagPoses.get(j).get(), camsToBot
      //     //       .get(k)));
      //     //   // :> The .get(j)s correspond to the for loop but the other one turns it into a Pose3D instead of an
      //     //   optional Pose3D
      //     //   }
      //     // }
      //   }
      // }


      // :> An if statement is used here to save robot resources and time so instead of estimating Pose 3Ds when it
      // reads only from one camera it can just report that position
      // :> The other part of this if statement checks to make sure that no matter what if there is a null in the
      // array or the array is of 0 length
      /*  it doesn't crash the robot and just returns null for the function as it should be doing
       */
      if (robotPoses.size() == 0) {
        return null;
      } else if (robotPoses.size() == 1) {
        return robotPoses.get(0);
      }

      // :> When it see's targets from two targets the funciton below will average two poses together to get the most
      // accurate field postition.
      Pose3d averageRobotPoses = GeneralUtil.averagePose3d(robotPoses.toArray(new Pose3d[0]));
      return averageRobotPoses;
      // :> It's okay if averagePose3D = null since CRP can be null anyway and it gets passed in to averagerobotposes
    //}

    //return null;
  }

  // :> Manually normalizes the vector since WPILIB doesn't have a function for it already.
  public static Vector<N3> normalize(Vector<N3> vector) {
    return vector.div(Math.sqrt(vector.elementPower(2).elementSum()));
  }

  // :> Returns the best and all targets it can see and if it doesn't see any it returns null
  /* It also needs a camera to do this and takes all the data from the camera that it can find
   */ 
  private PhotonTrackedTarget getBestTarget(PhotonCamera cam) {
    PhotonPipelineResult result = cam.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    if (target != null && target.getPoseAmbiguity() > 0.1 ){
        return null;
    }
    return target;
  }

  /**
   * Get the {@link Pose3d} associated with a given target location
   *
   * @param targetID is the selected target to find the Pose3d of, as an integer from 1 to 8
   * @param offset   is the offset as a Pose3d where positive offset is from the target towards the center of the field
   * @return Pose3d representing the position of a scoring position on the field
   */

  // public Pose3d getScoringPose(int targetID, Transform3d offset) {
  //   Optional<Pose3d> tagPose = m_aprilTagFieldLayout.getTagPose(targetID);
  //   Pose3d scorePose;
  //   if (targetID > 4) {
  //     scorePose = tagPose.get().plus(offset.times(-1));
  //   } else {
  //     scorePose = tagPose.get().plus(offset);
  //   }
  //   return scorePose;
  // }
  @Override
  public void periodic() {
    // :> Periodically adds target values to the targets array equal to what the cameras can see
    for (int i = 0; i < cameras.size(); i++) {
      targets.add(i, getBestTarget(cameras.get(i)));
      PhotonPipelineResult r = cameras.get(i).getLatestResult();
        if (r.hasTargets()) {
          results.add(r);
          targets.add(r.getBestTarget());
        }
    }

    // This method will be called once per scheduler run
    // :> Checks if targets is empty as a precaution to make sure that it isn't getting values from targets that
    // don't exist
    if (!targets.isEmpty()) {
      Pose3d robotPose = checkRobotPosition();
      // :> As a precuation this makes sure that the field pos mangager isn't getting updated with null data that way
      // we don't get a null pointer exception
      if (robotPose != null) {
        m_field.updateFieldPosWithPhotonVisionPose(robotPose.toPose2d());
      }
    }
    // :> Clears the targets array that way it doesn't overflow with old targets or cause an out of bounds error
    targets.clear();
    //results.clear();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  // :> This method checks to see if it can see an apriltag to update Shuffleboard
  public boolean seeingApriltag() {
    return !targets.isEmpty();
  }
}