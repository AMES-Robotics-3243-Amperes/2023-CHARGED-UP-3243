package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldPosManager;

public class PhotonVisionSubsystem extends SubsystemBase {

// :> the = new ArrayList is because the code interprets the array as null otherwise.
static ArrayList<Transform3d> camsToBot = new ArrayList<Transform3d>();
ArrayList<PhotonPipelineResult> results = new ArrayList<PhotonPipelineResult>();
ArrayList<PhotonCamera> cameras = new ArrayList<PhotonCamera>();
public static ArrayList<PhotonTrackedTarget> targets = new ArrayList<PhotonTrackedTarget>();
static ArrayList<Transform3d> cameraToTargets = new ArrayList<Transform3d>();
public static ArrayList<Optional<Pose3d>> tagPoses = new ArrayList<Optional<Pose3d>>();
public static ArrayList<Pose3d> robotPoses = new ArrayList<Pose3d>();


PhotonCamera m_camera;
PhotonCamera m_camera2;
PhotonCamera m_camera3;
FieldPosManager m_field;

  private final Field2d m_field2d = new Field2d();

  // :D these are values that should be in constants after testing
  public static final String cameraName = Constants.PhotonVision.cameraName1;
  public static final String camera2Name = Constants.PhotonVision.cameraName2;
  // :> BIG NOTE: Arducam_OV9281_MMN2 is the name of the camera but that is not what it looks for. It is looking for what photonvision reads

  //public static final string cameraName = "Microsoft_LifeCam_HD-3000";
  // :D this is a Transform3d that tracks the transformation from the camera to the robot
  public static final Transform3d camToBot1 = new Transform3d(
    new Pose3d(

      Units.inchesToMeters(14),
      0,
      Units.inchesToMeters(6),
      new Rotation3d(0,0,0)
    
    ),
    
    new Pose3d()
  );
public static final Transform3d camtoBot2 = new Transform3d(
  new Pose3d(
  Units.inchesToMeters(-2),
  Units.inchesToMeters(-13),
  Units.inchesToMeters(12.5),
  new Rotation3d(0,0, Units.degreesToRadians(-140))

  ),
  new Pose3d()
);


  // :D end constants

  // :D a field layout stores all the vision targets on the field
  static AprilTagFieldLayout m_aprilTagFieldLayout;



  public PhotonVisionSubsystem(FieldPosManager field) {
    m_field = field;
    camsToBot.add(camToBot1);
    camsToBot.add(camtoBot2);
    // :D instantiate the camera and load the field layout
    m_camera = new PhotonCamera(cameraName);
    m_camera2 = new PhotonCamera(camera2Name);
    cameras.add(m_camera);
    cameras.add(m_camera2);
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
  //public PhotonTrackedTarget returnBestTarget(){
    //// :D getting an object (of type var, eugh) which contains the data for all the photon targets visible by the camera
    //var results = m_camera.getLatestResult();
    //// :D now getting the "best" target, as ruled by photonlib, to pull data from later
    //if (results.hasTargets()) {
      //PhotonTrackedTarget target = results.getBestTarget();
      //return target;
    //}
    //return null;
  //}


  /**
   * Get the estimated camera position based on photon target data.
   *
   * @return {@link Pose3d} representing the position of the camera on the field, or null if no valid targets are found
   */
  public static Pose3d checkRobotPosition(){
    // :> I'm so sorry for all of the for loops it is necessary for the three cameras.
    if (!targets.isEmpty()){
      for (int i = 0; i < 1; i++) {
      cameraToTargets.add(targets.get(i).getBestCameraToTarget());
      }
      // :D Optional allows us to account for the case when the robot sees a fiducial that is not on the field layout file (ids 1-8)
      for (int i = 0; i < 1; i++) {
      tagPoses.add(m_aprilTagFieldLayout.getTagPose(targets.get(i).getFiducialId()));
      }
      for (int j = 0; j < 1; j++) {
      if (tagPoses.get(j).isPresent()){
        robotPoses.add(PhotonUtils.estimateFieldToRobotAprilTag(cameraToTargets.get(j), tagPoses.get(j).get(), camsToBot.get(j)));
        // :> The .get(j)s correspond to the for loop but the other one turns it into a Pose3D instead of an optional Pose3D
        
        
      }
    }
    Pose3d averageRobotPoses = averagePose3d(robotPoses.toArray(new Pose3d[robotPoses.size()]));
    return averageRobotPoses;
    }
    return null;
  }
  // :> Behold the cursed functions Hale made to average Pose3Ds 
  // :> Quiver in its Assemblic WPILIBERAL glory

  private static Pose3d averagePose3d(Pose3d... poses) {
    Translation3d[] translations = new Translation3d[poses.length];
    Rotation3d[] rotations = new Rotation3d[poses.length];

    for (int i = 0; i < poses.length; i++) {
      translations[i] = poses[i].getTranslation();
      rotations[i] = poses[i].getRotation();
    }

    return new Pose3d(averageTranslation3d(translations), averageRotation3d(rotations));
  }


  /**<h2>Finds the average translation between any number of translations</h2>
   * <p>H!</p>
   * @param translations The translations to average between
   * @return The average translation
   */
  private static Translation3d averageTranslation3d(Translation3d... translations) {
    int numArguments = translations.length;

    Translation3d averageTranslation = new Translation3d();

    for (Translation3d translation : translations) {
      averageTranslation = averageTranslation.plus( translation.div(numArguments) );
    }

    return averageTranslation;
  }


  private static Rotation3d averageRotation3d(Rotation3d... rotations) {
    int numArguments = rotations.length;

    // :> Takes the rotation into it's vectors
    Vector<N3> averageVector = VecBuilder.fill(0, 0, 0);
    double averageAngle = 0;

    // :> Averages the values of each vector and it's angles
    for (Rotation3d rotation : rotations) {
      averageVector = new Vector<N3>(averageVector.plus(rotation.getAxis().div(numArguments)));
      averageAngle += rotation.getAngle() / numArguments;
    }

    // :> normalizes the Vector and angle and puts it back together into a Rotation3D
    averageVector = normalize(averageVector);
    return new Rotation3d(averageVector, averageAngle);
  }

  // :> Manually normalizes the vector since WPILIB doesn't have a function for it already.
  private static Vector<N3> normalize(Vector<N3> vector) {
    return vector.div(Math.sqrt(vector.elementPower(2).elementSum()));
  }





 /**
   * Get the {@link Pose3d} associated with a given target location
   *
   * @param targetID is the selected target to find the Pose3d of, as an integer from 1 to 8
   * @param offset is the offset as a Pose3d where positive offset is from the target towards the center of the field
   * 
   * @return Pose3d representing the position of a scoring position on the field
   */

   public Pose3d getScoringPose(int targetID, Transform3d offset){
    Optional<Pose3d> tagPose = m_aprilTagFieldLayout.getTagPose(targetID);
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

  for (int i = 0; i < 1; i++) {
    PhotonPipelineResult r=cameras.get(i).getLatestResult();
    if (r.hasTargets()) {
      results.add(r);
      targets.add(results.get(i).getBestTarget());
    }
  }

    // :D create a SmartDashboard widget for the field positions
    SmartDashboard.putData("field", m_field2d);

    // This method will be called once per scheduler run
    if (targets.isEmpty() != true){
      m_field.updateFieldPosWithPhotonVisionPose(checkRobotPosition().toPose2d());
    }
    targets.clear();
  }

@Override
public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  //&& TODO: define function that returns true if any of the cameras are seeing an Apriltag
  public boolean seeingApriltag() {
    return !targets.isEmpty();
  }
}