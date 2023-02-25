package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// :> I have become eyes, observer of targets
// :> Features stolen code from last year Lemon Juice
public class LimelightSubsystem extends SubsystemBase {


  // :> Network Table stuff
  static NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  // :> Inputting LimeLight entries into the NetworkTables for later
  static NetworkTableEntry tx = limelightTable.getEntry("tx");   // ++ x offset 
  static NetworkTableEntry ty = limelightTable.getEntry("ty");   // ++ y offset
  static NetworkTableEntry ta = limelightTable.getEntry("ta");   // ++ area of target 
  static NetworkTableEntry tv = limelightTable.getEntry("tv");   // 1 if limelight sees a valid target, 0 otherwise

  // :> Not to be confused with targets in PhotonVisionSubsystem
  static Pose2d target = new Pose2d();

  public LimelightSubsystem() {

  }

  public static Pose2d getTarget() {
    return target;
  }

  //  ++ ---------------- target position methods -----------

  /**
   * ++ This method gets the X position of the target the Limelight sees. This is also the rotational error of the robot
   *
   * @return the X position of the target (as an angle)
   */
  public static double getTargetX() {
    return tx.getDouble(0.0);
  }

  /**
   * ++ This method gets the Y position of the target the Limelight sees (including limelight angle offset)
   *
   * @return the Y position of the target (as an angle)
   */
  public static double getTargetY() {
    double angle = ((ty.getDouble(
      (-Constants.Limelight.limelightAngleOffset))) + Constants.Limelight.limelightAngleOffset);
    return angle;
    // ++ I did a weird default value to make the method return 0 if no value is found
  }

  /**
   * ++ This method gets the area of the target the Limelight sees
   *
   * @return the area position of the target (as a fraction of total camera area)
   */
  public static double getTargetArea() {
    return ta.getDouble(0.0);
  }

  /**
   * ++ this method determines if the Limelight sees any valid targets
   *
   * @return true if it sees one or more valid targets, false otherwise
   */
  public static Boolean isTargetValid() {
    // ++ NOTE: "ta" returns "1.0" if it sees ANY number of valid targets
    // ++ for example, it would still return "1.0" if it sees 3 valid targets
    double tvOutput = tv.getDouble(0.0);
    return tvOutput == 1.0;
  }

  private static double findDistanceFromPole1() {
    double distance = (Constants.Limelight.LemontoPole1Height / Math.tan(Math.toRadians(getTargetY())));
    SmartDashboard.putNumber("CALCULATED DISTANCE P1: ", distance);
    return distance;
  }

  private static double findDistanceFromPole2() {
    double distance = (Constants.Limelight.LemontoPole2Height / Math.tan(Math.toRadians(getTargetY())));
    SmartDashboard.putNumber("CALCULATED DISTANCE P1: ", distance);
    return distance;
  }

}
