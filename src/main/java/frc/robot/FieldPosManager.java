// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.PhotonVisionSubsystem;

import java.util.Arrays;
import java.util.List;

/**
 * ++ This class manages the field locations for the robot. It'll deal with outputs from PhotonVision.
 * It'll also deal with odometry stuff.
 */
public class FieldPosManager {

  public DriverStation.Alliance allianceColor = DriverStation.getAlliance();

  public Pose2d latestRobotPosition = new Pose2d();


  public Pose2d latestOdometryPose = new Pose2d();
  public Pose2d previousOdometryPose = new Pose2d();

  public Boolean hasPhotonPose = false;

  // ++ this value is the current target for the robot scoring position
  public int currentIndex;

  /**
   * ++ this array holds the scoring positions based on current alliance
   */
  public Pose2d[] alliedScoringPositions;
  public Pose2d[] opposingScoringPositions;

  public FieldPosManager() {

  }

  /**
   * ++ this sets the scoring positions to the corresponding red or blue alliance points
   * :D I made a list for the opposing scoring positions, because it might be useful
   */
  public void setScoringPositions() {
    if (allianceColor != DriverStation.Alliance.Invalid && allianceColor != null) {
      if (allianceColor == DriverStation.Alliance.Red) {
        alliedScoringPositions = Constants.FieldConstants.Red.scoringPositions;
        opposingScoringPositions = Constants.FieldConstants.Blue.scoringPositions;
      } else if (allianceColor == DriverStation.Alliance.Blue) {
        alliedScoringPositions = Constants.FieldConstants.Blue.scoringPositions;
        opposingScoringPositions = Constants.FieldConstants.Red.scoringPositions;
      }
    } else {
      System.err.println("INVALID ALLIANCE COLOR IN FIELDPOSMANAGER");
    }
  }

  /**
   * :D internal function to increment the robot's previous pose,
   * intended to be used only when updating the robot's pose based on
   * the swerve drive odometry.
   *
   * @param deltaPose is a {@link Transform2d} which transforms the robot's Pose2d
   */
  private void transformRobotPose(Transform2d deltaPose) {
    latestRobotPosition.transformBy(deltaPose);
  }

  /**
   * :D sets the robot pose to default (all zero values)
   * SHOULD ONLY BE USED FOR DEBUGGING
   */
  public void resetRobotPos() {
    latestRobotPosition = new Pose2d();
  }

  /**
   * :D This is a function intended ONLY to be used by the drive subsystem.
   * It finds how much the odometry pose has changed since the last periodic loop.
   * Ideally, replace the swerve's rotation data with imu rotation data
   * (and transform the imu data appropriately so that it lines up)
   *
   * @param swervePose is the Pose2d of the robot as reported by swerve odometry.
   */

  public void updateFieldPosWithSwerveData(Pose2d swervePose) {
    if (hasPhotonPose) {
      previousOdometryPose = latestOdometryPose;
      latestOdometryPose = swervePose;
      Transform2d transform = latestOdometryPose.minus(previousOdometryPose);
      transformRobotPose(transform);
    }
    hasPhotonPose = false;
  }

  /**
   * This function is intended ONLY to be used by PhotonVisionSubsystem.
   * it updates the latest robot pose with photonvision data
   *
   * @param photonPose is the position as reported by the PhotonVisionSubsystem.
   */
  public void updateFieldPosWithPhotonVisionPose(Pose2d photonPose) {
    setRobotPose(PhotonVisionSubsystem.checkRobotPosition().toPose2d());
    hasPhotonPose = true;
  }

  /**
   * Gives the robot's estimated Pose, based on odometry, imu data, and photonvision data.
   *
   * @return estimated robot position as a Pose2d
   */
  public Pose2d getRobotPose() {
    return latestRobotPosition;
  }

  /**
   * :D internal function to overwrite the robot's previous pose,
   * intended to be used only when updating the robot's pose based on
   * the photonvision data.
   *
   * @param pose is the Pose2d to set the robot's latest position to
   */
  private void setRobotPose(Pose2d pose) {
    latestRobotPosition = pose;
  }

  public int getNearestScoringZoneIndex(Pose2d robotPose, boolean ofCurrentAlliance) {
    List<Pose2d> alliedScorePoses = Arrays.asList(alliedScoringPositions);
    List<Pose2d> opposeScorePoses = Arrays.asList(opposingScoringPositions);

    if (ofCurrentAlliance) {
      Pose2d nearestAllyPose = robotPose.nearest(alliedScorePoses);
      for (int i = 0; i < alliedScoringPositions.length; i++) {
        if (alliedScoringPositions[i] == nearestAllyPose) {
          return i;
        }
      }

      return -1;

    } else {
      Pose2d nearestOppPose = robotPose.nearest(opposeScorePoses);
      for (int i = 0; i < alliedScoringPositions.length; i++) {
        if (alliedScoringPositions[i] == nearestOppPose) {
          return i;
        }
      }
      return -1;

    }
  }

  /**
   * H!
   * Assumes ofCurrentAlliance is true
   *
   * @param robotPose The current robot position
   * @return the index for the nearest pose that you can score at
   */
  public int getNearestScoringZoneIndex(Pose2d robotPose) {
    return getNearestScoringZoneIndex(robotPose, true);
  }

  /**
   * H!
   * Assumes robotPose is the current robot position
   *
   * @param ofCurrentAlliance The current robot position
   * @return the index for the nearest pose that you can score at
   */
  public int getNearestScoringZoneIndex(boolean ofCurrentAlliance) {
    return getNearestScoringZoneIndex(getRobotPose(), ofCurrentAlliance);
  }

  /**
   * H!
   * Assumes robotPose is the current robot position, and that ofCurrentAlliance is true
   *
   * @return the index for the nearest pose that you can score at
   */
  public int getNearestScoringZoneIndex() {
    return getNearestScoringZoneIndex(getRobotPose(), true);
  }

  /**
   * A function to find the 2d poses as a top-down view of selected field elements.
   * IDs correspond to the target you're looking for, and can be represented by the following diagram of the field,
   * where the numbers 0-8 on the edges represent the scoring zone IDs:
   *
   * <pre>
   *
   * |                                                     |
   * |                                                     |
   * |                                                     |
   * |0                                                   0|
   * |1                                                   1|
   * |2                                                   2|
   * |3                                                   3|
   * |4                                                   4|
   * |5                                                   5|
   * |6                                                   6|
   * |7                                                   7|
   * |8                                                   8|
   * </pre>
   *
   * @param spot              is a field element enum, of type FieldPosManager.fieldSpot2d, which chooses the
   *                          object whose pose we are looking for.
   * @param ofCurrentAlliance is a boolean which defines whether the field element in question belongs to our
   *                          alliance or the opponent's.
   * @param positionID        is an integer which determines the scoring zone if scoringPosition is passed in to the
   *                          'element' parameter
   * @return the position of the requested field element as a Pose2d.
   */
  public Pose2d get2dFieldObjectPose(fieldSpot2d spot, boolean ofCurrentAlliance, int positionID) {
    if (allianceColor != DriverStation.Alliance.Invalid && allianceColor != null) {
      if ((ofCurrentAlliance && allianceColor == DriverStation.Alliance.Red) || (!ofCurrentAlliance && allianceColor == DriverStation.Alliance.Blue)) {
        // :D red alliance poses
        switch (spot) {
          case doubleLoadingZone:
            return Constants.FieldConstants.Red.doubleLoadingZone;

          case singleLoadingZone:
            return Constants.FieldConstants.Red.singleLoadingZone;

          case chargeStationBottomLeft:
            return Constants.FieldConstants.Red.chargeStationBottomLeft;

          case chargeStationTopRight:
            return Constants.FieldConstants.Red.chargeStationTopRight;

          case scoringPosition:
            // :D TODO: make the robot not throw a fit if scoringZoneID is accidentally not passed in
            return Constants.FieldConstants.Red.scoringPositions[positionID];

          default:
            // :D TODO: figure out what to put here
            return new Pose2d();
        }
      } else {
        // :D blue alliance poses
        switch (spot) {
          case doubleLoadingZone:
            return Constants.FieldConstants.Blue.doubleLoadingZone;

          case singleLoadingZone:
            return Constants.FieldConstants.Blue.singleLoadingZone;

          case chargeStationBottomLeft:
            return Constants.FieldConstants.Blue.chargeStationBottomLeft;

          case chargeStationTopRight:
            return Constants.FieldConstants.Blue.chargeStationTopRight;

          case scoringPosition:
            // :D TODO: make the robot not throw a fit if scoringZoneID is accidentally not passed in
            return Constants.FieldConstants.Blue.scoringPositions[positionID];

          default:
            // :D TODO: figure out what to put here
            return new Pose2d();
        }
      }

    } else {
      System.err.println("INVALID ALLIANCE COLOR IN FIELDPOSMANAGER");
      return null;
    }
  }

  /**
   * A function to find grabber poses as {@link Pose3d}.
   * IDs correspond to the target you're looking for, and can be represented by the following diagram of the field,
   * where the numbers 0-8 on the edges represent the scoring zone IDs and the numbers 0-3 in the center represent
   * the initial game piece positions:
   *
   * <pre>
   *
   * |                                                     |
   * |                                                     |
   * |                                                     |
   * |0                                                   0|
   * |1                     0       0                     1|
   * |2                                                   2|
   * |3                     1       1                     3|
   * |4                                                   4|
   * |5                     2       2                     5|
   * |6                                                   6|
   * |7                     3       3                     7|
   * |8                                                   8|
   * </pre>
   *
   * @param spot              is a grabber spot enum, of type FieldPosManager.fieldSpot3d, which chooses the spot
   *                          whose pose we are looking for.
   * @param ofCurrentAlliance is a boolean which defines whether the field element in question belongs to our
   *                          alliance or the opponent's.
   * @param positionID        is an integer which determines the scoring zone as well as picks 1-4 of the field
   *                          center game pieces
   * @return the position of the requested field element as a {@link Pose3d}.
   */
  public Pose3d get3dFieldObjectPose(fieldSpot3d spot, boolean ofCurrentAlliance, int positionID) {
    if (allianceColor != DriverStation.Alliance.Invalid && allianceColor != null) {
      if ((ofCurrentAlliance && allianceColor == DriverStation.Alliance.Red) || (!ofCurrentAlliance && allianceColor == DriverStation.Alliance.Blue)) {
        // :D red alliance poses
        switch (spot) {
          case highGrabberScoring:
            return new Pose3d(Constants.FieldConstants.Red.grabberPositions.highTargetsX,
              Constants.FieldConstants.targetPositionsY[positionID],
              Constants.FieldConstants.targetPositionsHiZ[positionID], new Rotation3d());

          case middleGrabberScoring:
            return new Pose3d(Constants.FieldConstants.Red.grabberPositions.middleTargetsX,
              Constants.FieldConstants.targetPositionsY[positionID],
              Constants.FieldConstants.targetPositionsMidZ[positionID], new Rotation3d());

          case lowGrabberScoring:
            return new Pose3d(Constants.FieldConstants.Red.grabberPositions.lowTargetsX,
              Constants.FieldConstants.targetPositionsY[positionID], Constants.FieldConstants.targetPositionsLowZ,
              new Rotation3d());

          case centerFieldGamePieces:
            return Constants.FieldConstants.Red.grabberPositions.fieldCenterGamePieces[positionID];

          default:
            // :D TODO: figure out what to put here
            return new Pose3d();
        }
      } else {
        // :D blue alliance poses
        switch (spot) {
          case highGrabberScoring:
            return new Pose3d(Constants.FieldConstants.Blue.grabberPositions.highTargetsX,
              Constants.FieldConstants.targetPositionsY[positionID],
              Constants.FieldConstants.targetPositionsHiZ[positionID], new Rotation3d());

          case middleGrabberScoring:
            return new Pose3d(Constants.FieldConstants.Blue.grabberPositions.middleTargetsX,
              Constants.FieldConstants.targetPositionsY[positionID],
              Constants.FieldConstants.targetPositionsMidZ[positionID], new Rotation3d());

          case lowGrabberScoring:
            return new Pose3d(Constants.FieldConstants.Blue.grabberPositions.lowTargetsX,
              Constants.FieldConstants.targetPositionsY[positionID], Constants.FieldConstants.targetPositionsLowZ,
              new Rotation3d());

          case centerFieldGamePieces:
            return Constants.FieldConstants.Blue.grabberPositions.fieldCenterGamePieces[positionID];

          default:
            // :D TODO: figure out what to put here
            return new Pose3d();
        }
      }
    } else {
      System.err.println("INVALID ALLIANCE COLOR IN FIELDPOSMANAGER");
      return null;
    }
  }

  /**
   * A function which gives positions for autonomous paths between scoring zones and on-field game pieces.
   *
   * @param path              is an enum of type {@link FieldPosManager.autoPath} which is used to say whether the
   *                          upper or lower autonomous path is to be taken.
   * @param ofCurrentAlliance is a boolean which should probably always be true. Detects set alliance and if false,
   *                          will use opposing alliance's positions
   * @param pathStep          is an int from 0 to 1 of the autonomous path step.
   * @return {@link Pose2d} of a position along an autonomous path we are intending to take
   */
  public Pose2d getAutoPose(autoPath path, boolean ofCurrentAlliance, int pathStep) {
    if (allianceColor != DriverStation.Alliance.Invalid && allianceColor != null) {
      if ((ofCurrentAlliance && allianceColor == DriverStation.Alliance.Red) || (!ofCurrentAlliance && allianceColor == DriverStation.Alliance.Blue)) {
        // red
        switch (path) {
          case upperPath:
            return Constants.FieldConstants.Red.autoPositions.upperPath[pathStep];

          case lowerPath:
            return Constants.FieldConstants.Red.autoPositions.lowerPath[pathStep];

          default:
            return new Pose2d();
        }
      } else {
        // blue
        switch (path) {
          case upperPath:

            return Constants.FieldConstants.Blue.autoPositions.upperPath[pathStep];

          case lowerPath:
            return Constants.FieldConstants.Blue.autoPositions.lowerPath[pathStep];

          default:
            return new Pose2d();
        }
      }
    } else {
      System.err.println("INVALID ALLIANCE COLOR IN FIELDPOSMANAGER");
      return null;
    }
  }

  public enum fieldSpot2d {
    doubleLoadingZone, singleLoadingZone, chargeStationBottomLeft, chargeStationTopRight, scoringPosition
  }

  public enum fieldSpot3d {
    highGrabberScoring, middleGrabberScoring, lowGrabberScoring, centerFieldGamePieces
  }

  public enum autoPath {
    upperPath, lowerPath
  }

}

// auto movement points 
//  blue
//   lower path(2.25, 0.9) (6, 0.9)
//   upper path(2.25, 4.6) (6, 4.6)
//  red
//   lower path(14.25, 0.9) (10.5, 0.9)
//   upper path(14.25, 4.6) (10.5, 4.6)
