// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;

/** ++ This class manages the field locations for the robot. It'll deal with outputs from PhotonVisioin. 
 * It'll also deal with odometry stuff.
 */
public class FieldPosManager {

    public DriverStation.Alliance allianceColor = DriverStation.getAlliance();

    public Pose2d latestRobotPosition = new Pose2d();

    // ++ this value is the current target target for the robot scoring position
    public int currentIndex;

    /** ++ this array holds the scoring positions based on current alliance */
    public Pose2d scoringPositions[];

    /** ++ this sets the scoring positions to the corresponding red or blue alliance points */
    public void setScoringPositions() {
        if (allianceColor != DriverStation.Alliance.Invalid && allianceColor != null ){
            if (allianceColor == DriverStation.Alliance.Red){
                scoringPositions = Constants.FieldConstants.redScoringPositions;
            }
            else if (allianceColor == DriverStation.Alliance.Blue){
                scoringPositions = Constants.FieldConstants.blueScoringPositions;
            }
        }
    }

    public FieldPosManager(){
        
    }

    public void setRobotPose(Pose2d pose){
        latestRobotPosition = pose;
    }

    public void incrementRobotPose(Transform2d deltaPose){
        latestRobotPosition.transformBy(deltaPose);
    }
    
    public Pose2d getRobotPose(){
        return latestRobotPosition;
    }

}
