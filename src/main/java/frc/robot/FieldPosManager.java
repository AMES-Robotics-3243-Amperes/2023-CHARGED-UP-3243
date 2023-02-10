// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.DriverStation;

/** ++ This class manages the field locations for the robot. It'll deal with outputs from PhotonVisioin. 
 * It'll also deal with odometry stuff.
 */
public class FieldPosManager {

    public DriverStation.Alliance allianceColor = DriverStation.getAlliance();

    public Pose2d latestRobotPosition = new Pose2d();

    public Pose2d latestOdometryPose = new Pose2d();
    public Pose2d previousOdometryPose = new Pose2d();

    public Boolean hasPhotonPose = false;

    // ++ this value is the current target target for the robot scoring position
    public int currentIndex;

    /** ++ this array holds the scoring positions based on current alliance */
    public Pose2d alliedScoringPositions[];
    public Pose2d opposingScoringPositions[];

    /** ++ this sets the scoring positions to the corresponding red or blue alliance points
     *  :D I made a list for the opposiing scoring positions, because it might be useful
    */
    public void setScoringPositions() {
        if (allianceColor != DriverStation.Alliance.Invalid && allianceColor != null ){
            if (allianceColor == DriverStation.Alliance.Red){
                alliedScoringPositions = Constants.FieldConstants.redScoringPositions;
                opposingScoringPositions = Constants.FieldConstants.blueScoringPositions;
            }
            else if (allianceColor == DriverStation.Alliance.Blue){
                alliedScoringPositions = Constants.FieldConstants.blueScoringPositions;
                opposingScoringPositions = Constants.FieldConstants.blueScoringPositions;
            }
        }
    }

    public FieldPosManager(){
        
    }

    /** 
     * :D internal function to overwrite the robot's previous pose,
     * intended to be used only when updating the robot's pose based on
     * the photonvision data.
     * @param pose is the Pose2d to set the robot's latest position to
    */
    private void setRobotPose(Pose2d pose){
        latestRobotPosition = pose;
    }

    /** 
     * :D internal function to increment the robot's previous pose,
     * intended to be used only when updating the robot's pose based on
     * the swerve drive odometry.
     * @param deltaPose is a Transform2d which transforms the robot's Pose2d
    */
    private void transformRobotPose(Transform2d deltaPose){
        latestRobotPosition.transformBy(deltaPose);
    }

    /** 
     * :D This is a function intended ONLY to be used by the swerve module subsystem.
     * It finds how much the odometry pose has changed since the last periodic loop.
     * Ideally, replace the swerve's rotation data with imu rotation data
     * (and transform the imu data appropriately so that it lines up)
     * @param swervePose is the Pose2d of the robot as reported by swerve odometry.
    */
    public void updateFieldPosWithSwerveOdometry(Pose2d swervePose){
        if (hasPhotonPose){
            previousOdometryPose = latestOdometryPose;
            latestOdometryPose = swervePose;
            Transform2d transform = latestOdometryPose.minus(previousOdometryPose);
            transformRobotPose(transform);
        }
        hasPhotonPose = false;
    }

    /** This function is intended ONLY to be used by PhotonVisionSubsystem.
     *  it updates the latest robot pose with photonvision data
     * @param photonPose is the position as reported by the PhotonVisionSubsystem.
    */
    public void updateFieldPosWithPhotonVisionPose(Pose2d photonPose){
        setRobotPose(photonPose);
        hasPhotonPose = true;
    }
    
    /**
     * Gives the robot's estimated Pose, based on odometry, imu data, and photonvision data.
     * @return estimated robot position as a Pose2d
     */
    public Pose2d getRobotPose(){
        return latestRobotPosition;
    }

}
