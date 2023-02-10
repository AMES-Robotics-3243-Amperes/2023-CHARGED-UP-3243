package frc.robot.utility_classes;

import edu.wpi.first.math.geometry.Translation2d;

public class VisionTargetArea {
    public Translation2d position;
    public Translation2d size;
    public double rotation;
    public int id;
    public VisionTargetArea(Translation2d visionTargetPosition, Translation2d floorAreaSize, double rotationAboutX, int photonVisionID) {
        position = visionTargetPosition;
        size = floorAreaSize;
        rotation = rotationAboutX;
        id = photonVisionID;
    }
}