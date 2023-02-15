package frc.robot.utility_classes;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class RobotTracking {
    public Translation2d robotPosition;
    public Rotation2d rotation;
    public RobotTracking (double x, double y, double th) {
        UpdatePosition(x, y, th);
    }

    public void CalculatePosition(int seenID,double seenSkew,Translation2d seenTarget){
        VisionTargetArea simTarget = SimulatedFieldPositions.getTarget(seenID);
        double simRotation = simTarget.rotation;
        Translation2d simPos = simTarget.position;

        
    }

    public void UpdatePosition(double x, double y, double th){
        robotPosition = new Translation2d(x, y);
        rotation = new Rotation2d(th);
    }
}