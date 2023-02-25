package frc.robot.utility_classes;

import edu.wpi.first.math.geometry.Translation2d;

public class SimulatedFieldPositions {
    public SimulatedFieldPositions() {}

    // :D distances measured in inches from center of field and rotations in degrees from positive x-axis

    public static VisionTargetArea id1 = new VisionTargetArea(new Translation2d(20,54), new Translation2d(2,2), 90, 1);
    public static VisionTargetArea id2 = new VisionTargetArea(new Translation2d(-20,54), new Translation2d(2,2), 90, 2);
    public static VisionTargetArea id3 = new VisionTargetArea(new Translation2d(20,-54), new Translation2d(2,2), -90, 3);
    public static VisionTargetArea id5 = new VisionTargetArea(new Translation2d(-20,-54), new Translation2d(2,2), -90, 5);

    public static VisionTargetArea getTarget(int id){
        switch(id){

        case 1:
            return id1;
        case 2:
            return id2;
        case 3:
            return id3;
        case 5:
            return id5;
        default:
            return id1;
        }
    }
}