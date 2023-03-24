package frc.robot.utility_classes;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.DoublePhotonVisionSubsystem;

public class GeneralUtil {

    //&& This utility class holds all the LegAnkle functions that don't need to be in LegAnkleSubsystem to work

    /** ++ clamps input between two extreme values
     * @param min the lower extreme value
     * @param max the upper extreme value
     * @param x the input value to be clamped
     * 
     * @return the value after clamping
     */
    public static double clamp(double min,  double max, double x) {
      return x > max ? max : Math.max(x, min);
    }

    /**Rounds a number to a given number of places
     * H!
     * 
     * @param value The value to round
     * @param places The number of places to round to
     * @return The rounded value
     */
    public static double roundAvoid(double value, int places) {
      double scale = Math.pow(10, places);
      return Math.round(value * scale) / scale;
    }

    public static int negativeSafeMod(int a, int b) {
      if (a >= 0) {
        return a % b;
      }
      return ((a % b) + b) % b;
    }

    /**
     * <> clamps a {@link Rotation2d} between -180 and 180 degrees
     *
     * @return the adjusted {@link Rotation2d}
     */
    public static Rotation2d clampRotation2d(Rotation2d toClamp) {
      // now from -360 to 360
      double moddedDegree = toClamp.getDegrees() % 360;

      // now from 0 to 360
      double doubleModdedDegree = (moddedDegree + 360) % 360;
    
      // now from -180 to 180
      double finalDegree = doubleModdedDegree <= 180 ? doubleModdedDegree : doubleModdedDegree - 360;

      return Rotation2d.fromDegrees(finalDegree);
    }

    // :> Behold the cursed functions Hale made to average Pose3Ds 
    // :> Quiver in its Assemblic WPILIBERAL glory

    public static Rotation3d averageRotation3d(Rotation3d... rotations) {
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
      averageVector = DoublePhotonVisionSubsystem.normalize(averageVector);
      return new Rotation3d(averageVector, averageAngle);
    }

    /**
     * <h2>Finds the average translation between any number of translations</h2>
     * <p>H!</p>
     *
     * @param translations The translations to average between
     * @return The average translation
     */
    public static Translation3d averageTranslation3d(Translation3d... translations) {
      int numArguments = translations.length;
    
      Translation3d averageTranslation = new Translation3d();
    
      // :> Adds together both all translations and divides them to average them together
      for (Translation3d translation : translations) {
        averageTranslation = averageTranslation.plus(translation);
      }
    
      averageTranslation = averageTranslation.div(numArguments);
    
      return averageTranslation;
    }

    public static Pose3d averagePose3d(Pose3d... poses) {
      // :> Makes an array equal to the robot poses it gets for calculations later
      Translation3d[] translations = new Translation3d[poses.length];
      Rotation3d[] rotations = new Rotation3d[poses.length];
    
      for (int i = 0; i < poses.length; i++) {
        // :> Sets the translation and rotation arrays equal to the actual translations and rotation
        translations[i] = poses[i].getTranslation();
        rotations[i] = poses[i].getRotation();
      }
    
      return new Pose3d(averageTranslation3d(translations), averageRotation3d(rotations));
    }
}