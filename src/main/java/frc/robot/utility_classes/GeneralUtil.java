package frc.robot.utility_classes;

import edu.wpi.first.math.geometry.Rotation2d;

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
}
