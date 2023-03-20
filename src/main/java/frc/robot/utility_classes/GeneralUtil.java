package frc.robot.utility_classes;

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
}
