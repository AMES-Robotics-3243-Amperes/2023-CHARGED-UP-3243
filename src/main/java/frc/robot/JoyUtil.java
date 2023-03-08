package frc.robot;

import frc.robot.Constants.JoyUtilConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * <> {@link CommandXboxController} with many joystick tweaks in addition
 * to various other helper functions. All around a swell time :)
 *
 * <p> Raw joystick output goes through deadzoning, curving, trigger
 * speed multiplier adjusting, then rate limiting. This means that
 * the rate limit is absolute and will always be obeyed no matter what. </p>
 *
 * <p> Curving is the following for input x, coefficients a and b,
 * and exponents n and k: output = a(x^n) + b(x^k) </p>
 */
public class JoyUtil extends CommandXboxController {
  private static final double sqrt2Over2 = Math.sqrt(2) / 2;

  private final double deadzone;
  private final double exponent1, exponent2, coefficient1, coefficient2;
  private final double leftTriggerLeftStickMultiplier, rightTriggerLeftStickMultiplier;
  private final double leftTriggerRightStickMultiplier, rightTriggerRightStickMultiplier;

  private final SlewRateLimiter leftXRateLimiter, leftYRateLimiter, rightXRateLimiter, rightYRateLimiter;

  /**
   * <> creates a new {@link JoyUtil} with the provided values
   *
   * @param port                             the assigned DriverStation port
   * @param deadzone                         the deadzone size for both axis of both joysticks
   * @param rateLimitLeft                    the max output change that can occur in one second for the left joystick
   * @param rateLimitRight                   the max output change that can occur in one second for the right joystick
   * @param exponent1                        the first exponent of the joystick curve
   * @param exponent2                        the second exponent of the joystick curve
   * @param coefficient1                     the coefficient applied to the first exponent of joystick curve
   * @param coefficient2                     the coefficient applied to the second exponent of joystick curve
   * @param leftTriggerLeftStickMultiplier   the amount the left joystick output is multiplied by if the left trigger
   *                                         is pressed fully
   * @param rightTriggerLeftStickMultiplier  the amount the left joystick output is multiplied by if the right
   *                                         trigger is pressed fully
   * @param leftTriggerRightStickMultiplier  the amount the right joystick output is multiplied by if the left
   *                                         trigger is pressed fully
   * @param rightTriggerRightStickMultiplier the amount the right joystick output is multiplied by if the right
   *                                         trigger is pressed fully
   */
  public JoyUtil(int port, double deadzone, double rateLimitLeft, double rateLimitRight, double exponent1,
                 double exponent2, double coefficient1, double coefficient2, double leftTriggerLeftStickMultiplier,
                 double rightTriggerLeftStickMultiplier, double leftTriggerRightStickMultiplier,
                 double rightTriggerRightStickMultiplier) {
    super(port);

    this.deadzone = deadzone;

    this.leftXRateLimiter = new SlewRateLimiter(rateLimitLeft);
    this.leftYRateLimiter = new SlewRateLimiter(rateLimitLeft);
    this.rightXRateLimiter = new SlewRateLimiter(rateLimitRight);
    this.rightYRateLimiter = new SlewRateLimiter(rateLimitRight);

    this.exponent1 = exponent1;
    this.exponent2 = exponent2;
    this.coefficient1 = coefficient1;
    this.coefficient2 = coefficient2;

    this.leftTriggerLeftStickMultiplier = leftTriggerLeftStickMultiplier;
    this.rightTriggerLeftStickMultiplier = rightTriggerLeftStickMultiplier;
    this.leftTriggerRightStickMultiplier = leftTriggerRightStickMultiplier;
    this.rightTriggerRightStickMultiplier = rightTriggerRightStickMultiplier;

    // <> even exponents make for very weird behaviour so provide
    // a one-time warning in the rio log if there are even exponents
    if (exponent1 % 2 == 0 || exponent2 % 2 == 0) {
      System.out.println("Exponents of joystick curve aren't odd!");
    }
  }

  /**
   * <> creates a new {@link JoyUtil} with the values in constants
   *
   * @param port the port of the controller
   */
  public JoyUtil(int port) {
    this(port, JoyUtilConstants.kDeadzone,
      JoyUtilConstants.kRateLimitLeft, JoyUtilConstants.kRateLimitRight, JoyUtilConstants.exponent1,
      JoyUtilConstants.exponent2, JoyUtilConstants.coeff1, JoyUtilConstants.coeff2,
      JoyUtilConstants.leftTriggerSpeedMultiplier, JoyUtilConstants.rightTriggerSpeedMultiplier,
      JoyUtilConstants.leftTriggerSpeedMultiplier, JoyUtilConstants.rightTriggerSpeedMultiplier);
  }

  @Override
  public double getLeftX() {
    double rawOutput = super.getLeftX();
    double preRateLimiting = composeJoystickFunctions(rawOutput, leftTriggerLeftStickMultiplier,
      rightTriggerLeftStickMultiplier);

    return leftXRateLimiter.calculate(preRateLimiting);
  }

  @Override
  public double getRightX() {
    double rawOutput = super.getRightX();
    double preRateLimiting = composeJoystickFunctions(rawOutput, leftTriggerRightStickMultiplier,
      rightTriggerRightStickMultiplier);

    return rightXRateLimiter.calculate(preRateLimiting);
  }

  @Override
  public double getLeftY() {
    double rawOutput = super.getLeftY();
    double preRateLimiting = composeJoystickFunctions(rawOutput, leftTriggerLeftStickMultiplier,
      rightTriggerLeftStickMultiplier);

    return leftYRateLimiter.calculate(preRateLimiting);
  }

  @Override
  public double getRightY() {
    double rawOutput = super.getRightY();
    double preRateLimiting = composeJoystickFunctions(rawOutput, leftTriggerRightStickMultiplier,
      rightTriggerRightStickMultiplier);

    return rightYRateLimiter.calculate(preRateLimiting);
  }

  /**
   * <> get value of A button as a boolean
   *
   * @return the value of the A button
   */
  public boolean getAButton() {
    return a().getAsBoolean();
  }

  /**
   * <> get value of B button as a boolean
   *
   * @return the value of the B button
   */
  public boolean getBButton() {
    return b().getAsBoolean();
  }

  /**
   * <> get value of X button as a boolean
   *
   * @return the value of the X button
   */
  public boolean getXButton() {
    return x().getAsBoolean();
  }

  /**
   * <> get value of Y button as a boolean
   *
   * @return the value of the Y button
   */
  public boolean getYButton() {
    return y().getAsBoolean();
  }

  /**
   * <> get value of left bumper as a boolean
   *
   * @return the value of the left bumper
   */
  public boolean getLeftBumper() {
    return leftBumper().getAsBoolean();
  }

  /**
   * <> get value of right bumper as a boolean
   *
   * @return the value of the right bumper
   */
  public boolean getRightBumper() {
    return rightBumper().getAsBoolean();
  }

  /**
   * <> get value of left stick pressed in as boolean
   *
   * @return the value of the left stick
   */
  public boolean getLeftStick() {
    return leftStick().getAsBoolean();
  }

  /**
   * <> get value of right stick pressed in as boolean
   *
   * @return the value of the right stick
   */
  public boolean getRightStick() {
    return rightStick().getAsBoolean();
  }

  /**
   * <> get if the d-pad is pointed left
   *
   * @return if the d-pad is facing left
   */
  public boolean getPOVLeft() {
    return povLeft().getAsBoolean();
  }

  /**
   * <> get if the d-pad is pointed right
   *
   * @return if the d-pad is facing right
   */
  public boolean getPOVRight() {
    return povRight().getAsBoolean();
  }

  /**
   * <> get if the d-pad is pointed up
   *
   * @return if the d-pad is facing up
   */
  public boolean getPOVUp() {
    return povUp().getAsBoolean();
  }

  /**
   * <> get if the d-pad is pointed down
   *
   * @return if the d-pad is facing down
   */
  public boolean getPOVDown() {
    return povDown().getAsBoolean();
  }

  /**
   * <> get if the d-pad is pointed up left
   *
   * @return if the d-pad is facing up left
   */
  public boolean getPOVUpLeft() {
    return povUpLeft().getAsBoolean();
  }

  /**
   * <> get if the d-pad is pointed up right
   *
   * @return if the d-pad is facing up right
   */
  public boolean getPOVUpRight() {
    return povUpRight().getAsBoolean();
  }

  /**
   * <> get if the d-pad is pointed down left
   *
   * @return if the d-pad is facing down left
   */
  public boolean getPOVDownLeft() {
    return povDownLeft().getAsBoolean();
  }

  /**
   * <> get if the d-pad is pointed down right
   *
   * @return if the d-pad is facing down right
   */
  public boolean getPOVDownRight() {
    return povDownRight().getAsBoolean();
  }

  /**
   * <> get if the d-pad isn't pressed
   *
   * @return if the d-pad isn't pressed
   */
  public boolean getPOVNotPressed() {
    return !getPOVLeft() && !getPOVUpLeft() && !getPOVUp() && !getPOVUpRight() && !getPOVRight() && !getPOVDownRight() && !getPOVDown() && !getPOVDownLeft();
  }

  /**
   * <> gets the value of the d-pad y-axis (sin of pov angle)
   *
   * @return the value
   */
  public double getPOVYAxis() {
    if (getPOVUp()) {
      return 1;
    } else if (getPOVDown()) {
      return -1;
    } else if (getPOVUpLeft() || getPOVUpRight()) {
      return sqrt2Over2;
    } else if (getPOVDownLeft() || getPOVDownRight()) {
      return -sqrt2Over2;
    } else {
      return 0;
    }
  }

  /**
   * <> gets the value of the d-pad x-axis (cos of pov angle)
   *
   * @return the value
   */
  public double getPOVXAxis() {
    if (getPOVRight()) {
      return 1;
    } else if (getPOVLeft()) {
      return -1;
    } else if (getPOVUpRight() || getPOVDownRight()) {
      return sqrt2Over2;
    } else if (getPOVUpLeft() || getPOVDownLeft()) {
      return -sqrt2Over2;
    } else {
      return 0;
    }
  }

  /**
   * <> applies a deadzone to an input using the deadzone
   * specified in the object
   *
   * <p> the output of this functions can still be any number
   * from 0 to 1 to allow very small outputs to still be achieved </p>
   *
   * @param value the value to apply the deadzone to
   * @return the value with the deadzone applied
   */
  private double applyDeadzone(double value) {
    // <> apply the raw deadzone
    double deadzoned = MathUtil.applyDeadband(value, deadzone);

    // <> if raw deadzoning outputs 0, return 0 now
    if (deadzoned == 0) {
      return 0;
    }

    // <> the code now needs to take the output with an absolute value between the
    // deadzone and 1 and remap it so that its absolute value can be between 0
    // and 1. this looks complicated but that's just because the ranges have to be in the
    // negative numbers if the deadzoned value is negative
    int multiplier = deadzoned > 0 ? 1 : -1;
    return remap(deadzoned, deadzone * multiplier, multiplier, 0, multiplier);
  }

  /**
   * <> curves a given input
   *
   * @param value the value before the curve
   * @return the value curved
   */
  private double applyCurve(double value) {
    double term1 = coefficient1 * Math.pow(value, exponent1);
    double term2 = coefficient2 * Math.pow(value, exponent2);

    return term1 + term2;
  }

  /**
   * <> apply the left and right trigger multipliers
   *
   * @param value                  the value before having the multipliers applied
   * @param leftTriggerMultiplier  the multiplier that will be applied if the left trigger is pressed fully
   * @param rightTriggerMultiplier the multiplier that will be applied if the right trigger is pressed fully
   * @return the value with trigger multipliers applied
   */
  private double applyTriggerMultipliers(double value, double leftTriggerMultiplier, double rightTriggerMultiplier) {
    // get the amounts we need to multiply the raw value by (take the trigger input and
    // use it to linearly interpolate between 1 and the multiplier for the trigger)
    double realLeftTriggerMultiplier = remap(getLeftTriggerAxis(), 0, 1, 1, leftTriggerMultiplier);
    double realRightTriggerMultiplier = remap(getRightTriggerAxis(), 0, 1, 1, rightTriggerMultiplier);

    return value * realLeftTriggerMultiplier * realRightTriggerMultiplier;
  }

  /**
   * <> applies deadzoning, curve, and trigger multipliers to a raw input
   *
   * @param value                  the raw input from the joystick
   * @param leftTriggerMultiplier  the left trigger output multiplier for the axis being calculated
   * @param rightTriggerMultiplier the right trigger output multiplier for the axis being calculated
   * @return the raw input after being deadzoned, curved, and having trigger multipliers applied
   * @apiNote does not do any rate limiting
   */
  private double composeJoystickFunctions(double value, double leftTriggerMultiplier, double rightTriggerMultiplier) {
    double withDeadzone = applyDeadzone(value);
    double withCurve = applyCurve(withDeadzone);
    double withMultipliers = applyTriggerMultipliers(withCurve, leftTriggerMultiplier, rightTriggerMultiplier);

    return withMultipliers;
  }

  /**
   * <> simple (but very useful) math function that remaps a value from
   * one range of numbers to another range of numbers
   *
   * @param value the value to remap
   * @param low1  the lower bound of the range to map from
   * @param high1 the upper bound of the range to map from
   * @param low2  the lower bound of the range to map to
   * @param high2 the upper bound of the range to map to
   * @return the remapped value
   */
  private double remap(double value, double low1, double high1, double low2, double high2) {
    double range1Size = high1 - low1;
    double range2Size = high2 - low2;
    double percentIntoRange1 = (value - low1) / range1Size;

    return low2 + range2Size * percentIntoRange1;
  }
}
