package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType; 

import frc.robot.Constants;

/** ++ we'll use this class to write methods that help us process joystick inputs
* and will mostly be for drive train things, and will include things like:
* deadzone functions and joystick input curves, and anything else we need. 
*/
public final class JoyUtil extends XboxController {




    // ++ WE HAVE A BUNCH OF FUNCTIONS HERE, AND WE NEED TO APPLY THEM IN THE RIGHT ORDER
    // ++ check the "composeDriveJoyFunctions" method at the bottom to see the order this should be done in 
    // ++ (((I'm not putting it here to avoid multiple versions of the "correct" order)))


    /** creates a new JoyUtil joystick.
     * @param controllerID
     */
    public JoyUtil(int controllerID) {
        super(controllerID);
    }




    // ++  rumble stuff ----------------

    public void rumbleLeft(double strength) {
        setRumble(RumbleType.kLeftRumble, strength);
    }

    public void rumbleRight(double strength) {
        setRumble(RumbleType.kRightRumble, strength);
    }

    public void stopRumbleLeft() {
        setRumble(RumbleType.kLeftRumble, 0.0);
    }

    public void stopRumbleRight() {
        setRumble(RumbleType.kRightRumble, 0.0);
    }

    public void stopBothRumble() {
        setRumble(RumbleType.kLeftRumble, 0.0);
        setRumble(RumbleType.kRightRumble, 0.0);
    }



    // ++ end rumble stuff ------------



    // ++ these are the methods used to 
    double prevFilteredX;
    double prevFilteredY;
    double prevFilteredR;

    public void zeroPreviousFiltered() {
        // ++ this zeroes all the previous filtered values
        prevFilteredX = 0.0;
        prevFilteredY = 0.0;
        prevFilteredR = 0.0;
    }

    

    // ++ these methods make it so you don't have to pass anything in when you call them, you just call the 
    // ++ method that corresponds with the joystick you want. It also keeps track of the previous filtered value 
    public double getDriveStraightWithAdjustments(){
        double rawJoyPos = getLeftY(); // + (getDPadX()); 
        double filterStrength = Constants.Joysticks.driveLowPassFilterStrength;
        double damperStrength = Constants.DriveTrain.DriveConstants.kDrivingSpeedDamper;
        double adjustedPos = composeDriveJoyFunctions(rawJoyPos, prevFilteredX, filterStrength, damperStrength);

        // prevFilteredX = lowPassFilter(rawJoyPos, prevFilteredX, filterStrength);
        return adjustedPos;
    }
    public double getDriveStrafeWithAdjustments(){
        double rawJoyPos = -getLeftX(); // + (getDPadY()); 
        double filterStrength = Constants.Joysticks.driveLowPassFilterStrength;
        double damperStrength = Constants.DriveTrain.DriveConstants.kDrivingSpeedDamper;
        double adjustedPos = composeDriveJoyFunctions(rawJoyPos, prevFilteredY, filterStrength, damperStrength); 

        // prevFilteredY = lowPassFilter(rawJoyPos, prevFilteredY, filterStrength);
        return adjustedPos;
    }
    public double getRotationWithAdjustments() {
        // ++ this gets the position on the rotation axis, then adjusts it to what we need
        // ++ right now, we need a deadzone and then a low pass filter, but that might change later
        
        // ++ the rotation axis is right x
        double rawJoyPos = getRightX();
        double filterStrength = Constants.Joysticks.rotationLowPassFilterStrength;
        double damperStrength = Constants.DriveTrain.DriveConstants.kAngularSpeedDamper;
        double adjustedPos = ( lowPassFilter( posWithDeadzone(rawJoyPos), prevFilteredR, filterStrength) * damperStrength );
        return adjustedPos;
    }
    //meah for mayor





    // ++ these are the methods called above ===================================================================

    public static double posWithDeadzone(double pos) {
        // ++ takes input and compares it to deadzone size
        // returns joystick size if it's greater than the deadzone, 0 otherwise

        return MathUtil.applyDeadband(pos, Constants.Joysticks.deadZoneSize);
    }

  
    public static double lowPassFilter(double pos, double prevFilterJoy, double filterStrength) {
        // ++ this method smoothes out the joystick input so
        // ++ "prevFilterJoy" is the previous output of this function
        double filteredSpeed = ((filterStrength * prevFilterJoy) + ((1- filterStrength) * pos));
        return filteredSpeed;
    }


    public static double joyCurve(double pos) {
        // ++ this method will take the linear joystick input and puts it into a polynomial curve

        double a = Constants.Joysticks.aCoeff; 
        double b = Constants.Joysticks.bCoeff;
        int firstPower = Constants.Joysticks.firstPower;
        int secondPower = Constants.Joysticks.secondPower; 

        return ( (a * (Math.pow(pos,firstPower))) + (b * (Math.pow(pos,secondPower))) ); 
    }

    public static double fastMode(double pos, double fastmodeInput) {
        // ++ this method should return an adjusted joystick position with the fastmode
        // ++ fastmodeInput is the position of the trigger

        double fastmodeConstant = Constants.Joysticks.fastModeMaxMultiplier;
        double fastmodeMultiplier= (fastmodeInput * fastmodeConstant) + 1.0;
        double adjustedPos = pos * fastmodeMultiplier;
        return adjustedPos;

        /* ss finalMultiplier is the damperStrength scaled by the ((Right Trigger scaled by the fastModeMaxMultiplier) + 1)
        * for instance, if the damperStrength is 0.5 and the fastModeMaxMultiplier is 3, 
        * when the Right Trigger is 0, Fast Mode is off and the fastModeMaxMultiplier is nullified,
        * and the finalMultiplier is just damperStrength
        * when the Right Trigger is 0.5, fastModeMaxMultiplier is halved (1.5), and adds 1 for 2.5
        * so damperStrength, the default multiplier, is scaled up by half of the Maximum Multiplier
        * and when the Right Trigger is 1, it's scaled up by the Maximum.
        * hope that makes sense
        * I did this because it's a multiplier and it would sure be a shame 
        * if nullifying the fastmodemultiplier caused the finalmultiplier to be 0,
        * disabling non fast mode
        */
    }


    public double composeDriveJoyFunctions(double rawJoyPos, double prevFilterJoy, double filterStrength, double damperStrength){
        // ++ IMPORTANT: please note that this function now shouldn't be called outside of this class-- this class used to be
        // ++ just full of methods, but it's now a wrapper class

        /* ++ this method will compose all the previous joy functions, so
        * THIS WILL BE THE ONLY METHOD USED for adjusting the drive joysticks
        *
        * ++ we need to apply the methods in the RIGHT ORDER
        * we want to do:
        * - deadzone
        * - low pass filtering
        * - joy curve
        * - do fastmode stuff
        * - convert joystick range [-1, 1] to range of robot speed [-max speed, max speed]
        * - dampen the output w/ a multiplier
        * and the order might have to be changed as we add more functions, 
        * but deadzone should probably stay first, and dampening should probably stay last
        */ 

        double withDead = posWithDeadzone(rawJoyPos);
        double withFilter = lowPassFilter(withDead, prevFilterJoy, filterStrength);
        double withCurve = joyCurve(withFilter); 
        double withSpeedMode = fastMode(withCurve, (getRightTriggerAxis() - (getLeftTriggerAxis() * Constants.Joysticks.slowModeMultiplier)));
        double withDamper = withSpeedMode * Constants.DriveTrain.DriveConstants.kDrivingSpeedDamper;

        double adjustedJoyPos = withDamper;

        // ++ I decided to make seperate variables for everything to make it a little more readable /\

        return adjustedJoyPos;
        /* ++ we return [above variable] becasue that was the last thing done to the input; 
        it'll need to be changed if/when more functions are added */
    }


    /** ++ gets the x component vector of the D-Pad 
     * @return D-Pad x component
    */
    public double getDPadX(){
        if( getPOV() != -1 ) {
            return Math.cos( Math.toRadians(getPOV() - 90.0) ) * Constants.Joysticks.dPadDamper;
        } else {
            return 0.0;
        }
    }
 
    /** ++ gets the y component of the D-Pad 
     * @return D-Pad y component
    */
    public double getDPadY() {
        if (getPOV() != -1) {
            return Math.sin( Math.toRadians(getPOV() - 90.0)) * Constants.Joysticks.dPadDamper;
        } else {
            return 0.0;
        }
    }


}