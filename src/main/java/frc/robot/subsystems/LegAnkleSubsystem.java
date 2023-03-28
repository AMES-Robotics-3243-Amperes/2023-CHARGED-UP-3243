// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WristAndArm.Limits;
import frc.robot.utility_classes.GeneralUtil;
import frc.robot.utility_classes.LegAnklePosition;
import frc.robot.utility_classes.SemiAbsoluteEncoder;

import static frc.robot.Constants.WristAndArm.*;

public class LegAnkleSubsystem extends SubsystemBase {

  /**Performs inverse kinematics to get motor positions for given robot relative positions
   * H!
   * 
   * @param x The distance in front of the leg pivot
   * @param y The distance above the leg pivot
   * @param pitch The pitch relative to the horizontal forward direction in radians
   * @param roll The roll where 0 is up in radians
   * 
   * @return A {@link LegAnklePosition} object with the positions the motors should go to to acomplish this
   */
  public static LegAnklePosition IK(double x, double y, double pitch, double roll) {
    // H! Inverse kinematics: see more detailed math here: https://www.desmos.com/calculator/l89yzwijul \
    double targetArmAngle = pivotEncoderOffset + Math.atan2(y - Constants.WristAndArm.wristLength * Math.sin(pitch),   x - Constants.WristAndArm.wristLength * Math.cos(pitch) );
    /*
    if (targetY - Constants.WristAndArm.wristLength * Math.sin(targetPitch) >= 0) {
      targetArmAngle = Math.atan((targetX - Constants.WristAndArm.wristLength * Math.cos(targetPitch)) / -(targetY - Constants.WristAndArm.wristLength * Math.sin(targetPitch))) + Math.PI / 2;
    } else {
      targetArmAngle = Math.atan(-(targetY - Constants.WristAndArm.wristLength * Math.sin(targetPitch))  /  (targetX - Constants.WristAndArm.wristLength * Math.cos(targetPitch))) + 3 * Math.PI / 2;
    }*/
    
    double targetArmLength; 
    if (targetArmAngle == 0) {
      targetArmLength = Math.abs(x - Constants.WristAndArm.wristLength * Math.cos(pitch));
    } else {
      targetArmLength = (y - Constants.WristAndArm.wristLength * Math.sin(pitch)) / Math.sin(targetArmAngle);
    }
    double targetWristAngle = Math.PI - targetArmAngle + pitch;
    double targetWristRoll = roll;

    // H! Convert angles to motor rotations
    targetArmAngle /= 2 * Math.PI;
    targetWristAngle /= 2 * Math.PI;
    targetWristRoll /= 2* Math.PI;

    // 0.5 is up for the roll encoder, but 0 for the inverse kinematics
    targetWristRoll += 0.5;
    targetWristAngle += 0.5;

    return new LegAnklePosition(targetArmLength, targetArmAngle, targetWristAngle, targetWristRoll);
  }




  public boolean deleteThis_doSetpoint = true;

  private DigitalInput extensionLimitSwitch = new DigitalInput(0);
  private DigitalInput pivotLimitSwitch = new DigitalInput(1);
  
  private SparkMaxPIDController pidPivot;
  private SparkMaxPIDController pidExtension;
  private SparkMaxPIDController pidPitch;
  private SparkMaxPIDController pidRoll;
  
  public CANSparkMax motorPivot = new CANSparkMax(MotorIDs.armPivot, MotorType.kBrushless);
  public CANSparkMax motorExtension = new CANSparkMax(MotorIDs.armExtension, MotorType.kBrushless);
  public CANSparkMax motorPitchRight = new CANSparkMax(MotorIDs.WristPitchRight, MotorType.kBrushless);
  public CANSparkMax motorPitchLeft = new CANSparkMax(MotorIDs.WristPitchLeft, MotorType.kBrushless);
  public CANSparkMax motorRoll = new CANSparkMax(MotorIDs.WristRoll, MotorType.kBrushless);

  // H! This whole thing with aliases does actually work, I checked this

  /**An alias for the motor controller that leads pitch
   * H!
   */
  public CANSparkMax motorPitchLeader = motorPitchLeft;
  /**An alias for the motor controller that follows pitch
   * H!
   */
  public CANSparkMax motorPitchFollower = motorPitchRight;

  public SparkMaxAbsoluteEncoder encoderPivotAbsolute = motorPivot.getAbsoluteEncoder(Type.kDutyCycle);
  public RelativeEncoder encoderPivotRelative = motorPivot.getEncoder();
  public RelativeEncoder encoderExtension = motorExtension.getEncoder(/*Type.kDutyCycle*/);
  public SemiAbsoluteEncoder encoderPitch;
  public RelativeEncoder encoderPitchRelative;
  public SparkMaxAbsoluteEncoder encoderPitchAbsolute = motorPitchLeader.getAbsoluteEncoder(Type.kDutyCycle);
  public RelativeEncoder encoderPitchRight = motorPitchRight.getEncoder();
  public RelativeEncoder encoderPitchLeft = motorPitchLeft.getEncoder();
  public RelativeEncoder encoderRollRelative;
  public SemiAbsoluteEncoder encoderRoll;

  private RelativeEncoder encoderPitchLeader = motorPitchLeader.getEncoder();
  private RelativeEncoder encoderPitchFollower = motorPitchFollower.getEncoder();

  private final LegAnklePosition startingPosition = IK(StartingPosition.x, StartingPosition.y, StartingPosition.pitch, StartingPosition.roll);
  private LegAnklePosition targetPosition = startingPosition;

  // H! FOR TESTING PURPOSES
  private ShuffleboardTab tab = Shuffleboard.getTab("Arm Testing");

  // H! FOR TESTING PURPOSES
  private GenericEntry extensionPValue;
  private GenericEntry extensionIValue;
  private GenericEntry extensionDValue;
  private GenericEntry extensionFFValue;

  private GenericEntry pivotPValue;
  private GenericEntry pivotIValue;
  private GenericEntry pivotDValue;
  private GenericEntry pivotFFValue;

  private GenericEntry pitchPValue;
  private GenericEntry pitchIValue;
  private GenericEntry pitchDValue;
  private GenericEntry pitchFFValue;

  private GenericEntry rollPValue;
  private GenericEntry rollIValue;
  private GenericEntry rollDValue;
  private GenericEntry rollFFValue;

  
  // :D Ahctipredocne

  /** Creates a new LegAnkleSubsystem. */
  public LegAnkleSubsystem() {
    // H! Hey, if the pitch just kinda isn't or you changed which motor is right and left, uncomment this and deploy it once, then recomment it.
    // H! You'll then need to manually set the conversion factor of the CANSparkMaxs through Rev Hardware Client
    //motorPitchRight.restoreFactoryDefaults();
    //motorPitchLeft.restoreFactoryDefaults();
    encoderPitchAbsolute.setInverted(false); // :D hi I changed this from true to false, since it was previously inverted from what the standard is on the pivot

    encoderPitch = new SemiAbsoluteEncoder(motorPitchLeader);
    encoderRoll = new SemiAbsoluteEncoder(motorRoll);

    encoderPitchRelative = encoderPitch.getSparkMAXEncoder();

    // H! Invert the motors that need to be inverted
    motorExtension.setInverted(true);
    motorRoll.setInverted(false);
    motorPivot.setInverted(false);
    motorPitchRight.setInverted(true);
    motorPitchLeft.setInverted(false);



    motorPitchFollower.follow(motorPitchLeader, true);

    pidPivot = motorPivot.getPIDController();
    pidExtension = motorExtension.getPIDController();
    pidPitch = motorPitchLeader.getPIDController();
    pidRoll = motorRoll.getPIDController();


    // :D I added this in, it limits the output voltage of the motor
    pidPivot.setOutputRange(-pivotOutputRange, pivotOutputRange);
    pidPitch.setOutputRange(-pitchOutputRange, pitchOutputRange);
    pidExtension.setOutputRange(-extensionOutputRange, extensionOutputRange);
    pidRoll.setOutputRange(-rollOutputRange, rollOutputRange);
    


    // H! Set the position conversion factors. Pivot is commented out because it didn't want to set. It's burned to flash manually
    encoderExtension.setPositionConversionFactor(extensionEncoderConversionFactor);
    //armPivotEncoder.setPositionConversionFactor(1/10);//(1 / 100) * (35/50) * (21/32) = 0.00459357
    encoderPivotAbsolute.setPositionConversionFactor(pivotEncoderConversionFactor); // TODO :D Check this value
    encoderPitchRight.setPositionConversionFactor(pitchEncoderConversionFactor);
    encoderPitchLeft.setPositionConversionFactor(pitchEncoderConversionFactor);
    

    
    //MotorPos startingMotorPosition = IK(StartingPosition.x, StartingPosition.y, StartingPosition.pitch, StartingPosition.roll);
    encoderExtension.setPosition(startingPosition.extension/*minLength*/);
    
    //encoderPivotRelative.setPosition(startingPosition.pivot); // :D I commented this out because we are using a semiabsolute encoder now
    encoderPitch.setZeroOffset(wristPitchEncoderSetZeroOffset);
    encoderPivotAbsolute.setZeroOffset(wristPivotEncoderSetZeroOffset); // TODO :D check this value
    // :D ^ this is in between the extreme values, so that the seam has the pivot facing where it physically can't go
    // :D the straight up direction is 0.43 on the absolute encoder
    // H! Set the left pitch encoder 
    //encoderPitchFollower.setPosition(encoderPitch.getPosition());

    // H! Used to reset the absolute encoder. Do not run this unless that's what you want to do
    //wristRollEncoder.setZeroOffset(0);
    //wristRollEncoder.setZeroOffset(wristRollEncoder.getPosition() - 0.5);
    // :D hi I turned this into a constant // H! Great!
    encoderRoll.setZeroOffset(wristRollEncoderSetZeroOffset);

    encoderPitchRight.setPosition(encoderPitch.getPosition());

    encoderRollRelative = encoderRoll.getSparkMAXEncoder();
    encoderRoll.setPositionConversionFactor(1/75); // :D TODO <constant


    pidRoll.setFeedbackDevice(encoderRollRelative);
    pidPivot.setFeedbackDevice(encoderPivotAbsolute);
    pidPitch.setFeedbackDevice(encoderPitchRelative);
    

    // H! Set the PID values initially
    setPIDFValues(pidExtension, PID.Extension.P, PID.Extension.I, PID.Extension.D, PID.Extension.FF); 
    setPIDFValues(pidPivot,     PID.Pivot.P,     PID.Pivot.I,     PID.Pivot.D,     PID.Pivot.FF); 
    setPIDFValues(pidPitch,   PID.Pitch.P,     PID.Pitch.I,     PID.Pitch.D,     PID.Pitch.FF);
    setPIDFValues(pidRoll,    PID.Roll.P,      PID.Roll.I,      PID.Roll.D,      PID.Roll.FF);
    
    // H! Set soft current limits
    motorPivot.setSmartCurrentLimit(pivotCurrentLimit);
    motorExtension.setSmartCurrentLimit(extensionCurrentLimit);
    motorPitchRight.setSmartCurrentLimit(pitchCurrentLimit);
    motorPitchLeft.setSmartCurrentLimit(pitchCurrentLimit);
    motorRoll.setSmartCurrentLimit(rollCurrentLimit);

    // H! Set hard current limits
    motorPivot.setSecondaryCurrentLimit(NEO1650CurrentLimitHard);
    motorExtension.setSecondaryCurrentLimit(NEO1650CurrentLimitHard);
    motorPitchRight.setSecondaryCurrentLimit(NEO550CurrentLimitHard);
    motorPitchLeft.setSecondaryCurrentLimit(NEO550CurrentLimitHard);
    motorRoll.setSecondaryCurrentLimit(NEO550CurrentLimitHard);




    motorRoll.burnFlash();
    motorPivot.burnFlash();
    motorPitchLeft.burnFlash();
    motorPitchRight.burnFlash();

    // H! Make all the PID tuning tabs in shuffleboard
    extensionPValue = tab.add("Ext P Value", PID.Extension.P).getEntry();
    extensionIValue = tab.add("Ext I Value", PID.Extension.I).getEntry();
    extensionDValue = tab.add("Ext D Value", PID.Extension.D).getEntry();
    extensionFFValue = tab.add("Ext FF Value", PID.Extension.FF).getEntry();

    pivotPValue = tab.add("Piv P Value", PID.Pivot.P).getEntry();
    pivotIValue = tab.add("Piv I Value", PID.Pivot.I).getEntry();
    pivotDValue = tab.add("Piv D Value", PID.Pivot.D).getEntry();
    pivotFFValue = tab.add("Piv FF Value", PID.Pivot.FF).getEntry();

    pitchPValue = tab.add("Pch P Value", PID.Pitch.P).getEntry();
    pitchIValue = tab.add("Pch I Value", PID.Pitch.I).getEntry();
    pitchDValue = tab.add("Pch D Value", PID.Pitch.D).getEntry();
    pitchFFValue = tab.add("Pch FF Value", PID.Pitch.FF).getEntry();

    rollPValue = tab.add("Rol P Value", PID.Roll.P).getEntry();
    rollIValue = tab.add("Rol I Value", PID.Roll.I).getEntry();
    rollDValue = tab.add("Rol D Value", PID.Roll.D).getEntry();
    rollFFValue = tab.add("Rol FF Value", PID.Roll.FF).getEntry();
  }











  public void resetRoll() {
    targetPosition.roll = 0.0;
  }


  public void updatePIDValues() {
    setPIDFValues(pidExtension,
      extensionPValue.getDouble(PID.Extension.P), 
      extensionIValue.getDouble(PID.Extension.I), 
      extensionDValue.getDouble(PID.Extension.D), 
      extensionFFValue.getDouble(PID.Extension.FF)
    );

    setPIDFValues(pidPivot, 
      pivotPValue.getDouble(PID.Pivot.P), 
      pivotIValue.getDouble(PID.Pivot.I), 
      pivotDValue.getDouble(PID.Pivot.D), 
      pivotFFValue.getDouble(PID.Pivot.FF)
    );

    setPIDFValues(pidPitch, 
      pitchPValue.getDouble(PID.Pitch.P), 
      pitchIValue.getDouble(PID.Pitch.I), 
      pitchDValue.getDouble(PID.Pitch.D), 
      pitchFFValue.getDouble(PID.Pitch.FF)
    );

    setPIDFValues(pidRoll, 
      rollPValue.getDouble(PID.Roll.P), 
      rollIValue.getDouble(PID.Roll.I), 
      rollDValue.getDouble(PID.Roll.D), 
      rollFFValue.getDouble(PID.Roll.FF)
    );
  }



  public LegAnklePosition getMotorPosition() {
    return new LegAnklePosition(
      encoderExtension.getPosition(), 
      encoderPivotAbsolute.getPosition(), 
      encoderPitch.getPosition(), 
      encoderRoll.getPosition()
    );
  }

  /**Set the motor positions the legAnkle will go to 
   * H!
   * 
   * @param newPosition A {@link LegAnklePosition} object with the positions to go to
   */
  public void setMotorPositions(LegAnklePosition newPosition) {
    
    System.out.println("$$$$$$$$$$$$$$$$$$$$$$$");
    System.out.println(newPosition.extension);
    System.out.println(newPosition.pivot);
    System.out.println(newPosition.pitch);
    System.out.println(newPosition.roll);
    System.out.println("$$$$$$$$$$$$$$$$$$$$$$$");
    setMotorPositions(newPosition.extension, newPosition.pivot, newPosition.pitch, newPosition.roll);
  }

  /**Set the motor positions the legAnkle will go to 
   * H!
   * 
   * @param extension The extension to go to
   * @param pivot The pivot to go to
   * @param pitch The pitch to go to
   * @param roll The roll to go to
   */
  public void setMotorPositions(Double extension, Double pivot, Double pitch, Double roll) {
    if(extension != null){
      targetPosition.extension = extension;
    }
    if(pitch != null){
      targetPosition.pitch = pitch;
    }
    if(pivot != null){
      targetPosition.pivot = pivot;
    }
    if(roll != null){
      targetPosition.roll = roll;
    }
  }

  /**Moves the motor positions the legAnkle will go to by a given amount
   * H!
   * 
   * @param extension The extension amount to move the setpoint by
   * @param pivot The pivot amount to move the setpoint by
   * @param pitch The pitch amount to move the setpoint by
   * @param roll The roll amount to move the setpoint by
   */
  public void changeMotorPositions(double extension, double pivot, double pitch, double roll) {
    targetPosition.extension += extension;
    targetPosition.pivot += pivot;
    targetPosition.pitch += pitch;
    targetPosition.roll += roll;
  }


  /**Sets the x, y, robot relative pitch, and roll that the legAnkle will try to go to 
   * H!
   * 
   * @param x The x that the leg ankle will try to go to
   * @param y The y that the leg ankle will try to go to
   * @param pitch The robot relative pitch (instead of arm relative pitch) that the leg ankle will try to go to
   * @param roll The roll that the leg ankle will try to go to
   */
  public void setKinematicPositions(double x, double y, double pitch, double roll) {
    targetPosition = IK(x, y, pitch, roll);
  }

  /**Changes the x, y, robot relative pitch, and roll that the legAnkle will try to go to by a given amount
   * H!
   * 
   * @param x The change in the x that the leg ankle will try to go to
   * @param y The change in the y that the leg ankle will try to go to
   * @param pitch The change in the robot relative pitch (instead of arm relative pitch) that the leg ankle will try to go to
   * @param roll The change in the roll that the leg ankle will try to go to
   */
  public void changeKinematicPositions(double x, double y, double pitch, double roll) {
    // H! Get the current setpoint positions in x, y, robot relative pitch, and roll
    double[] currentKinematicPosiitons = targetPosition.getKinematicPositions();
    
    setKinematicPositions(
      currentKinematicPosiitons[0] + x,
      currentKinematicPosiitons[1] + y,
      currentKinematicPosiitons[2] + pitch,
      currentKinematicPosiitons[3] + roll
    );
  }

  /**Tells whether the arm is within a small distance of its current setpoint
   * H!
   * 
   * @return A boolean for whether the arm is in a small distance of all of its setpoints.
   */
  public boolean isArmPositioned() {
    // H! Return whether it's in the right position
    // H! TODO: TEST THIS // :D tested enough already, right?
    return (
      (targetPosition.pivot == null || Math.abs( encoderPivotAbsolute.getPosition() - targetPosition.pivot ) < atSetpointThreshold) &&
      (targetPosition.extension == null || Math.abs( encoderExtension.getPosition() - targetPosition.extension ) < atSetpointThreshold) &&
      (targetPosition.pitch == null || Math.abs( encoderPitch.getPosition() - targetPosition.pitch ) < atSetpointThreshold) &&
      (targetPosition.roll == null || Math.abs( encoderRoll.getPosition() - targetPosition.roll ) < atSetpointThreshold)
    );
  }


  /**Sets the motor speeds of each motor manually
   * H!
   * 
   * @param pivot The motor value to set the pivot motor to
   * @param extension The motor value to set the extension motor to
   * @param pitch The motor value to set the pitch motor to
   * @param roll The motor value to set the roll motor to
   * 
   * @deprecated Using direct motor speeds is unideal, use {@link changeMotorPositions} to change the position the leg will try to go to
   */
  @Deprecated
  public void setMotorSpeeds(double pivot, double extension, double pitch, double roll) {
    motorExtension.set(extension / 10);
    motorPivot.set(pivot / 10);
    motorPitchRight.set(pitch / 10);
    motorRoll.set(roll / 10);
  }


  /**Returns the current setpoints that the leg ankle is trying to go to
   * H!
   * 
   * @return The current setpoints that the leg ankle is trying to go to in the form of a {@link LegAnklePosition} object
   */
  public LegAnklePosition getManualSetpoints() {
    return targetPosition;
  }









  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // H! ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // H! ~~~~~~~ Debug output ~~~~~~~
    // H! ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    // H! find kinematic position and display it
    // :D I commented these out because it's cluttering shuffleboard
    double[] kinematicSetpoints = targetPosition.getKinematicPositions();
    // SmartDashboard.putNumber("targetX", kinematicSetpoints[0]);
    // SmartDashboard.putNumber("targetY", kinematicSetpoints[1]);
    // SmartDashboard.putNumber("targetIKPitch", kinematicSetpoints[2]);
    // SmartDashboard.putNumber("targetIKRoll", kinematicSetpoints[3]);

    // SmartDashboard.putNumber("rollRelativeEncoder", encoderRoll.getSparkMAXEncoder().getPosition());



    // H! ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // H! ~~~~~~~ Setpoint processing ~~~~~~~
    // H! ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    // H! If the limit switch is triggered, we're at min extension.
    SmartDashboard.putBoolean("limit switch pressed", extensionLimitSwitch.get());
    if (extensionLimitSwitch.get()) {
      //encoderExtension.setPosition(Limits.extensionMin); Disabled because mechanical keeps not doing the limit switch properly
    }

    // ++ clamp values to be safe -------------------------------------------
    // H! Prevent arm from extending too much or too little
    targetPosition.extension = GeneralUtil.clamp(Limits.extensionMin, Limits.extensionMax, targetPosition.extension);

    // H! Prevent arm from pivoting too much or too little
    targetPosition.pivot = GeneralUtil.clamp(Limits.pivotMin, Limits.pivotMax, targetPosition.pivot);

    // H! Prevent arm from pivoting too much or too little
    targetPosition.pitch = GeneralUtil.clamp(Limits.pitchMin, Limits.pitchMax, targetPosition.pitch);

    // H! Prevent arm from pivoting too much or too little
    targetPosition.roll = GeneralUtil.clamp(Limits.rollMin, Limits.rollMax, targetPosition.roll);


    // H! NOTE: If you add xy limits, make sure to remove the comment in constants saying they're only used with IK
    
    // ++ pivot
    // targetPosition.pivot = clamp(minPivotPos, maxPivotPos, targetPosition.pivot);
    // ++ pitch
    // targetPosition.pitch = clamp (minPitchPos, maxPitchPos, targetPosition.pitch);
    // ++ ----------------------

    // H! Set the position refrences
    targetPosition.applyToMotors(pidExtension, pidPivot, pidPitch,  pidRoll);
  }



  public void testPeriodic() {
    // H! This is maybe right TODO actually test it
    if (pivotLimitSwitch.get()) {
      encoderPivotAbsolute.setZeroOffset(encoderPivotAbsolute.getZeroOffset() + 0.5);
    }
  }










  /**Sets the PIDF values of a pid controller
   * H!
   * 
   * @param pidController The PID controller to set the values of
   * @param p The P value to use
   * @param i The I value to use
   * @param d The D value to use
   * @param f The F value to use
   */
  private static void setPIDFValues(SparkMaxPIDController pidController, double p, double i, double d, double f) {
    pidController.setP(p);
    pidController.setI(i);
    pidController.setD(d);
    pidController.setFF(f);
  }
}