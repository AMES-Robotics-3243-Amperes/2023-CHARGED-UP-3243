// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility_classes.SemiAbsoluteEncoder;

import static frc.robot.Constants.WristAndArm.*;

public class LegAnkleSubsystem extends SubsystemBase {

  public class MotorPos {
    public double extension;
    public double pivot;
    public double pitch;
    public double roll; 

    public MotorPos(double extension, double pivot, double pitch, double roll) {
      this.extension = extension;
      this.pivot = pivot;
      this.pitch = pitch;
      this.roll = roll;
    }
  }

  public MotorPos IK(double x, double y, double pitch, double roll) {
    // H! Inverse kinematics: see more detailed math here: https://www.desmos.com/calculator/l89yzwijul \
    double targetArmAngle = Math.atan2(y - Constants.WristAndArm.wristLength * Math.sin(pitch),   x - Constants.WristAndArm.wristLength * Math.cos(pitch) );
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

    targetWristRoll += 0.5;

    return new MotorPos(targetArmLength, targetArmAngle, targetWristAngle, targetWristRoll);
  }




  public boolean deleteThis_doSetpoint = true;

  private boolean PIDControl = true;

  private boolean manualSetpoints = true;

  private int counter = 0;

  private DigitalInput extensionLimitSwitch = new DigitalInput(0);
  
  private SparkMaxPIDController pidArmPivot;
  private SparkMaxPIDController pidArmExtension;
  private SparkMaxPIDController pidWristPitch;
  private SparkMaxPIDController pidWristRoll;
  
  private CANSparkMax armPivot = new CANSparkMax(MotorIDs.armPivot, MotorType.kBrushless);
  private CANSparkMax armExtension = new CANSparkMax(MotorIDs.armExtension, MotorType.kBrushless);
  private CANSparkMax wristPitchRight = new CANSparkMax(MotorIDs.WristPitchRight, MotorType.kBrushless);
  private CANSparkMax wristPitchLeft = new CANSparkMax(MotorIDs.WristPitchLeft, MotorType.kBrushless);
  private CANSparkMax wristRoll = new CANSparkMax(MotorIDs.WristRoll, MotorType.kBrushless);

  private RelativeEncoder armPivotEncoder = armPivot.getEncoder(/*Type.kDutyCycle*/);
  private RelativeEncoder armExtensionEncoder = armExtension.getEncoder(/*Type.kDutyCycle*/);
  private SemiAbsoluteEncoder wristPitchEncoder = new SemiAbsoluteEncoder(wristPitchLeft); // H! TODO which motor controller this is plugged into is unconfimed
  private RelativeEncoder wristPitchEncoderLeft = wristPitchEncoder.getSparkMAXEncoder();
  // H! TODO REMOVE THIS
  private SparkMaxAbsoluteEncoder wristPitchEncoderLeftAbsolute = wristPitchLeft.getAbsoluteEncoder(Type.kDutyCycle);

  private RelativeEncoder wristPitchEncoderRight = wristPitchRight.getEncoder(/*Type.kDutyCycle*/);
  private RelativeEncoder wristRollSparkMAXEncoder;
  private SemiAbsoluteEncoder wristRollEncoder = new SemiAbsoluteEncoder(wristRoll);

  private double targetX = StartingSetpoints.x;
  private double targetY = StartingSetpoints.y;
  private double targetPitch = StartingSetpoints.pitch;
  private double targetRoll = StartingSetpoints.roll;

  private MotorPos startingPosition = IK(targetX, targetY, targetPitch, targetRoll);
  private double targetPivotSetpoint = startingPosition.pivot;
  private double targetExtensionSetpoint = startingPosition.extension;
  private double targetPitchSetpoint = startingPosition.pitch;
  private double targetRollSetpoint = startingPosition.roll;

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

  


  /** Creates a new LegAnkleSubsystem. */
  public LegAnkleSubsystem() {
    armExtension.setInverted(true);

    wristPitchLeft.follow(wristPitchRight, true);

    extensionPValue = tab.add("Ext P Value", PID.Extension.P).getEntry();
    extensionIValue = tab.add("Ext I Value", PID.Extension.I).getEntry();
    extensionDValue = tab.add("Ext D Value", PID.Extension.D).getEntry();
    extensionFFValue = tab.add("Ext FF Value", PID.Extension.FF).getEntry();

    pivotPValue = tab.add("Piv P Value", PID.Pivot.P).getEntry();
    pivotIValue = tab.add("Piv I Value", PID.Pivot.I).getEntry();
    pivotDValue = tab.add("Piv D Value", PID.Pivot.D).getEntry();
    pivotFFValue = tab.add("Piv FF Value", PID.Pivot.FF).getEntry();

    pitchPValue = tab.add("Pch P Value", PID.Roll.P).getEntry();
    pitchIValue = tab.add("Pch I Value", PID.Roll.I).getEntry();
    pitchDValue = tab.add("Pch D Value", PID.Roll.D).getEntry();
    pitchFFValue = tab.add("Pch FF Value", PID.Roll.FF).getEntry();

    rollPValue = tab.add("Rol P Value", PID.Roll.P).getEntry();
    rollIValue = tab.add("Rol I Value", PID.Roll.I).getEntry();
    rollDValue = tab.add("Rol D Value", PID.Roll.D).getEntry();
    rollFFValue = tab.add("Rol FF Value", PID.Roll.FF).getEntry();


    pidArmPivot = armPivot.getPIDController();
    pidArmExtension = armExtension.getPIDController();
    pidWristPitch = wristPitchRight.getPIDController();
    pidWristRoll = wristRoll.getPIDController();

    // :D I added this in
    pidArmPivot.setOutputRange(-pivotOutputRange, pivotOutputRange);

    

    pidArmPivot.setFeedbackDevice(armPivotEncoder);

    armExtensionEncoder.setPositionConversionFactor(extensionEncoderConversionFactor);
    //armPivotEncoder.setPositionConversionFactor(1/10);//(1 / 100) * (35/50) * (21/32) = 0.00459357
    wristPitchEncoderRight.setPositionConversionFactor(pitchEncoderConversionFactor);
    wristPitchEncoderLeft.setPositionConversionFactor(pitchEncoderConversionFactor);
    
    
    
    wristRoll.setInverted(true);

    
    MotorPos startingMotorPosition = IK(StartingPosition.x, StartingPosition.y, StartingPosition.pitch, StartingPosition.roll);
    armExtensionEncoder.setPosition(startingMotorPosition.extension/*minLength*/);
    armPivotEncoder.setPosition(startingMotorPosition.pivot);
    wristPitchEncoderLeft.setPosition(wristPitchEncoder.getPosition());

    // H! Used to reset the absolute encoder. Do not run this unless that's what you want to do
    //wristRollEncoder.setZeroOffset(0);
    //wristRollEncoder.setZeroOffset(wristRollEncoder.getPosition() - 0.5);
    // :D hi I turned this into a constant // H! Great!
    wristRollEncoder.setZeroOffset(wristRollEncoderSetZeroOffset);

    wristRollSparkMAXEncoder = wristRollEncoder.getSparkMAXEncoder();
    wristRollEncoder.setPositionConversionFactor(1/75);

    pidWristRoll.setFeedbackDevice(wristRollSparkMAXEncoder);

    

    wristRoll.burnFlash();

    

    

    // H! Set the PID values initially
    setPIDFValues(pidArmExtension, PID.Extension.P, PID.Extension.I, PID.Extension.D, PID.Extension.FF); 
    setPIDFValues(pidArmPivot,     PID.Pivot.P,     PID.Pivot.I,     PID.Pivot.D,     PID.Pivot.FF); 
    setPIDFValues(pidWristPitch,   PID.Pitch.P,     PID.Pitch.I,     PID.Pitch.D,     PID.Pitch.FF); 
    setPIDFValues(pidWristRoll,    PID.Roll.P,      PID.Roll.I,      PID.Roll.D,      PID.Roll.FF);
    
    // H! Set soft current limits
    armPivot.setSmartCurrentLimit(pivotCurrentLimit);
    armExtension.setSmartCurrentLimit(extensionCurrentLimit);
    wristPitchRight.setSmartCurrentLimit(pitchCurrentLimit);
    wristPitchLeft.setSmartCurrentLimit(pitchCurrentLimit);
    wristRoll.setSmartCurrentLimit(rollCurrentLimit);

    // H! Set hard current limits
    armPivot.setSecondaryCurrentLimit(NEO1650CurrentLimitHard);
    armExtension.setSecondaryCurrentLimit(NEO1650CurrentLimitHard);
    wristPitchRight.setSecondaryCurrentLimit(NEO550CurrentLimitHard);
    wristPitchLeft.setSecondaryCurrentLimit(NEO550CurrentLimitHard);
    wristRoll.setSecondaryCurrentLimit(NEO550CurrentLimitHard);
  }


  public void resetRoll() {
    targetRoll = 0;
  }


  public void updatePIDValues() {
    setPIDFValues(pidArmExtension,
      extensionPValue.getDouble(PID.Extension.P), 
      extensionIValue.getDouble(PID.Extension.I), 
      extensionDValue.getDouble(PID.Extension.D), 
      extensionFFValue.getDouble(PID.Extension.FF)
    );

    setPIDFValues(pidArmPivot, 
      pivotPValue.getDouble(PID.Pivot.P), 
      pivotIValue.getDouble(PID.Pivot.I), 
      pivotDValue.getDouble(PID.Pivot.D), 
      pivotFFValue.getDouble(PID.Pivot.FF)
    );

    setPIDFValues(pidWristPitch, 
      pitchPValue.getDouble(PID.Pitch.P), 
      pitchIValue.getDouble(PID.Pitch.I), 
      pitchDValue.getDouble(PID.Pitch.D), 
      pitchFFValue.getDouble(PID.Pitch.FF)
    );

    setPIDFValues(pidWristRoll, 
      rollPValue.getDouble(PID.Roll.P), 
      rollIValue.getDouble(PID.Roll.I), 
      rollDValue.getDouble(PID.Roll.D), 
      rollFFValue.getDouble(PID.Roll.FF)
    );
  }



  /** H! Moves the arm-wrist assembly by a given position difference
   *
   * @param x The difference in distance in front of the pivot
   * @param y The difference in distance above the target
   * @param pitch The difference in pitch to approach at
   * @param roll The difference in roll to approach at
  */
  public void moveByXYTheta(double x, double y, double pitch, double roll) {
    moveToXYTheta(
      targetX + (Math.abs(x) < 0.01 ? 0 : x * changeXMultiplier),
      targetY + (Math.abs(y) < 0.01 ? 0 : y * changeYMultiplier),
      targetPitch + (Math.abs(pitch) < 0.01 ? 0 : pitch * changePitchMultiplier),
      targetRoll + (Math.abs(roll) < 0.01 ? 0 : roll * changeRollMultiplier)
    );
  }

  /** H! Moves the arm-wrist assembly to a given position and rotation.
   *
   * @param xIn The target distance in front of the pivot
   * @param yIn The target distance above the pivot
   * @param pitchIn The target pitch to approach at
   * @param rollIn The target roll to approach at
   * @return Whether the arm is in a small range of the target
  */
  public boolean moveToXYTheta(double xIn, double yIn, double pitchIn, double rollIn) {
    // H! Prevent the arm from going places it shouldn't with clamps
    targetX = roundAvoid(clamp(minX, maxX, xIn ), 5);
    targetY = roundAvoid(clamp(minY, maxY, yIn ), 5);
    targetPitch = pitchIn;
    targetRoll = rollIn; 

    MotorPos targetMotorPositions = IK(targetX, targetY, targetPitch, targetRoll);

    // H! Return whether it's in the right position
    return (
      Math.abs( armPivotEncoder.getPosition() - targetMotorPositions.pivot ) < atSetpointThreshold &&
      Math.abs( armExtensionEncoder.getPosition() - targetMotorPositions.extension ) < atSetpointThreshold &&
      Math.abs( wristPitchEncoder.getPosition() - targetMotorPositions.pitch ) < atSetpointThreshold &&
      Math.abs( wristRollEncoder.getPosition() - targetMotorPositions.roll ) < atSetpointThreshold
    );

  }


  public Boolean nearTargetPos() {
    // H! Return whether it's in the right position
    // H! TODO TEST THIS
    return (
      Math.abs( armPivotEncoder.getPosition() - targetPivotSetpoint ) < atSetpointThreshold &&
      Math.abs( armExtensionEncoder.getPosition() - targetExtensionSetpoint ) < atSetpointThreshold &&
      Math.abs( wristPitchEncoder.getPosition() - targetPitchSetpoint ) < atSetpointThreshold &&
      Math.abs( wristRollEncoder.getPosition() - targetRollSetpoint ) < atSetpointThreshold
    );
  }



  public void setMotorSpeeds(double pivot, double extension, double pitch, double roll) {
    armExtension.set(extension / 10);
    armPivot.set(pivot / 10);
    wristPitchRight.set(pitch / 10);
    wristRoll.set(roll / 10);

    PIDControl = false;
  }


  public void moveManualSetpoints(double pivot, double extension, double pitch, double roll) {
    targetPivotSetpoint += pivot / 100;
    targetExtensionSetpoint += extension / 100;
    targetPitchSetpoint += pitch / 100;
    targetRollSetpoint += roll / 100;

    manualSetpoints = true;
  }

  public void setManualSetpoints(double pivot, double extension, double pitch, double roll) {
    targetPivotSetpoint = pivot;
    targetExtensionSetpoint = extension;
    targetPitchSetpoint = pitch;
    targetRollSetpoint = roll;

    manualSetpoints = true;
  }


  public MotorPos getManualSetpoints() {
    return new MotorPos(targetExtensionSetpoint, targetPivotSetpoint, targetPitchSetpoint, targetRollSetpoint);
  }


  @Override
  public void periodic() {

    // H! The counter ensures intensive processes only run every so often
    counter += 1;
    counter %= 50;


    // H! If the limit switch is triggered, we're at min extension.
    SmartDashboard.putBoolean("limit switch pressed", extensionLimitSwitch.get());
    if (extensionLimitSwitch.get()) {
      armExtensionEncoder.setPosition(minLength);
    }

    //System.out.println(extensionLimitSwitch.get());
    

    
    
    //armExtensionEncoder.setPosition(minLength);
    
    SmartDashboard.putNumber("targetX", targetX);
    SmartDashboard.putNumber("targetY", targetY);
    SmartDashboard.putNumber("targetPitch", targetPitch);
    SmartDashboard.putNumber("targetRoll", targetRoll);
    //SmartDashboard.putNumber("###############################");

    // This method will be called once per scheduler run
    MotorPos targetPosition = IK(targetX, targetY, targetPitch, targetRoll);

    if (manualSetpoints) {
      targetPosition.pivot = targetPivotSetpoint;
      targetPosition.extension = targetExtensionSetpoint;
      targetPosition.pitch = targetPitchSetpoint;
      targetPosition.roll = targetRollSetpoint;
    }
    manualSetpoints = false;


    // ++ clamp values to be safe -------------------------------------------
    // H! Prevent arm from extending too much or too little
    targetPosition.extension = clamp(minLength, maxLength, targetPosition.extension);
    // ++ pivot
    // targetPosition.pivot = clamp(minPivotPos, maxPivotPos, targetPosition.pivot);
    // ++ pitch
    // targetPosition.pitch = clamp (minPitchPos, maxPitchPos, targetPosition.pitch);
    // ++ ----------------------

    SmartDashboard.putNumber("targetArmAngle", targetPosition.pivot);
    SmartDashboard.putNumber("targetArmLength", targetPosition.extension);
    SmartDashboard.putNumber("targetWristAngle", targetPosition.pitch);
    SmartDashboard.putNumber("targetWristRoll", targetPosition.roll);
    //Shuffleboard.put("-------------------------------");

    if (PIDControl && deleteThis_doSetpoint) {
      pidArmPivot.setReference(targetPosition.pivot, CANSparkMax.ControlType.kPosition);
      pidArmExtension.setReference(targetPosition.extension, CANSparkMax.ControlType.kPosition);
      pidWristPitch.setReference(targetPosition.pitch, CANSparkMax.ControlType.kPosition);
      pidWristRoll.setReference(targetPosition.roll, CANSparkMax.ControlType.kPosition);
    }
    PIDControl = true;
    

    SmartDashboard.putNumber("armPivot", armPivot.getOutputCurrent());    
    SmartDashboard.putNumber("armExtension", armExtension.getOutputCurrent());
    SmartDashboard.putNumber("wristPitch", wristPitchRight.getOutputCurrent());
    SmartDashboard.putNumber("wristRoll", wristRoll.getOutputCurrent());

    SmartDashboard.putNumber("armPivotLength", armPivotEncoder.getPosition());    
    SmartDashboard.putNumber("armExtensionLength", armExtensionEncoder.getPosition());
    SmartDashboard.putNumber("wristPitchLength", wristPitchEncoder.getPosition());
    SmartDashboard.putNumber("wristPitchRelativeLength", wristPitchEncoderLeft.getPosition());
    SmartDashboard.putNumber("wristPitchAbsoluteLength", wristPitchEncoderLeftAbsolute.getPosition());
    SmartDashboard.putNumber("wristRollLength", wristRollEncoder.getPosition());

    SmartDashboard.putNumber("Wrist Roll", wristRollEncoder.getSparkMAXEncoder().getPosition());
  }

  /** ++ clamps input between two extreme values
   * @param min the lower extreme value
   * @param max the upper extreme value
   * @param x the input value to be clamped
   */
  private static double clamp(double min,  double max, double x) {
    return x > max ? max : Math.max(x, min);
   }

  private static void setPIDFValues(SparkMaxPIDController pidController, double p, double i, double d, double f) {
    pidController.setP(p);
    pidController.setI(i);
    pidController.setD(d);
    pidController.setFF(f);
  }

  public static double roundAvoid(double value, int places) {
    double scale = Math.pow(10, places);
    return Math.round(value * scale) / scale;
  }
}
