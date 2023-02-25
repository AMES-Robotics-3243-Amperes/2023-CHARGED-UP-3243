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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import static frc.robot.Constants.WristAndArm.*;

public class LegAnkleSubsystem extends SubsystemBase {

  private boolean PIDControl = true;

  private boolean manualSetpoints = true;

  private int counter = 0;

  private DigitalInput extensionLimitSwitch = new DigitalInput(0);
  
  private SparkMaxPIDController pidArmPivot;
  private SparkMaxPIDController pidArmExtention;
  private SparkMaxPIDController pidWristPitch;
  private SparkMaxPIDController pidWristRoll;
  
  private CANSparkMax armPivot = new CANSparkMax(MotorIDs.armPivot, MotorType.kBrushless);
  private CANSparkMax armExtension = new CANSparkMax(MotorIDs.armExtension, MotorType.kBrushless);
  private CANSparkMax wristPitchRight = new CANSparkMax(MotorIDs.WristPitchRight, MotorType.kBrushless);
  private CANSparkMax wristPitchLeft = new CANSparkMax(MotorIDs.WristPitchLeft, MotorType.kBrushless);
  private CANSparkMax wristRoll = new CANSparkMax(MotorIDs.WristRoll, MotorType.kBrushless);

  private RelativeEncoder armPivotEncoder = armPivot.getEncoder(/*Type.kDutyCycle*/);
  private RelativeEncoder armExtensionEncoder = armExtension.getEncoder(/*Type.kDutyCycle*/);
  private RelativeEncoder wristPitchEncoderRight = wristPitchRight.getEncoder(/*Type.kDutyCycle*/);
  private RelativeEncoder wristPitchEncoderLeft = wristPitchLeft.getEncoder(/*Type.kDutyCycle*/);
  private SparkMaxAbsoluteEncoder wristRollEncoder = wristRoll.getAbsoluteEncoder(Type.kDutyCycle);

  private double targetX = 0.0;
  private double targetY = 1.2;
  private double targetPitch = 0.0;
  private double targetRoll = 0.0;

  private double targetPivotSetpoint = 0.25;
  private double targetExtensionSetpoint = 1.2;
  private double targetPitchSetpoint = 0.0;
  private double targetRollSetpoint = 0.0;

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
    pidArmExtention = armExtension.getPIDController();
    pidWristPitch = wristPitchRight.getPIDController();
    pidWristRoll = wristRoll.getPIDController();

    pidWristRoll.setFeedbackDevice(wristRollEncoder);

    pidArmPivot.setFeedbackDevice(armPivotEncoder);

    armExtensionEncoder.setPositionConversionFactor(extensionEncoderConversionFactor);
    //armPivotEncoder.setPositionConversionFactor(1/10);//(1 / 100) * (24/54) * (21/32) = 0.00291666666
    wristPitchEncoderRight.setPositionConversionFactor(1/60);
    wristPitchEncoderLeft.setPositionConversionFactor(1/60);
    
    
    
    wristRoll.setInverted(true);


    armExtensionEncoder.setPosition(1.2/*minLength*/);
    armPivotEncoder.setPosition(0.25);
    wristPitchEncoderLeft.setPosition(0.25);
    wristPitchEncoderRight.setPosition(0.25);

    // H! Used to reset the absolute encoder. Do not run this unless that's what you want to do
    //wristRollEncoder.setZeroOffset(0);
    //wristRollEncoder.setZeroOffset(wristRollEncoder.getPosition() - 0.5);
    wristRollEncoder.setZeroOffset(.163);
    

    //System.out.println("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
    //System.out.println("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
    //System.out.println(wristRollEncoder.getZeroOffset());
    //System.out.println("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
    //System.out.println("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");

    

    

    // H! These should really be in constants, but that's a future me problem
    setPIDFValues(pidArmExtention, PID.Extension.P, PID.Extension.I, PID.Extension.D, PID.Extension.FF); 
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
    targetRoll = Math.PI;
  }


  public void updatePIDValues() {
    setPIDFValues(pidArmExtention, 
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



  /** H! Moves the arm-wrist assembly by a given position diference 
   * @param x The diference in distance in front of the pivot
   * @param y The diference in distance above the target
   * @param pitch The diference in pitch to approach at
   * @param roll The diference in roll to aproach at
  */
  public void moveByXYTheta(double x, double y, double pitch, double roll) {
    //System.out.println(x);
    moveToXYTheta(
      targetX + (Math.abs(x) < 0.01 ? 0 : x * Constants.WristAndArm.changeXMultiplier),
      targetY + (Math.abs(y) < 0.01 ? 0 : y * Constants.WristAndArm.changeYMultiplier),
      targetPitch + (Math.abs(pitch) < 0.01 ? 0 : pitch * Constants.WristAndArm.changePitchMultiplier),
      targetRoll + (Math.abs(roll) < 0.01 ? 0 : roll * Constants.WristAndArm.changeRollMultiplier)
    );
  }

  /** H! Moves the arm-wrist assembly to a given position and rotation. 
   * @param x The target distance in front of the pivot
   * @param y The target distance above the pivot
   * @param pitch The target pitch to approach at
   * @param roll The target roll to aproach at
   * @return Whether the arm is in a small range of the target
  */
  public boolean moveToXYTheta(double xIn, double yIn, double pitchIn, double rollIn) {
    // H! Prevent the arm from going places it shouldn't with clamps
    targetX = roundAvoid(clamp(minX, maxX, xIn ), 5);
    targetY = roundAvoid(clamp(minY, maxY, yIn ), 5);
    targetPitch = pitchIn;
    targetRoll = rollIn; 

    // H! Inverse kinematics: see more detailed math here: https://www.desmos.com/calculator/l89yzwijul 
    double targetArmAngle = Math.atan2(targetY + Constants.WristAndArm.wristLength * Math.sin(targetPitch),   targetX + Constants.WristAndArm.wristLength * Math.cos(targetPitch) );

    /*if (targetY + Constants.WristAndArm.wristLength * Math.sin(targetPitch) >= 0) {
      targetArmAngle = Math.atan((targetX + Constants.WristAndArm.wristLength * Math.cos(targetPitch)) / -(targetY + Constants.WristAndArm.wristLength * Math.sin(targetPitch))) + Math.PI / 2;
    } else {
      targetArmAngle = Math.atan(-(targetY + Constants.WristAndArm.wristLength * Math.sin(targetPitch))  /  (targetX + Constants.WristAndArm.wristLength * Math.cos(targetPitch))) + 3 * Math.PI / 2;
    }*/
    
    double targetArmLength = (targetY + Constants.WristAndArm.wristLength * Math.sin(targetPitch)) / Math.sin(targetArmAngle);
    double targetWristAngle = Math.PI + targetPitch - targetArmAngle;
    double targetWristRoll = targetRoll;

    // H! Convert angles to motor rotations
    targetArmAngle /= 2 * Math.PI;
    targetWristAngle /= 2 * Math.PI;
    targetWristRoll /= 2 * Math.PI;

    // H! Return whether it's in the right position
    return (
      Math.abs( armPivotEncoder.getPosition() - targetArmAngle ) < atSetpointThreshold &&
      Math.abs( armExtensionEncoder.getPosition() - targetArmLength ) < atSetpointThreshold &&
      Math.abs( wristPitchEncoderRight.getPosition() - targetWristAngle ) < atSetpointThreshold &&
      Math.abs( wristRollEncoder.getPosition() - targetWristRoll ) < atSetpointThreshold
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


  @Override
  public void periodic() {

    // H! The counter ensures intensive processes only run every so often
    counter += 1;
    counter %= 50;


    // H! If the limit switch is triggered, we're at min extension.
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
    // H! Inverse kinematics: see more detailed math here: https://www.desmos.com/calculator/l89yzwijul \
    double targetArmAngle;

    if (targetY + Constants.WristAndArm.wristLength * Math.sin(targetPitch) >= 0) {
      targetArmAngle = Math.atan((targetX + Constants.WristAndArm.wristLength * Math.cos(targetPitch)) / -(targetY + Constants.WristAndArm.wristLength * Math.sin(targetPitch))) + Math.PI / 2;
    } else {
      targetArmAngle = Math.atan(-(targetY + Constants.WristAndArm.wristLength * Math.sin(targetPitch))  /  (targetX + Constants.WristAndArm.wristLength * Math.cos(targetPitch))) + 3 * Math.PI / 2;
    }
    
    double targetArmLength = (targetY + Constants.WristAndArm.wristLength * Math.sin(targetPitch)) / Math.sin(targetArmAngle);
    double targetWristAngle = Math.PI - targetArmAngle + targetPitch;
    double targetWristRoll = targetRoll;

    // H! Convert angles to motor rotations
    targetArmAngle /= 2 * Math.PI;
    targetWristAngle /= 2 * Math.PI;
    targetWristRoll /= 2* Math.PI;


    if (manualSetpoints) {
      targetArmAngle = targetPivotSetpoint;
      targetArmLength = targetExtensionSetpoint;
      targetWristAngle = targetPitchSetpoint;
      targetWristRoll = targetRollSetpoint;
    }

    manualSetpoints = false;


    targetWristRoll += 0.5;

    // H! Prevent arm from extending too much or too little
    targetArmLength = clamp(minLength, maxLength, targetArmLength);

    SmartDashboard.putNumber("targetArmAngle", targetArmAngle);
    SmartDashboard.putNumber("targetArmLength", targetArmLength);
    SmartDashboard.putNumber("targetWristAngle", targetWristAngle);
    SmartDashboard.putNumber("targetWristRoll", targetWristRoll);
    //Shuffleboard.put("-------------------------------");

    if (PIDControl) {
      pidArmPivot.setReference(targetArmAngle, CANSparkMax.ControlType.kPosition);
      pidArmExtention.setReference(targetArmLength, CANSparkMax.ControlType.kPosition);
      pidWristPitch.setReference(targetWristAngle, CANSparkMax.ControlType.kPosition);
      pidWristRoll.setReference(targetWristRoll, CANSparkMax.ControlType.kPosition);
    }
    PIDControl = true;
    

    SmartDashboard.putNumber("armPivot", armPivot.getOutputCurrent());    
    SmartDashboard.putNumber("armExtension", armExtension.getOutputCurrent());
    SmartDashboard.putNumber("wristPitch", wristPitchRight.getOutputCurrent());
    SmartDashboard.putNumber("wristRoll", wristRoll.getOutputCurrent());

    SmartDashboard.putNumber("armPivotLength", armPivotEncoder.getPosition());    
    SmartDashboard.putNumber("armExtensionLength", armExtensionEncoder.getPosition());
    SmartDashboard.putNumber("wristPitchLength", wristPitchEncoderRight.getPosition());
    SmartDashboard.putNumber("wristRollLength", wristRollEncoder.getPosition());
  }










  private static double clamp(double min,  double max, double x) {
    return x>max?max:(x<min?min:x); // H! I have written the most unreadable line of code of my entire life. Witness the result.
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
