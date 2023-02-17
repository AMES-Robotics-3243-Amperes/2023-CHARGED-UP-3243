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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import static frc.robot.Constants.WristAndArm.*;

public class LegAnkleSubsystem extends SubsystemBase {
  
  private SparkMaxPIDController pidArmPivot;
  private SparkMaxPIDController pidArmExtention;
  private SparkMaxPIDController pidWristPitch;
  private SparkMaxPIDController pidWristRoll;
  
  private CANSparkMax armPivot = new CANSparkMax(MotorIDs.armPivot, MotorType.kBrushless);
  private CANSparkMax armExtension = new CANSparkMax(MotorIDs.armExtension, MotorType.kBrushless);
  private CANSparkMax wristPitch = new CANSparkMax(MotorIDs.WristPitch, MotorType.kBrushless);
  private CANSparkMax wristRoll = new CANSparkMax(MotorIDs.WristRoll, MotorType.kBrushless);

  private RelativeEncoder armPivotEncoder = armPivot.getEncoder(/*Type.kDutyCycle*/);
  private RelativeEncoder armExtensionEncoder = armExtension.getEncoder(/*Type.kDutyCycle*/);
  private SparkMaxAbsoluteEncoder wristPitchEncoder = wristPitch.getAbsoluteEncoder(Type.kDutyCycle);
  private SparkMaxAbsoluteEncoder wristRollEncoder = wristRoll.getAbsoluteEncoder(Type.kDutyCycle);

  private double targetX = 0.0;
  private double targetY = 1.1;
  private double targetPitch = 0.0;
  private double targetRoll = 0.0;

  // H! FOR TESTING PURPOSES
  //private ShuffleboardTab tab = Shuffleboard.getTab("Arm Testing");

  // H! FOR TESTING PURPOSES
  //private GenericEntry PValue = tab.add("P Value", PID.Extension.P).getEntry();
  //private GenericEntry IValue = tab.add("I Value", PID.Extension.I).getEntry();
  //private GenericEntry DValue = tab.add("D Value", PID.Extension.D).getEntry();
  //private GenericEntry FFValue = tab.add("FF Value", PID.Extension.FF).getEntry();


  /** Creates a new LegAnkleSubsystem. */
  public LegAnkleSubsystem() {
    pidArmPivot = armPivot.getPIDController();
    pidArmExtention = armExtension.getPIDController();
    pidWristPitch = wristPitch.getPIDController();
    pidWristRoll = wristRoll.getPIDController();

    pidArmPivot.setFeedbackDevice(armPivotEncoder);

    armExtensionEncoder.setPositionConversionFactor(extensionEncoderConversionFactor); // H! TODO < TRY CHANGING THIS TO 1
    //armPivotEncoder.setPositionConversionFactor(1/10);//(1 / 100) * (24/54) * (21/32) = 0.00291666666

    armExtensionEncoder.setPosition(minLength);
    armPivotEncoder.setPosition(0.25);

    

    

    // H! These should really be in constants, but that's a future me problem
    setPIDFValues(pidArmExtention, PID.Extension.P, PID.Extension.I, PID.Extension.D, PID.Extension.FF); 
    setPIDFValues(pidArmPivot,     PID.Pivot.P,     PID.Pivot.I,     PID.Pivot.D,     PID.Pivot.FF); 
    setPIDFValues(pidWristPitch,   PID.Pitch.P,     PID.Pitch.I,     PID.Pitch.D,     PID.Pitch.FF); 
    setPIDFValues(pidWristRoll,    PID.Roll.P,      PID.Roll.I,      PID.Roll.D,      PID.Roll.FF); 


    
    // H! Set soft current limits
    armPivot.setSmartCurrentLimit(pivotCurrentLimit);
    armExtension.setSmartCurrentLimit(extensionCurrentLimit);
    wristPitch.setSmartCurrentLimit(pitchCurrentLimit);
    wristRoll.setSmartCurrentLimit(rollCurrentLimit);

    // H! Set hard current limits
    armPivot.setSecondaryCurrentLimit(NEO1650CurrentLimitHard);
    armExtension.setSecondaryCurrentLimit(NEO1650CurrentLimitHard);
    wristPitch.setSecondaryCurrentLimit(NEO550CurrentLimitHard);
    wristRoll.setSecondaryCurrentLimit(NEO550CurrentLimitHard);
  }

  /** H! Moves the arm-wrist assembly by a given position diference 
   * @param x The diference in distance in front of the pivot
   * @param y The diference in distance above the target
   * @param pitch The diference in pitch to approach at
   * @param roll The diference in roll to aproach at
  */
  public void moveByXYTheta(double x, double y, double pitch, double roll) {
    moveToXYTheta(
      targetX + x * Constants.WristAndArm.changeXMultiplier,
      targetY + y * Constants.WristAndArm.changeYMultiplier,
      targetPitch + pitch * Constants.WristAndArm.changePitchMultiplier,
      targetRoll + roll * Constants.WristAndArm.changeRollMultiplier
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
    double targetArmAngle;

    if (targetY + Constants.WristAndArm.wristLength * Math.sin(targetPitch) >= 0) {
      targetArmAngle = Math.atan((targetX + Constants.WristAndArm.wristLength * Math.cos(targetPitch)) / -(targetY + Constants.WristAndArm.wristLength * Math.sin(targetPitch))) + Math.PI / 2;
    } else {
      targetArmAngle = Math.atan(-(targetY + Constants.WristAndArm.wristLength * Math.sin(targetPitch))  /  (targetX + Constants.WristAndArm.wristLength * Math.cos(targetPitch))) + 3 * Math.PI / 2;
    }
    
    double targetArmLength = (targetY + Constants.WristAndArm.wristLength * Math.sin(targetPitch)) / Math.sin(targetArmAngle);
    double targetWristAngle = targetPitch - targetArmAngle;
    double targetWristRoll = targetRoll;

    // H! Convert angles to motor rotations
    targetArmAngle /= 2 * Math.PI;

    // H! Return whether it's in the right position
    return (
      Math.abs( armPivotEncoder.getPosition() - targetArmAngle ) < atSetpointThreshold &&
      Math.abs( armExtensionEncoder.getPosition() - targetArmLength ) < atSetpointThreshold &&
      Math.abs( wristPitchEncoder.getPosition() - targetWristAngle ) < atSetpointThreshold &&
      Math.abs( wristRollEncoder.getPosition() - targetWristRoll ) < atSetpointThreshold
    );

  }

  @Override
  public void periodic() {

    /*setPIDFValues(pidArmExtention, 
      PValue.getDouble(PID.Extension.P), 
      IValue.getDouble(PID.Extension.I), 
      DValue.getDouble(PID.Extension.D), 
      FFValue.getDouble(PID.Extension.FF)
    );*/

    /*setPIDFValues(pidArmPivot, 
      PValue.getDouble(PID.Extension.P), 
      IValue.getDouble(PID.Extension.I), 
      DValue.getDouble(PID.Extension.D), 
      FFValue.getDouble(PID.Extension.FF)
    );*/
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
    double targetWristAngle = targetPitch - targetArmAngle;
    double targetWristRoll = targetRoll;

    // H! Convert angles to motor rotations
    targetArmAngle /= 2 * Math.PI;

    SmartDashboard.putNumber("targetArmAngle", targetArmAngle);
    SmartDashboard.putNumber("targetArmLength", targetArmLength);
    SmartDashboard.putNumber("targetWristAngle", targetWristAngle);
    SmartDashboard.putNumber("targetWristRoll", targetWristRoll);
    //Shuffleboard.put("-------------------------------");


    pidArmPivot.setReference(targetArmAngle, CANSparkMax.ControlType.kPosition);
    pidArmExtention.setReference(targetArmLength, CANSparkMax.ControlType.kPosition);
    pidWristPitch.setReference(targetWristAngle, CANSparkMax.ControlType.kPosition);
    pidWristRoll.setReference(targetWristRoll, CANSparkMax.ControlType.kPosition);

    SmartDashboard.putNumber("armPivot", armPivot.getOutputCurrent());    
    SmartDashboard.putNumber("armExtension", armExtension.getOutputCurrent());
    SmartDashboard.putNumber("wristPitch", wristPitch.getOutputCurrent());
    SmartDashboard.putNumber("wristRoll", wristRoll.getOutputCurrent());

    SmartDashboard.putNumber("armPivotLength", armPivotEncoder.getPosition());    
    SmartDashboard.putNumber("armExtensionLength", armExtensionEncoder.getPosition());
    SmartDashboard.putNumber("wristPitchLength", wristPitchEncoder.getPosition());
    SmartDashboard.putNumber("wristRollLength", wristRollEncoder.getPosition());
  }










  private static double clamp(double min, double max, double x) {
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
