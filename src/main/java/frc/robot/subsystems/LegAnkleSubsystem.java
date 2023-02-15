// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
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

  private SparkMaxAbsoluteEncoder armPivotEncoder = armPivot.getAbsoluteEncoder(Type.kDutyCycle);
  private SparkMaxAbsoluteEncoder armExtensionEncoder = armExtension.getAbsoluteEncoder(Type.kDutyCycle);
  private SparkMaxAbsoluteEncoder wristPitchEncoder = wristPitch.getAbsoluteEncoder(Type.kDutyCycle);
  private SparkMaxAbsoluteEncoder wristRollEncoder = wristRoll.getAbsoluteEncoder(Type.kDutyCycle);

  private double targetX = 0.0;
  private double targetY = 0.0;
  private double targetPitch = 0.0;
  private double targetRoll = 0.0;


  /** Creates a new LegAnkleSubsystem. */
  public LegAnkleSubsystem() {
    // H! TODO configure arm length encoder to return arm length, not encoder rotation

    pidArmPivot = armPivot.getPIDController();
    pidArmExtention = armExtension.getPIDController();
    pidWristPitch = wristPitch.getPIDController();
    pidWristRoll = wristRoll.getPIDController();

    // H! These should really be in constants, but that's a future me problem
    setPIDFValues(pidArmPivot,     0, 0, 0, 0.001); 
    setPIDFValues(pidArmExtention, 0, 0, 0, 0.001); 
    setPIDFValues(pidWristPitch,   0, 0, 0, 0.001); 
    setPIDFValues(pidWristRoll,    0, 0, 0, 0.001); 


    
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
    targetX = clamp(maxX, minX, xIn);
    targetY = clamp(maxY, minY, yIn);
    targetPitch = pitchIn;
    targetRoll = rollIn;

    // H! Inverse kinematics: see more detailed math here: https://www.desmos.com/calculator/l89yzwijul 
    double targetArmAngle = Math.atan((targetY + Constants.WristAndArm.wristLength * Math.sin(targetPitch))  /  (targetX + Constants.WristAndArm.wristLength * Math.cos(targetPitch)));
    double targetArmLength = (targetY + Constants.WristAndArm.wristLength * Math.sin(targetPitch)) / Math.sin(targetArmAngle);
    double targetWristAngle = targetPitch - targetArmAngle;
    double targetWristRoll = targetRoll;

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
    // This method will be called once per scheduler run
    // H! Inverse kinematics: see more detailed math here: https://www.desmos.com/calculator/l89yzwijul 
    double targetArmAngle = Math.atan((targetY + Constants.WristAndArm.wristLength * Math.sin(targetPitch))  /  (targetX + Constants.WristAndArm.wristLength * Math.cos(targetPitch)));
    double targetArmLength = (targetY + Constants.WristAndArm.wristLength * Math.sin(targetPitch)) / Math.sin(targetArmAngle);
    double targetWristAngle = targetPitch - targetArmAngle;
    double targetWristRoll = targetRoll;


    pidArmPivot.setReference(targetArmAngle, CANSparkMax.ControlType.kPosition);
    pidArmExtention.setReference(targetArmLength, CANSparkMax.ControlType.kPosition);
    pidWristPitch.setReference(targetWristAngle, CANSparkMax.ControlType.kPosition);
    pidWristRoll.setReference(targetWristRoll, CANSparkMax.ControlType.kPosition);
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
}
