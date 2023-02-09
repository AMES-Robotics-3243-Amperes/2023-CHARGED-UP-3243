// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import static frc.robot.Constants.WristAndArm.*;

public class LegAnkleSubsystem extends SubsystemBase {
  
  private PIDController pidArmPivot = new PIDController(0.1, 0, 0);
  private PIDController pidArmExtention = new PIDController(0.1, 0, 0);
  private PIDController pidWristPitch = new PIDController(0.1, 0, 0);
  private PIDController pidWristRoll = new PIDController(0.1, 0, 0); 
  
  private CANSparkMax armPivot = new CANSparkMax(MotorIDs.armPivot, MotorType.kBrushless);
  private CANSparkMax armExtension = new CANSparkMax(MotorIDs.armExtension, MotorType.kBrushless);
  private CANSparkMax wristPitch = new CANSparkMax(MotorIDs.WristPitch, MotorType.kBrushless);
  private CANSparkMax wristRoll = new CANSparkMax(MotorIDs.WristRoll, MotorType.kBrushless);

  private SparkMaxAbsoluteEncoder armPivotEncoder = armPivot.getAbsoluteEncoder(Type.kDutyCycle);
  private SparkMaxAbsoluteEncoder armExtensionEncoder = armExtension.getAbsoluteEncoder(Type.kDutyCycle);
  private SparkMaxAbsoluteEncoder wristPitchEncoder = wristPitch.getAbsoluteEncoder(Type.kDutyCycle);
  private SparkMaxAbsoluteEncoder wristRollEncoder = wristRoll.getAbsoluteEncoder(Type.kDutyCycle);

  private double targetSpeedArmPivot = 0.0;
  private double targetSpeedArmExtension = 0.0;
  private double targetSpeedWristPitch = 0.0;
  private double targetSpeedWristRoll = 0.0;

  private double targetX = 0.0;
  private double targetY = 0.0;
  private double targetPitch = 0.0;
  private double targetRoll = 0.0;


  /** Creates a new LegAnkleSubsystem. */
  public LegAnkleSubsystem() {
    // H! TODO configure arm length encoder to return arm length, not encoder rotation
    
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
    targetX = targetX + x * Constants.WristAndArm.changeXMultiplier;
    targetY = targetY + y * Constants.WristAndArm.changeYMultiplier;
    targetPitch = targetPitch + pitch * Constants.WristAndArm.changePitchMultiplier;
    targetRoll = targetRoll + roll * Constants.WristAndArm.changeRollMultiplier;

    moveToXYTheta(targetX, targetY, targetPitch, targetRoll);
  }

  /** H! Moves the arm-wrist assembly to a given position and rotation. 
   * @param x The target distance in front of the pivot
   * @param y The target distance above the pivot
   * @param pitch The target pitch to approach at
   * @param roll The target roll to aproach at
   * @return Whether the arm is in a small range of the target
  */
  public boolean moveToXYTheta(double xIn, double yIn, double pitchIn, double rollIn) {
    // H! Prevent the arm from going places it shouldn't
    double x = clamp(maxX, minX, xIn);
    double y = clamp(maxY, minY, yIn);
    double pitch = pitchIn;
    double roll = rollIn;

    // H! Inverse kinematics: see more detailed math here: https://www.desmos.com/calculator/l89yzwijul 
    double targetArmAngle = Math.atan((y + Constants.WristAndArm.wristLength * Math.sin(pitch))  /  (x + Constants.WristAndArm.wristLength * Math.cos(pitch)));
    double targetArmLength = (y + Constants.WristAndArm.wristLength * Math.sin(pitch)) / Math.sin(targetArmAngle);
    double targetWristAngle = pitch - targetArmAngle;
    double targetWristRoll = roll;


    pidArmPivot.setSetpoint(targetArmAngle);
    pidArmExtention.setSetpoint(targetArmLength);
    pidWristPitch.setSetpoint(targetWristAngle);
    pidWristRoll.setSetpoint(targetWristRoll);

    // H! Return whether it's in the right position
    return (
      pidArmPivot.atSetpoint() &&
      pidArmExtention.atSetpoint() &&
      pidWristPitch.atSetpoint() &&
      pidWristRoll.atSetpoint()
    );

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    targetSpeedArmPivot = pidArmPivot.calculate(armPivotEncoder.getPosition());
    targetSpeedArmExtension = pidArmExtention.calculate(armExtensionEncoder.getPosition());
    targetSpeedWristPitch = pidWristPitch.calculate(wristPitchEncoder.getPosition());
    targetSpeedWristRoll = pidWristRoll.calculate(wristRollEncoder.getPosition());
    
    armPivot.set(targetSpeedArmPivot);
    armExtension.set(targetSpeedArmExtension);
    wristPitch.set(targetSpeedWristPitch);
    wristRoll.set(targetSpeedWristRoll);
  }










  private static double clamp(double min, double max, double x) {
    return x>max?max:(x<min?min:x); // H! I have written the most unreadable line of code of my entire life. Witness the result.
  }
}
