// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LegAnkleSubsystem extends SubsystemBase {
  
  private PIDController pidArmPivot = new PIDController(0.1, 0, 0);
  private PIDController pidArmExtention = new PIDController(0.1, 0, 0);
  private PIDController pidWristPitch = new PIDController(0.1, 0, 0);
  private PIDController pidWristRoll = new PIDController(0.1, 0, 0); 
  
  private CANSparkMax armPivot = new CANSparkMax(Constants.WristAndArm.MotorIDs.armPivot, MotorType.kBrushless);
  private CANSparkMax armExtension = new CANSparkMax(Constants.WristAndArm.MotorIDs.armExtension, MotorType.kBrushless);
  private CANSparkMax wristPitch = new CANSparkMax(Constants.WristAndArm.MotorIDs.WristPitch, MotorType.kBrushless);
  private CANSparkMax wristRoll = new CANSparkMax(Constants.WristAndArm.MotorIDs.WristRoll, MotorType.kBrushless);

  private SparkMaxAbsoluteEncoder armPivotEncoder = armPivot.getAbsoluteEncoder(Type.kDutyCycle);
  private SparkMaxAbsoluteEncoder armExtensionEncoder = armExtension.getAbsoluteEncoder(Type.kDutyCycle);
  private SparkMaxAbsoluteEncoder wristPitchEncoder = wristPitch.getAbsoluteEncoder(Type.kDutyCycle);
  private SparkMaxAbsoluteEncoder wristRollEncoder = wristRoll.getAbsoluteEncoder(Type.kDutyCycle);

  private double targetSpeedArmPivot = 0.0;
  private double targetSpeedArmExtension = 0.0;
  private double targetSpeedWristPitch = 0.0;
  private double targetSpeedWristRoll = 0.0;


  /** Creates a new LegAnkleSubsystem. */
  public LegAnkleSubsystem() {
    // H! TODO configure arm length encoder to return arm length, not encoder rotation
    
  }

  public void moveToXYTheta(double x, double y, double pitch, double roll) {
    // H! Inverse kinematics: see more detailed math here: https://www.desmos.com/calculator/l89yzwijul 
    double targetArmAngle = Math.atan((y + Constants.WristAndArm.wristLength * Math.sin(pitch))  /  (x + Constants.WristAndArm.wristLength * Math.cos(pitch)));
    double targetArmLength = (y + Constants.WristAndArm.wristLength * Math.sin(pitch)) / Math.sin(targetArmAngle);
    double targetWristAngle = pitch - targetArmAngle;
    double targetWristRoll = roll;


    pidArmPivot.setSetpoint(targetArmAngle);
    pidArmExtention.setSetpoint(targetArmLength);
    pidWristPitch.setSetpoint(targetWristAngle);
    pidWristRoll.setSetpoint(targetWristRoll);

    // H! TODO Add the wrist stuff. Feel free to basically copy what I did and just change the names


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
}
