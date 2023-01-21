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

  
  private CANSparkMax armPivot = new CANSparkMax(Constants.MotorIDs.armPivot, MotorType.kBrushless);
  private CANSparkMax armExtension = new CANSparkMax(Constants.MotorIDs.armExtension, MotorType.kBrushless);

  private SparkMaxAbsoluteEncoder armPivotEncoder = armPivot.getAbsoluteEncoder(Type.kDutyCycle);
  private SparkMaxAbsoluteEncoder armExtensionEncoder = armExtension.getAbsoluteEncoder(Type.kDutyCycle);

  private double targetSpeedArmPivot = 0.0;
  private double targetSpeedArmExtension = 0.0;


  /** Creates a new LegAnkleSubsystem. */
  public LegAnkleSubsystem() {
    // H! TODO configure arm length encoder to return arm length, not encoder rotation
    
  }

  public void moveToXYTheta(double x, double y, double pitch) {
    // H! Inverse kinematics: see more detailed math here: https://www.desmos.com/calculator/l89yzwijul
    double targetArmAngle = Math.atan((y + Constants.ArmData.wristLength * Math.sin(pitch))  /  (x + Constants.ArmData.wristLength * Math.cos(pitch)));
    double targetArmLength = (y + Constants.ArmData.wristLength * Math.sin(pitch)) / Math.sin(targetArmAngle);
    double targetWristAngle = pitch - targetArmAngle;


    pidArmPivot.setSetpoint(targetArmAngle);
    pidArmExtention.setSetpoint(targetArmLength);

    // H! TODO Add the wrrist stuff. Feel free to basicly copy what I did and jsut change the names
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    targetSpeedArmPivot = pidArmPivot.calculate(armPivotEncoder.getPosition());
    targetSpeedArmExtension = pidArmExtention.calculate(armExtensionEncoder.getPosition());
  }
}
