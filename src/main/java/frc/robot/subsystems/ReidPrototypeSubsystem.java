// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import static frc.robot.Constants.Grabber.*;

public class ReidPrototypeSubsystem extends SubsystemBase {

  // ££ Creates all of the motor and the encoder; defines the PID controller
  CANSparkMax grabberMotor = new CANSparkMax(kGrabberMotorId, MotorType.kBrushless);
  CANSparkMax compliantMotorZero = new CANSparkMax(kCompliantMotorIdOne, MotorType.kBrushless);
  CANSparkMax compliantMotorOne = new CANSparkMax(kCompliantMotorIdTwo, MotorType.kBrushless);
  private final SparkMaxAbsoluteEncoder grabberEncoder;
  private final SparkMaxPIDController PIDController;
  private Boolean opening = false;
  private Boolean closing = false;
 
  /** Creates a new ExampleSubsystem. */
  public ReidPrototypeSubsystem() {
    // ££ Creates the PID Controller and the Absolute encoder from the motor and sets the initial current limit
    grabberMotor.setSecondaryCurrentLimit(kCurrentLimit);
    compliantMotorZero.setSecondaryCurrentLimit(kCurrentLimit);
    compliantMotorOne.setSecondaryCurrentLimit(kCurrentLimit);
    PIDController = grabberMotor.getPIDController();
    grabberEncoder = grabberMotor.getAbsoluteEncoder(Type.kDutyCycle);
    PIDController.setFeedbackDevice(grabberEncoder);  }

  public void resetStateValues() {
    grabberMotor.set(0);
    compliantMotorOne.set(0);
    compliantMotorZero.set(0);
    opening = false;
    closing = false;
  }

  public void openGrabber() {
    setPIDValues(kPositionP, kPositionI, kPositionD, kPositionFF);
    PIDController.setReference(kNegativeEncoderRotationLimit, ControlType.kPosition);
    compliantMotorZero.set(0);
    compliantMotorOne.set(0);
    opening = true;
    closing = false;
  }

  public void closeGrabber() {
    setPIDValues(kPositionP, kPositionI, kPositionD, kPositionFF);
    PIDController.setReference(kPositiveEncoderRotationLimit, ControlType.kPosition);
    compliantMotorZero.set(kWheelSpeed);
    compliantMotorOne.set(kWheelSpeed);
    opening = false;
    closing = true;
  }

  public void setPIDValues(double kP, double kI, double kD, double kFF) {
    PIDController.setP(kP);
    PIDController.setI(kI);
    PIDController.setD(kD);
    PIDController.setFF(kFF);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Actual Current", grabberMotor.getOutputCurrent());
    SmartDashboard.putNumber("Absolute Encoder", grabberEncoder.getPosition());
    
    if (grabberEncoder.getPosition() > (kPositiveEncoderRotationLimit - 0.02) && !opening) {
      setPIDValues(kCurrentP, kCurrentI, kCurrentD, kCurrentFF);
      PIDController.setReference(kCurrentTarget, ControlType.kCurrent);
      compliantMotorZero.set(0);
      compliantMotorOne.set(0);
    }
  }

  @Override
   public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
