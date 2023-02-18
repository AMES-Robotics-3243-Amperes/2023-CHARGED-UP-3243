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

import frc.robot.Constants;

public class ReidPrototypeSubsystem extends SubsystemBase {

  // ££ Creates all of the motor and the encoder; defines the PID controller
  CANSparkMax grabberMotor = new CANSparkMax(Constants.kGrabberMotorId, MotorType.kBrushless);
  CANSparkMax compliantMotorZero = new CANSparkMax(Constants.kCompliantMotorIdOne, MotorType.kBrushless);
  CANSparkMax compliantMotorOne = new CANSparkMax(Constants.kCompliantMotorIdTwo, MotorType.kBrushless);
  private final SparkMaxAbsoluteEncoder grabberEncoder;
  private final SparkMaxPIDController currentPIDController;
  private boolean opening = false;
  private boolean closing = false;
 
  /** Creates a new ExampleSubsystem. */
  public ReidPrototypeSubsystem() {
    // ££ Creates the PID Controller and the Absolute encoder from the motor and sets the initial current limit
    grabberMotor.setSecondaryCurrentLimit(Constants.kCurrentLimit);
    compliantMotorZero.setSecondaryCurrentLimit(Constants.kCurrentLimit);
    compliantMotorOne.setSecondaryCurrentLimit(Constants.kCurrentLimit);
    currentPIDController = grabberMotor.getPIDController();
    grabberEncoder = grabberMotor.getAbsoluteEncoder(Type.kDutyCycle);
    setPIDValues(0.02, 0, 0, 0.0075);
  }

  public void resetStateValues() {
    opening = false;
    closing = false;
    grabberMotor.set(0);
    compliantMotorOne.set(0);
    compliantMotorZero.set(0);
  }

  public void openGrabber() {
    grabberMotor.set(-Constants.kGrabberSpeed);
    opening = false;
    closing = true;
  }

  public void closeGrabber() {
    grabberMotor.set(Constants.kGrabberSpeed);
    compliantMotorZero.set(Constants.kWheelSpeed);
    compliantMotorOne.set(Constants.kWheelSpeed);
    opening = true;
    closing = false;
  }

  public void setCurrentReference(boolean openGrabber) {
    // ££ Sets the target amperage value and displays that along with the motors current to Smart Dashboard when called
    if (openGrabber) { 
      currentPIDController.setReference(Constants.ktargetAmperage, ControlType.kCurrent);
      opening = true;
      closing = false;
    } else {
      currentPIDController.setReference(-Constants.ktargetAmperage, ControlType.kCurrent);
      opening = false;
      closing = true;
    }
  }

  public void setPIDValues(double kP, double kI, double kD, double kFF) {
    currentPIDController.setP(kP);
    currentPIDController.setI(kI);
    currentPIDController.setD(kD);
    currentPIDController.setFF(kFF);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Actual Current", grabberMotor.getOutputCurrent());
    SmartDashboard.putNumber("Absolute Encoder", grabberEncoder.getPosition());

    if (grabberMotor.getOutputCurrent() > 1.0) {
      compliantMotorZero.set(0);
      compliantMotorOne.set(0);
    }
    
    if (grabberEncoder.getPosition() >= Constants.kPositiveEncoderRotationLimit && opening) {
      grabberMotor.set(0);
      currentPIDController.setReference(0, ControlType.kCurrent);
      compliantMotorZero.set(0);
      compliantMotorOne.set(0);
      opening = false;
    }

    if (grabberEncoder.getPosition() <= Constants.kNegativeEncoderRotationLimit && closing) {
      grabberMotor.set(0);
      currentPIDController.setReference(0, ControlType.kCurrent);
      compliantMotorZero.set(0);
      compliantMotorOne.set(0);
      closing = false;
    }
  }

  @Override
   public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
