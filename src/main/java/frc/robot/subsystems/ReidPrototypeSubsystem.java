// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
<<<<<<< Updated upstream
import edu.wpi.first.wpilibj.DigitalInput;
=======
>>>>>>> Stashed changes
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
<<<<<<< Updated upstream
  CANSparkMax grabberMotor = new CANSparkMax(Constants.kMotorId, MotorType.kBrushless);
  CANSparkMax compliantMotorZero = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax compliantMotorOne = new CANSparkMax(2, MotorType.kBrushless);
  private final SparkMaxAbsoluteEncoder grabberEncoder;
  private final SparkMaxPIDController currentPIDController;

  public double currentFF = 0;
  public double currentP = 0;
  public double currentI = 0;
  public double currentD = 0;
 
  /** Creates a new ExampleSubsystem. */
  public ReidPrototypeSubsystem() {
    // ££ Creates the PID Controller from the motor and sets the initial current limit
    grabberMotor.setSmartCurrentLimit(Constants.kCurrentLimit);
    compliantMotorZero.setInverted(true);
    compliantMotorOne.setInverted(true);
    grabberMotor.setInverted(true);
    currentPIDController = grabberMotor.getPIDController();
    grabberEncoder = grabberMotor.getAbsoluteEncoder(Type.kDutyCycle);
  }

  public void spinMotor(double speed) {
    // ££ Spins the two compliant wheel motors when called
    compliantMotorZero.set(speed);
    compliantMotorOne.set(speed);
  }

  public void closeGrabber(double speed) {
    // ££ Grabs an object. Makes sure the grabber arms don't get too close together
    if (grabberEncoder.getPosition() > Constants.kPositiveEncoderRotationLimit) {
      grabberMotor.set(0);
    } else {
      grabberMotor.set(speed);
    }
  }

  public void openGrabber(double speed) {
    // ££ Releases an object. Makes sure the grabber arms don't get too far apart
    if (grabberEncoder.getPosition() < Constants.kNegativeEncoderRotationLimit) {
      grabberMotor.set(0);
    } else {
      grabberMotor.set(-1 * speed);
    }
=======
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
    setPIDValues(0, 0, 0, 0.001);
  }

  public void closeGrabber() {
    grabberMotor.set(Constants.kGrabberSpeed);
    compliantMotorZero.set(Constants.kWheelSpeed);
    compliantMotorOne.set(Constants.kWheelSpeed);
    opening = false;
    closing = true;
  }

  public void openGrabber() {
    grabberMotor.set(-Constants.kGrabberSpeed);
    opening = true;
    closing = false;
>>>>>>> Stashed changes
  }

  public void setCurrentReference(double targetAmperage) {
    // ££ Sets the target amperage value and displays that along with the motors current to Smart Dashboard when called
    currentPIDController.setReference(targetAmperage, ControlType.kCurrent);

    SmartDashboard.putNumber("Target Current", targetAmperage);
<<<<<<< Updated upstream
    SmartDashboard.putNumber("Actual Current", grabberMotor.getOutputCurrent());
  }

  public void setPIDValues(double kP, double kI, double kD, double kFF) {
    // ££ Sets the P, I, D, and FF values of the PID controller when called
=======
  }

  public void setPIDValues(double kP, double kI, double kD, double kFF) {
>>>>>>> Stashed changes
    currentPIDController.setP(kP);
    currentPIDController.setI(kI);
    currentPIDController.setD(kD);
    currentPIDController.setFF(kFF);
  }

  @Override
  public void periodic() {
<<<<<<< Updated upstream
    // This method will be called once per scheduler run
=======
    compliantMotorOne.set(0.1);
    compliantMotorZero.set(0.1);

    System.out.println("hey ");

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Actual Current", grabberMotor.getOutputCurrent());
    SmartDashboard.putNumber("Absolute Encoder", grabberEncoder.getPosition());

    if (grabberEncoder.getPosition() * Constants.kGearRatio  >= Constants.kPositiveEncoderRotationLimit && opening) {
      grabberMotor.set(0);
    }

    if (grabberEncoder.getPosition() * Constants.kGearRatio <= Constants.kNegativeEncoderRotationLimit && closing) {
      grabberMotor.set(0);
    }
>>>>>>> Stashed changes
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
