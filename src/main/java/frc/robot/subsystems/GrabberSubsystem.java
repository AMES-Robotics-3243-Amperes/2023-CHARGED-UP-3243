// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.lang.invoke.ConstantCallSite;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import frc.robot.Constants;

public class GrabberSubsystem extends SubsystemBase {

  public static enum GrabberState {
    moving      (null),
    fullOpen    (Constants.Grabber.openGrabberToWidthSetpoint),
    partialOpen (Constants.Grabber.openGrabberSetpoint),
    close       (Constants.Grabber.closedGrabberSetpoint);

    public final Double position;

    GrabberState(Double position) {
      this.position = position;
    }
  }

  // ++ create motor objects, encoders, and PIDs

  // ++ grabberOpenerMotor opens and closes the sides of the grabber --
  // ++ grabber motor object:
  private CANSparkMax grabberOpenerMotor = new CANSparkMax(Constants.Grabber.grabberOpenerMotorID, MotorType.kBrushless);
  // ++ this encoder reads how far open the grabber is
  private SparkMaxAbsoluteEncoder grabberOpenerEncoder;
  // ++ create PID object
  private SparkMaxPIDController grabberOpenerPID;

  // ++ these motors control the spinning wheels on the sides of the grabber
  //private CANSparkMax wheelMotorOne = new CANSparkMax(Constants.Grabber.wheelMotorOneID, MotorType.kBrushless);
  //private CANSparkMax wheelMotorTwo = new CANSparkMax(Constants.Grabber.wheelMotorTwoID, MotorType.kBrushless);

  // ++ these encoders read compliant wheel motor offsets
  //private RelativeEncoder wheelMotorEncoderOne;
  //private RelativeEncoder wheelMotorEncoderTwo;

  private CANSparkMax wheelMotorOne = new CANSparkMax(Constants.Grabber.wheelMotorOneID, MotorType.kBrushless);
  private CANSparkMax wheelMotorTwo = new CANSparkMax(Constants.Grabber.wheelMotorTwoID, MotorType.kBrushless);

  // ++ create PID objects
  private SparkMaxPIDController wheelMotorOnePID;
  private SparkMaxPIDController wheelMotorTwoPID;
  
  
  private RelativeEncoder grabberRelativeEncoder;
  private RelativeEncoder compliantRelativeEncoder1;
  private RelativeEncoder compliantRelativeEncoder2;


 

  /** ++ Creates a new GrabberSubsystem. This subsystem controls the grabber, and NOT the wrist*/
  public GrabberSubsystem() {
    
    // ++ initializes opener objects
    grabberRelativeEncoder = grabberOpenerMotor.getEncoder();
    grabberOpenerEncoder = grabberOpenerMotor.getAbsoluteEncoder(Type.kDutyCycle);
    //    grabberOpenerEncoder.setPositionConversionFactor(Constants.Grabber.grabberMotorOpenerGearRatio);
    grabberOpenerPID = grabberOpenerMotor.getPIDController();
    grabberOpenerPID.setFeedbackDevice(grabberOpenerEncoder);
    
    
    grabberOpenerEncoder.setZeroOffset(0.65);
    
    // ++ initializes wheel objects
    compliantRelativeEncoder1 = wheelMotorOne.getEncoder();
    compliantRelativeEncoder2 = wheelMotorTwo.getEncoder();
    // wheelMotorEncoderOne = wheelMotorOne.getEncoder();
    // wheelMotorEncoderTwo = wheelMotorTwo.getEncoder();
    // wheelMotorEncoderOne.setVelocityConversionFactor(Constants.Grabber.wheelMotorGearRatio);
    // wheelMotorEncoderTwo.setVelocityConversionFactor(Constants.Grabber.wheelMotorGearRatio);
    wheelMotorOnePID = wheelMotorOne.getPIDController();
    wheelMotorTwoPID = wheelMotorTwo.getPIDController();
    
    // ++ set the PID values of the grabber
    setGrabberPIDValues();
    
    

    // ££ sets the current limits
    // ++ soft limits
    grabberOpenerMotor.setSmartCurrentLimit(Constants.Grabber.softOpenerMotorCurrentLimit);
    wheelMotorOne.setSmartCurrentLimit(Constants.Grabber.softWheelMotorCurrentLimit);
    wheelMotorTwo.setSmartCurrentLimit(Constants.Grabber.softWheelMotorCurrentLimit);
    // ++ hard limits
    grabberOpenerMotor.setSecondaryCurrentLimit(Constants.Grabber.hardOpenerMotorCurrentLimit);
    wheelMotorOne.setSecondaryCurrentLimit(Constants.Grabber.hardWheelMotorCurrentLimit);
    wheelMotorTwo.setSecondaryCurrentLimit(Constants.Grabber.hardWheelMotorCurrentLimit);

    grabberOpenerMotor.burnFlash();

    closeGrabber();

  }







  /** ++ sets grabber open position */
  public void setGrabberPosition (double position) {
    grabberOpenerPID.setReference(position, ControlType.kPosition);
  }

  public GrabberState getGrabberState() {
    if        (Math.abs(GrabberState.fullOpen.position    - grabberOpenerEncoder.getPosition()) < Constants.Grabber.isAtSetpointThreshhold) {
      return GrabberState.fullOpen;
    } else if (Math.abs(GrabberState.partialOpen.position - grabberOpenerEncoder.getPosition()) < Constants.Grabber.isAtSetpointThreshhold) {
      return GrabberState.partialOpen;
    } else if (Math.abs(GrabberState.close.position       - grabberOpenerEncoder.getPosition()) < Constants.Grabber.isAtSetpointThreshhold) {
      return GrabberState.close;
    } else {
      return GrabberState.moving;
    }
  }

  // public void spinCompliantWheels (double speed) {
  //   wheelMotorOne.set(speed);
  //   wheelMotorOne.set(speed);
  // }

  /** ++ opens grabber */
  public void openGrabber () {
    setGrabberPosition(Constants.Grabber.openGrabberSetpoint);
    setGrabberWheelSpeeds(0.0);
  }

  public void openGrabberToWidth () {
    setGrabberPosition(Constants.Grabber.openGrabberToWidthSetpoint);
    setGrabberWheelSpeeds(Constants.Grabber.intakeWheelSpeed);
  }

  public void ejectObject () {
    setGrabberPosition(Constants.Grabber.openGrabberToWidthSetpoint);
    setGrabberWheelSpeeds(Constants.Grabber.ejectWheelSpeed);
  }

  /** ++ closes grabber */
  public void closeGrabber () {
    setGrabberPosition(Constants.Grabber.closedGrabberSetpoint);
    setGrabberWheelSpeeds(Constants.Grabber.ambientWheelSpeed);
  }


  /** ++ sets speed of grabber intake wheels */
  public void setGrabberWheelSpeeds (double speed) {
    wheelMotorOnePID.setReference(speed, ControlType.kVelocity);
    wheelMotorTwoPID.setReference(speed, ControlType.kVelocity);
  }

  /** ++ sets grabber PID values for all motors. Should be run in grabber command init */
  public void setGrabberPIDValues(){
    grabberOpenerPID.setP(Constants.Grabber.openerMotorPGain);
    grabberOpenerPID.setI(Constants.Grabber.openerMotorIGain);
    grabberOpenerPID.setD(Constants.Grabber.openerMotorDGain);

    wheelMotorOnePID.setP(Constants.Grabber.wheelMotorPGain);
    wheelMotorOnePID.setI(Constants.Grabber.wheelMotorIGain);
    wheelMotorOnePID.setD(Constants.Grabber.wheelMotorDGain);

    wheelMotorTwoPID.setP(Constants.Grabber.wheelMotorPGain);
    wheelMotorTwoPID.setI(Constants.Grabber.wheelMotorIGain);
    wheelMotorTwoPID.setD(Constants.Grabber.wheelMotorDGain);
  }



  @Override
  public void periodic() {

    SmartDashboard.putNumber("grabber", grabberOpenerMotor.getAppliedOutput());
    SmartDashboard.putNumber("wheel1", wheelMotorOne.getAppliedOutput());
    SmartDashboard.putNumber("wheel2", wheelMotorTwo.getAppliedOutput());
    // This method will be called once per scheduler run
    
    // SmartDashboard.putNumber("Actual Current", grabberOpenerMotor.getOutputCurrent());
    SmartDashboard.putNumber("grabberEncoder", grabberOpenerEncoder.getPosition());
    

    // ++ grabber position safeties 
    if (grabberOpenerEncoder.getPosition() > Constants.Grabber.maximumGrabberLimit) {
      grabberOpenerPID.setReference ( Constants.Grabber.maximumGrabberLimit, ControlType.kPosition);
    } 
    if (grabberOpenerEncoder.getPosition() < Constants.Grabber.minimumGrabberLimit) {
      grabberOpenerPID.setReference( Constants.Grabber.minimumGrabberLimit, ControlType.kPosition);
    }

  }

  @Override
   public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}

