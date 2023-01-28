// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveTrain.ModuleConstants;

public class SwerveModule {

  private final CANSparkMax m_drivingSparkMax;
  private final CANSparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkMaxPIDController m_drivingPIDController;
  private final SparkMaxPIDController m_turningPIDController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(
    0.0,
    new Rotation2d()
  );

  /**
   * <> construct a swerve module with a driving id, can id, and chassis angular
   * offset
   */
  public SwerveModule(
    int drivingCANId,
    int turningCANId,
    double chassisAngularOffset
  ) {
    // <> initilize spark maxes
    m_drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // <> factory reset spark maxes to get them to a known state
    m_drivingSparkMax.restoreFactoryDefaults();
    m_turningSparkMax.restoreFactoryDefaults();

    // <> setup encoders
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);

    // <> setup pid controllers
    m_drivingPIDController = m_drivingSparkMax.getPIDController();
    m_turningPIDController = m_turningSparkMax.getPIDController();
    m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    // <> apply position and velocity conversion factors
    m_drivingEncoder.setPositionConversionFactor(
      ModuleConstants.EncoderFactors.kDrivingEncoderPositionFactor
    );
    m_drivingEncoder.setVelocityConversionFactor(
      ModuleConstants.EncoderFactors.kDrivingEncoderVelocityFactor
    );
    m_turningEncoder.setPositionConversionFactor(
      ModuleConstants.EncoderFactors.kTurningEncoderPositionFactor
    );
    m_turningEncoder.setVelocityConversionFactor(
      ModuleConstants.EncoderFactors.kTurningEncoderVelocityFactor
    );

    // <> invert the turning encoder
    m_turningEncoder.setInverted(
      ModuleConstants.PhysicalProperties.kTurningEncoderInverted
    );

    // <> enable pid wrap on 0 to 2 pi, as the wheels rotate freely
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(
      ModuleConstants.kTurningEncoderPositionPIDMinInput
    );
    m_turningPIDController.setPositionPIDWrappingMaxInput(
      ModuleConstants.kTurningEncoderPositionPIDMaxInput
    );

    // <> set p, i, and d terms for driving
    m_drivingPIDController.setP(ModuleConstants.PIDF.kDrivingP);
    m_drivingPIDController.setI(ModuleConstants.PIDF.kDrivingI);
    m_drivingPIDController.setD(ModuleConstants.PIDF.kDrivingD);
    m_drivingPIDController.setFF(ModuleConstants.PIDF.kDrivingFF);
    m_drivingPIDController.setOutputRange(
      ModuleConstants.PIDF.kDrivingMinOutput,
      ModuleConstants.PIDF.kDrivingMaxOutput
    );

    // <> set p, i, and d terms for turning
    m_turningPIDController.setP(ModuleConstants.PIDF.kTurningP);
    m_turningPIDController.setI(ModuleConstants.PIDF.kTurningI);
    m_turningPIDController.setD(ModuleConstants.PIDF.kTurningD);
    m_turningPIDController.setFF(ModuleConstants.PIDF.kTurningFF);
    m_turningPIDController.setOutputRange(
      ModuleConstants.PIDF.kTurningMinOutput,
      ModuleConstants.PIDF.kTurningMaxOutput
    );

    // <> set idle modes and current limits
    m_drivingSparkMax.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
    m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
    m_drivingSparkMax.setSmartCurrentLimit(
      ModuleConstants.kDrivingMotorCurrentLimit
    );
    m_turningSparkMax.setSmartCurrentLimit(
      ModuleConstants.kTurningMotorCurrentLimit
    );

    // <> save configurations in case of a brown out
    m_drivingSparkMax.burnFlash();
    m_turningSparkMax.burnFlash();

    // <> initilize desired state to the current state
    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());

    // <> reset driving encoder
    m_drivingEncoder.setPosition(0);
  }

  /**
   * <>
   *
   * @return current {@link SwerveModuleState} of the module
   */
  public SwerveModuleState getState() {
    // <> return a swerve module state adjusted for the chasis angular offset
    return new SwerveModuleState(
      m_drivingEncoder.getVelocity(),
      new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset)
    );
  }

  /**
   * <>
   *
   * @return current {@link Rotation2d} of the module
   */
  public Rotation2d getRotation() {
    return getState().angle;
  }

  /**
   * <>
   *
   * @return the {@link SwerveModulePosition} of the module
   */
  public SwerveModulePosition getPosition() {
    // <> return a new swerve module position adjusted for the angular offset
    return new SwerveModulePosition(
      m_drivingEncoder.getPosition(),
      new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset)
    );
  }

  /**
   * <> sets desired state of the module
   *
   * @param desiredState desired {@link SwerveModuleState}
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean allowLowSpeedTurning) {
    // <> apply chasis angular offset
    SwerveModuleState correctedDesiredState = new SwerveModuleState();

    correctedDesiredState.speedMetersPerSecond =
      desiredState.speedMetersPerSecond;
    correctedDesiredState.angle =
      desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // <> optimize state to avoid turning more than 90 degrees
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(
      correctedDesiredState,
      new Rotation2d(m_turningEncoder.getPosition())
    );

    // <> don't worry about turning the wheel if it's spinning a tiny amount
    if (
      Math.abs(optimizedDesiredState.speedMetersPerSecond) <
      ModuleConstants.kModuleMinSpeed &&
      !allowLowSpeedTurning
    ) {
      optimizedDesiredState = new SwerveModuleState(0, getRotation());
    }

    // <> command driving
    m_drivingPIDController.setReference(
      optimizedDesiredState.speedMetersPerSecond,
      CANSparkMax.ControlType.kVelocity
    );
    m_turningPIDController.setReference(
      optimizedDesiredState.angle.getRadians(),
      CANSparkMax.ControlType.kPosition
    );

    // <> set the desired state to the calculated desired state
    m_desiredState = desiredState;
  }

  /** <> zeroes all encoders */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
}
