// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.Configs;

/**
 * Single swerve module implementation for REV MAXSwerve.
 * 
 * Hardware:
 * - Drive Motor: NEO brushless motor (SparkMax)
 * - Turning Motor: NEO brushless motor (SparkMax)
 * - Turning Encoder: Through Bore absolute encoder
 * 
 * Control:
 * - Drive: Velocity control (meters per second)
 * - Turning: Position control (radians)
 */
public class MAXSwerveModule {

  // ===========================================================================================
  // Hardware Components
  // ===========================================================================================

  private final SparkMax m_drivingSpark;  // Drive motor controller
  private final SparkMax m_turningSpark;  // Turning motor controller

  private final RelativeEncoder m_drivingEncoder;   // Drive distance/velocity encoder
  private final AbsoluteEncoder m_turningEncoder;   // Absolute turning position encoder

  private final SparkClosedLoopController m_drivingClosedLoopController; // Drive PID controller
  private final SparkClosedLoopController m_turningClosedLoopController; // Turning PID controller

  // ===========================================================================================
  // Module State
  // ===========================================================================================

  private double m_chassisAngularOffset = 0; // Offset to align module zero with robot forward
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  // ===========================================================================================
  // Constructor
  // ===========================================================================================

  /**
   * Constructs a MAXSwerve module with specified CAN IDs and angular offset.
   * Applies configuration from Configs class to both SPARK controllers.
   * 
   * @param drivingCANId CAN ID for the driving motor
   * @param turningCANId CAN ID for the turning motor
   * @param chassisAngularOffset angular offset to align module zero with robot forward (radians)
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    // Initialize motor controllers
    m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

    // Get encoder references
    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

    // Get closed-loop controller references
    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    // Apply configurations from Configs class
    // Reset to safe parameters first, then persist to survive power cycles
    m_drivingSpark.configure(
        Configs.MAXSwerveModule.drivingConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_turningSpark.configure(
        Configs.MAXSwerveModule.turningConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Set angular offset and initialize desired state to current position
    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }

  // ===========================================================================================
  // State Getters
  // ===========================================================================================

  /**
   * Returns the current state of the module.
   * State includes velocity and angle relative to robot chassis.
   * 
   * @return current module state (velocity in m/s, angle in radians)
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to get angle relative to robot forward
    return new SwerveModuleState(
        m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   * Position includes distance traveled and angle relative to robot chassis.
   * 
   * @return current module position (distance in meters, angle in radians)
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to get angle relative to robot forward
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  // ===========================================================================================
  // Control
  // ===========================================================================================

  /**
   * Sets the desired state for the module.
   * 
   * Process:
   * 1. Apply chassis angular offset to desired angle
   * 2. Optimize state to avoid spinning more than 90 degrees
   * 3. Command motors to reach desired velocity and angle
   * 
   * Optimization: If the desired angle is >90° away, the wheel will reverse
   * direction and rotate to the opposite angle (<90° away) instead.
   * 
   * @param desiredState desired state with speed (m/s) and angle (radians)
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to convert from robot-relative to module-relative
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize to avoid spinning more than 90 degrees
    // May reverse wheel direction if angle is >90° away
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // Command motors to desired setpoints
    m_drivingClosedLoopController.setReference(
        correctedDesiredState.speedMetersPerSecond,
        ControlType.kVelocity);

    m_turningClosedLoopController.setReference(
        correctedDesiredState.angle.getRadians(),
        ControlType.kPosition);

    m_desiredState = desiredState;
  }

  // ===========================================================================================
  // Encoder Management
  // ===========================================================================================

  /**
   * Resets the driving encoder to zero.
   * Used to reset distance tracking (does not affect turning encoder).
   */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
}