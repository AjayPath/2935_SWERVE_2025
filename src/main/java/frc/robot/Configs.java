// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;

/**
 * Configuration class for MAXSwerve module SPARK MAX controllers.
 */
public final class Configs {
  
  public static final class MAXSwerveModule {
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {
      // Calculate conversion factors and feed forward gain
      double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
          / ModuleConstants.kDrivingMotorReduction;
      double turningFactor = 2 * Math.PI;
      double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

      // Driving motor configuration
      drivingConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50);
      
      drivingConfig.encoder
          .positionConversionFactor(drivingFactor)         // Meters
          .velocityConversionFactor(drivingFactor / 60.0); // Meters per second
      
      drivingConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(0.04, 0, 0) // Tune for your robot
          .velocityFF(drivingVelocityFeedForward)
          .outputRange(-1, 1);

      // Turning motor configuration
      turningConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(20);
      
      turningConfig.absoluteEncoder
          .inverted(true) // MAXSwerve output shaft rotates opposite to steering motor
          .positionConversionFactor(turningFactor)         // Radians
          .velocityConversionFactor(turningFactor / 60.0); // Radians per second
      
      turningConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          .pid(1, 0, 0) // Tune for your robot
          .outputRange(-1, 1)
          .positionWrappingEnabled(true) // Enable wrapping through 0° (e.g., 350° → 10°)
          .positionWrappingInputRange(0, turningFactor);
    }
  }
}