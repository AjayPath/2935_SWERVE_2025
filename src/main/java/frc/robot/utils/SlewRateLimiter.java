// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;

/**
 * Slew rate limiter with jerk control.
 * Limits acceleration and smoothly ramps the acceleration limit itself.
 * Prevents sudden changes that could tip robot or slip wheels.
 */
public class SlewRateLimiter {

  private double maxRateLimit;  // Max rate of change (units/sec)
  private double currRateLimit; // Current rate being applied (ramps from 0 to max)
  private double jerkLimit;     // How fast rate can increase (units/sec²)

  private double prevVal;
  private double prevTime;
  private boolean firstCall;

  private static final double RATE_DECAY_FACTOR = 0.5; // Rate decay speed

  /**
   * Creates slew rate limiter.
   * @param _rateLimit - max rate of change (units/sec)
   * @param _jerkLimit - max rate of rate change (units/sec²)
   */
  public SlewRateLimiter(double _rateLimit, double _jerkLimit) {
    maxRateLimit = _rateLimit;
    jerkLimit = _jerkLimit;
    currRateLimit = 0.0;
    prevVal = 0.0;
    prevTime = Timer.getFPGATimestamp();
    firstCall = true;
  }

  /**
   * Applies rate limiting to input.
   * @param _input - desired value
   * @return rate-limited output
   */
  public double CalculateSlewRate(double _input) {
    double currTime = Timer.getFPGATimestamp();
    double elapsedTime = currTime - prevTime;

    // Handle first call
    if (firstCall || elapsedTime < 1e-6) {
      prevTime = currTime;
      prevVal = _input;
      firstCall = false;
      return _input;
    }

    // Calculate desired change
    double desiredChange = _input - prevVal;
    double desiredRate = Math.abs(desiredChange) / elapsedTime;

    // Update current rate limit with jerk limiting
    if (desiredRate > currRateLimit) {
      // Increase rate gradually
      currRateLimit = Math.min(
          currRateLimit + jerkLimit * elapsedTime,
          maxRateLimit);
    } else {
      // Decay rate when not needed
      currRateLimit = Math.max(
          desiredRate,
          currRateLimit - jerkLimit * RATE_DECAY_FACTOR * elapsedTime);
      currRateLimit = Math.max(0.0, currRateLimit);
    }

    // Apply rate limiting
    double maxChange = currRateLimit * elapsedTime;
    double output;

    if (desiredChange > maxChange) {
      output = prevVal + maxChange;
    } else if (desiredChange < -maxChange) {
      output = prevVal - maxChange;
    } else {
      output = _input;
    }

    prevTime = currTime;
    prevVal = output;
    
    return output;
  }

  public double getCurrentRateLimit() {
    return currRateLimit;
  }

  public double getLastValue() {
    return prevVal;
  }

  /**
   * Resets limiter to specific value.
   * Use when teleop starts or need instant jump.
   * @param _input - value to reset to
   */
  public void ResetSlewRate(double _input) {
    prevVal = _input;
    prevTime = Timer.getFPGATimestamp();
    currRateLimit = 0.0;
    firstCall = true;
  }
}