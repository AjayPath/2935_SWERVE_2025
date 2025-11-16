// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;

/**
 * Custom slew rate limiter with jerk control.
 * 
 * Purpose:
 * - Limits how quickly a value can change (rate limiting)
 * - Smoothly ramps the rate limit itself (jerk limiting)
 * - Prevents sudden acceleration changes that could tip the robot or strain mechanisms
 * 
 * How It Works:
 * - Rate Limit: Maximum change per second (e.g., 2.0 m/s²)
 * - Jerk Limit: How fast the rate limit can increase (e.g., 5.0 m/s³)
 * - Gradually increases acceleration from 0 to maxRateLimit
 * - Rate limit decays when input changes are small
 * 
 * Example Use Case:
 * - Driver inputs full throttle instantly
 * - Instead of instant acceleration, robot smoothly ramps up
 * - Prevents wheel slip, tipping, and mechanical stress
 */
public class SlewRateLimiter {

  // ===========================================================================================
  // Rate Limiting Parameters
  // ===========================================================================================

  private double maxRateLimit;  // Maximum rate of change (units/second)
  private double currRateLimit; // Current rate of change being applied (starts at 0, ramps to max)
  private double jerkLimit;     // How fast the rate limit can increase (units/second²)

  // ===========================================================================================
  // State Tracking
  // ===========================================================================================

  private double prevVal;    // Last output value
  private double prevTime;   // Timestamp of last calculation
  private boolean firstCall; // Track first call to handle initialization

  // Tuning parameter for rate limit decay
  private static final double RATE_DECAY_FACTOR = 0.5; // How fast unused rate limit decays

  // ===========================================================================================
  // Constructor
  // ===========================================================================================

  /**
   * Creates a slew rate limiter with jerk control.
   * 
   * @param _rateLimit maximum rate of change (units per second)
   * @param _jerkLimit maximum rate of change of rate (units per second squared)
   */
  public SlewRateLimiter(double _rateLimit, double _jerkLimit) {
    maxRateLimit = _rateLimit;
    jerkLimit = _jerkLimit;
    currRateLimit = 0.0;
    prevVal = 0.0;
    prevTime = Timer.getFPGATimestamp();
    firstCall = true;
  }

  // ===========================================================================================
  // Rate Limiting
  // ===========================================================================================

  /**
   * Applies slew rate limiting to an input value.
   * 
   * Algorithm:
   * 1. Calculate time delta (handle first call gracefully)
   * 2. Calculate desired change and required rate
   * 3. Update current rate limit with jerk limiting
   * 4. Apply rate limit to output
   * 5. Decay unused rate limit for smoother transitions
   * 
   * The rate limit itself ramps up smoothly (jerk limiting) to prevent sudden changes.
   * 
   * @param _input the desired input value
   * @return rate-limited output value
   */
  public double CalculateSlewRate(double _input) {
    double currTime = Timer.getFPGATimestamp();
    double elapsedTime = currTime - prevTime;

    // Handle first call or very small time steps
    if (firstCall || elapsedTime < 1e-6) {
      prevTime = currTime;
      prevVal = _input;
      firstCall = false;
      return _input;
    }

    // Calculate desired change
    double desiredChange = _input - prevVal;
    double desiredRate = Math.abs(desiredChange) / elapsedTime;

    // Determine if we need to increase rate limit
    if (desiredRate > currRateLimit) {
      // Gradually increase rate limit (jerk limiting)
      currRateLimit = Math.min(
          currRateLimit + jerkLimit * elapsedTime,
          maxRateLimit);
    } else {
      // Decay rate limit when not needed (allows smooth direction changes)
      // This prevents the rate limit from staying high indefinitely
      currRateLimit = Math.max(
          desiredRate,
          currRateLimit - jerkLimit * RATE_DECAY_FACTOR * elapsedTime);
      
      // Ensure rate limit doesn't go below zero
      currRateLimit = Math.max(0.0, currRateLimit);
    }

    // Apply rate limiting
    double maxChange = currRateLimit * elapsedTime;
    double output;

    if (desiredChange > maxChange) {
      // Input increasing too fast
      output = prevVal + maxChange;
    } else if (desiredChange < -maxChange) {
      // Input decreasing too fast
      output = prevVal - maxChange;
    } else {
      // Input within limits
      output = _input;
    }

    // Update state for next iteration
    prevTime = currTime;
    prevVal = output;
    
    return output;
  }

  // ===========================================================================================
  // Getters
  // ===========================================================================================

  /**
   * Gets the current rate limit being applied.
   * Useful for debugging and tuning.
   * 
   * @return current rate limit (units/second)
   */
  public double getCurrentRateLimit() {
    return currRateLimit;
  }

  /**
   * Gets the last output value.
   * 
   * @return last output value
   */
  public double getLastValue() {
    return prevVal;
  }

  // ===========================================================================================
  // Reset
  // ===========================================================================================

  /**
   * Resets the limiter to a specific value.
   * Use this when teleop starts or when you need to instantly jump to a new value.
   * 
   * @param _input the value to reset to
   */
  public void ResetSlewRate(double _input) {
    prevVal = _input;
    prevTime = Timer.getFPGATimestamp();
    currRateLimit = 0.0;
    firstCall = true;
  }
}