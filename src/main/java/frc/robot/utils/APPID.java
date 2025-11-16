// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;

/**
 * Custom PID Controller implementation for closed-loop control.
 * 
 * PID Control: Calculates output = P + I - D
 * - P (Proportional): Responds to current error
 * - I (Integral): Accumulates past errors to eliminate steady-state error
 * - D (Derivative): Predicts future error based on rate of change
 */
public class APPID {

  // ===========================================================================================
  // PID Coefficients
  // ===========================================================================================

  private double m_p; // Proportional gain
  private double m_i; // Integral gain
  private double m_d; // Derivative gain

  // ===========================================================================================
  // Integral Control
  // ===========================================================================================

  private double m_izone;          // Integral zone - prevents windup when far from target
  private double m_errorSum;       // Accumulated error for integral term
  private double m_errorIncrement; // Maximum error added per cycle (prevents integral windup)

  // ===========================================================================================
  // Setpoint and Error Tracking
  // ===========================================================================================

  private double m_desiredValue;    // Target value (setpoint)
  private double m_oldDesiredValue; // Previous setpoint (detects changes)
  private double m_previousValue;   // Last measured value (for derivative calculation)
  private double m_errorEpsilon;    // Acceptable error range for "done" check

  // ===========================================================================================
  // Output Limiting
  // ===========================================================================================

  private double m_maxOutput; // Maximum output value (default 1.0)

  // ===========================================================================================
  // State Tracking
  // ===========================================================================================

  private boolean m_firstCycle;   // True on first calculation (skips derivative)
  private int m_cycleCount;       // Consecutive cycles within epsilon range
  private int m_minCycleCount;    // Required cycles in range before "done"
  private final Timer pidTimer;   // Tracks time between calculations

  // ===========================================================================================
  // Constructor
  // ===========================================================================================

  /**
   * Creates a new PID controller with specified gains and error tolerance.
   * 
   * @param p proportional gain
   * @param i integral gain
   * @param d derivative gain
   * @param epsilon acceptable error tolerance
   */
  public APPID(double p, double i, double d, double epsilon) {
    // PID gains
    m_p = p;
    m_i = i;
    m_d = d;
    m_errorEpsilon = epsilon;

    // Initialize default values
    m_desiredValue = 0;
    m_oldDesiredValue = 0;
    m_previousValue = 0;
    m_firstCycle = true;
    m_maxOutput = 1.0;
    m_errorSum = 0;
    m_errorIncrement = 1;
    m_izone = 0;

    // Completion tracking
    m_cycleCount = 0;
    m_minCycleCount = 10;

    // Timer for derivative calculation
    pidTimer = new Timer();
    pidTimer.start();
    pidTimer.reset();
  }

  // ===========================================================================================
  // Configuration Methods
  // ===========================================================================================

  /**
   * Updates the PID gains.
   * 
   * @param p proportional gain
   * @param i integral gain
   * @param d derivative gain
   */
  public void setConstants(double p, double i, double d) {
    m_p = p;
    m_i = i;
    m_d = d;
  }

  /**
   * Sets the integral zone range.
   * Only accumulates integral error when within this range of the target.
   * Set to 0 to disable (integral always accumulates).
   * 
   * @param izone maximum error for integral accumulation
   */
  public void setIzone(double izone) {
    m_izone = izone;
  }

  /**
   * Sets the error tolerance for "done" detection.
   * 
   * @param epsilon acceptable error range
   */
  public void setErrorEpsilon(double epsilon) {
    m_errorEpsilon = epsilon;
  }

  /**
   * Sets the maximum error increment per cycle.
   * Limits how much error can be added to integral sum each cycle (prevents windup).
   * 
   * @param inc maximum error increment
   */
  public void setErrorIncrement(double inc) {
    m_errorIncrement = inc;
  }

  /**
   * Sets the target value (setpoint).
   * 
   * @param target desired value
   */
  public void setDesiredValue(double target) {
    m_desiredValue = target;
  }

  /**
   * Sets the maximum output magnitude.
   * 
   * @param max maximum output (0.0 to 1.0)
   */
  public void setMaxOutput(double max) {
    if (max >= 0.0 && max <= 1.0) {
      m_maxOutput = max;
    }
  }

  /**
   * Sets the minimum number of consecutive cycles within epsilon before "done".
   * 
   * @param n minimum cycle count
   */
  public void setMinDoneCycles(int n) {
    m_minCycleCount = n;
  }

  // ===========================================================================================
  // PID Calculation
  // ===========================================================================================

  /**
   * Calculates the PID output based on current measured value.
   * 
   * Control Logic:
   * 1. P: Proportional to current error (error = target - current)
   * 2. I: Sum of all past errors (with limits to prevent windup)
   * 3. D: Rate of change of measurement (opposes velocity)
   * 
   * Output = P + I - D (D is subtracted to dampen oscillations)
   * 
   * @param currentValue current measured value
   * @return PID output in range [-maxOutput, maxOutput]
   */
  public double calcPID(double currentValue) {
    double pVal = 0.0;
    double iVal = 0.0;
    double dVal = 0.0;

    // First cycle: Initialize previous value, skip derivative
    if (m_firstCycle) {
      m_previousValue = currentValue;
      m_firstCycle = false;
      pidTimer.reset();
    }

    // Setpoint changed: Reset integral and state
    if (m_oldDesiredValue != m_desiredValue) {
      m_firstCycle = true;
      m_errorSum = 0;
    }

    // Calculate current error
    double error = m_desiredValue - currentValue;

    // P Term: Proportional to error
    pVal = m_p * error;

    // I Term: Accumulate error with anti-windup protection
    if (Math.abs(error) > m_errorEpsilon) {
      // Only accumulate when outside acceptable range
      if (m_izone == 0 || Math.abs(error) <= m_izone) {
        // Limit error increment to prevent windup
        if (Math.abs(error) <= m_errorIncrement) {
          m_errorSum += error;
        } else {
          m_errorSum += Math.signum(error) * m_errorIncrement;
        }
      }
    } else {
      // Within acceptable range: Reset integral
      m_errorSum = 0;
    }
    iVal = m_i * m_errorSum;

    // D Term: Rate of change (velocity) of measurement
    // Note: We use rate of measurement, not rate of error, to avoid derivative kick
    double dt = pidTimer.get();
    if (dt > 0 && !m_firstCycle) {
      double velocity = (currentValue - m_previousValue) / dt;
      dVal = m_d * velocity;
    }

    // Calculate total output: P + I - D
    // D is subtracted because it opposes velocity toward target
    double output = pVal + iVal - dVal;

    // Clamp output to maximum
    if (output > m_maxOutput) {
      output = m_maxOutput;
    } else if (output < -m_maxOutput) {
      output = -m_maxOutput;
    }

    // Update state for next cycle
    m_previousValue = currentValue;
    m_oldDesiredValue = m_desiredValue;
    pidTimer.reset();

    return output;
  }

  // ===========================================================================================
  // State Management
  // ===========================================================================================

  /**
   * Resets the accumulated integral error to zero.
   */
  public void resetErrorSum() {
    m_errorSum = 0;
  }

  /**
   * Checks if the controller has reached the target.
   * Returns true after staying within epsilon range for minimum cycle count.
   * 
   * @return true if target is reached and stable
   */
  public boolean isDone() {
    double currentError = Math.abs(m_desiredValue - m_previousValue);

    if (currentError <= m_errorEpsilon && !m_firstCycle) {
      m_cycleCount++;
      if (m_cycleCount >= m_minCycleCount) {
        m_cycleCount = 0;
        return true;
      }
    } else {
      m_cycleCount = 0; // Reset if we leave epsilon range
    }
    return false;
  }

  /**
   * Resets the PID controller to initial state.
   * Clears integral accumulation, cycle count, and state flags.
   */
  public void reset() {
    m_errorSum = 0;
    m_previousValue = 0;
    m_firstCycle = true;
    m_cycleCount = 0;
    pidTimer.reset();
  }
}