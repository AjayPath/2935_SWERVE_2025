// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;

/**
 * Custom PID controller for closed-loop control.
 * Output = P + I - D
 */
public class APPID {

  // PID gains
  private double m_p;
  private double m_i;
  private double m_d;

  // Integral anti-windup
  private double m_izone;          // Zone where integral accumulates (0 = always)
  private double m_errorSum;       // Accumulated error
  private double m_errorIncrement; // Max error added per cycle

  // Setpoint and tracking
  private double m_desiredValue;
  private double m_oldDesiredValue;
  private double m_previousValue;
  private double m_errorEpsilon; // Acceptable error for "done"

  // Output limiting
  private double m_maxOutput;

  // State
  private boolean m_firstCycle;
  private int m_cycleCount;
  private int m_minCycleCount; // Cycles in range before "done"
  private final Timer pidTimer;

  /**
   * Creates PID controller.
   * @param p - proportional gain
   * @param i - integral gain
   * @param d - derivative gain
   * @param epsilon - acceptable error tolerance
   */
  public APPID(double p, double i, double d, double epsilon) {
    m_p = p;
    m_i = i;
    m_d = d;
    m_errorEpsilon = epsilon;

    m_desiredValue = 0;
    m_oldDesiredValue = 0;
    m_previousValue = 0;
    m_firstCycle = true;
    m_maxOutput = 1.0;
    m_errorSum = 0;
    m_errorIncrement = 1;
    m_izone = 0;

    m_cycleCount = 0;
    m_minCycleCount = 10;

    pidTimer = new Timer();
    pidTimer.start();
    pidTimer.reset();
  }

  // Updates PID gains
  public void setConstants(double p, double i, double d) {
    m_p = p;
    m_i = i;
    m_d = d;
  }

  /**
   * Sets integral zone - only accumulates within this error range.
   * @param izone - max error for accumulation (0 = always accumulate)
   */
  public void setIzone(double izone) {
    m_izone = izone;
  }

  public void setErrorEpsilon(double epsilon) {
    m_errorEpsilon = epsilon;
  }

  /**
   * Limits integral windup.
   * @param inc - max error added per cycle
   */
  public void setErrorIncrement(double inc) {
    m_errorIncrement = inc;
  }

  public void setDesiredValue(double target) {
    m_desiredValue = target;
  }

  public void setMaxOutput(double max) {
    if (max >= 0.0 && max <= 1.0) {
      m_maxOutput = max;
    }
  }

  public void setMinDoneCycles(int n) {
    m_minCycleCount = n;
  }

  /**
   * Calculates PID output.
   * @param currentValue - current measurement
   * @return output in range [-maxOutput, maxOutput]
   */
  public double calcPID(double currentValue) {
    double pVal = 0.0;
    double iVal = 0.0;
    double dVal = 0.0;

    // First cycle - initialize
    if (m_firstCycle) {
      m_previousValue = currentValue;
      m_firstCycle = false;
      pidTimer.reset();
    }

    // Setpoint changed - reset
    if (m_oldDesiredValue != m_desiredValue) {
      m_firstCycle = true;
      m_errorSum = 0;
    }

    double error = m_desiredValue - currentValue;

    // P term - proportional to error
    pVal = m_p * error;

    // I term - accumulate error with anti-windup
    if (Math.abs(error) > m_errorEpsilon) {
      if (m_izone == 0 || Math.abs(error) <= m_izone) {
        // Limit increment to prevent windup
        if (Math.abs(error) <= m_errorIncrement) {
          m_errorSum += error;
        } else {
          m_errorSum += Math.signum(error) * m_errorIncrement;
        }
      }
    } else {
      m_errorSum = 0; // Reset when within tolerance
    }
    iVal = m_i * m_errorSum;

    // D term - rate of measurement change (not error to avoid derivative kick)
    double dt = pidTimer.get();
    if (dt > 0 && !m_firstCycle) {
      double velocity = (currentValue - m_previousValue) / dt;
      dVal = m_d * velocity;
    }

    // Total output: P + I - D (D opposes velocity)
    double output = pVal + iVal - dVal;

    // Clamp output
    if (output > m_maxOutput) {
      output = m_maxOutput;
    } else if (output < -m_maxOutput) {
      output = -m_maxOutput;
    }

    // Update state
    m_previousValue = currentValue;
    m_oldDesiredValue = m_desiredValue;
    pidTimer.reset();

    return output;
  }

  public void resetErrorSum() {
    m_errorSum = 0;
  }

  /**
   * Checks if target reached.
   * @return true after staying in range for minimum cycles
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
      m_cycleCount = 0;
    }
    return false;
  }

  // Resets controller to initial state
  public void reset() {
    m_errorSum = 0;
    m_previousValue = 0;
    m_firstCycle = true;
    m_cycleCount = 0;
    pidTimer.reset();
  }
}