// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Utility class for mathematical functions used in vector math and robot calculations.
 */
public class Calculations {

  // ===========================================================================================
  // Basic Math Operations
  // ===========================================================================================

  /**
   * Squares a value while preserving its sign.
   * 
   * @param input the value to be squared
   * @return the squared value with original sign
   */
  public static double SignSquare(double input) {
    if (input < 0) {
      return -input * input;
    } else {
      return input * input;
    }
  }

  /**
   * Calculates the Euclidean distance (magnitude) of a 2D vector.
   * 
   * @param x x component
   * @param y y component
   * @return magnitude of the vector
   */
  public static double pyth(double x, double y) {
    return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
  }

  /**
   * Calculates the Euclidean distance (magnitude) of a 2D vector.
   * 
   * @param x x component
   * @param y y component
   * @return magnitude of the vector
   */
  public static double pyth(int x, int y) {
    return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
  }

  /**
   * Returns the maximum of two numbers.
   * 
   * @param num1 first number
   * @param num2 second number
   * @return the larger value
   */
  public static double Max(double num1, double num2) {
    if (num1 > num2) {
      return num1;
    } else {
      return num2;
    }
  }

  /**
   * Returns the sign of a number.
   * 
   * @param value the number to check
   * @return 1.0 if positive or zero, -1.0 if negative
   */
  public static double GetSign(double value) {
    if (value >= 0.0) {
      return 1;
    } else {
      return -1;
    }
  }

  /**
   * Limits a value to a specified range.
   * 
   * @param input the value to limit
   * @param LIMIT the absolute limit
   * @return the limited value within [-LIMIT, LIMIT]
   */
  public static double LimitOutput(double input, double LIMIT) {
    if (input > LIMIT) {
      return LIMIT;
    } else if (input < -LIMIT) {
      return -LIMIT;
    } else {
      return input;
    }
  }

  // ===========================================================================================
  // Angle Normalization
  // ===========================================================================================

  /**
   * Normalizes an angle to the range (-180, 180].
   * 
   * @param input the angle to normalize (Rotation2d)
   * @return normalized angle
   */
  public static Rotation2d NormalizeAngle(Rotation2d input) {
    while (input.getDegrees() < -180) {
      input = input.plus(Rotation2d.fromDegrees(360));
    }
    while (input.getDegrees() > 180) {
      input = input.minus(Rotation2d.fromDegrees(360));
    }
    return input;
  }

  /**
   * Normalizes an angle to the range (-180, 180].
   * 
   * @param angleDeg the angle in degrees
   * @return normalized angle in degrees
   */
  public static double NormalizeAngle(double angleDeg) {
    double result = angleDeg % 360.0;
    if (result <= -180.0) {
      result += 360.0;
    } else if (result > 180.0) {
      result -= 360.0;
    }
    return result;
  }

  /**
   * Normalizes an angle to the range [0, 360).
   * 
   * @param angleDeg the angle in degrees
   * @return normalized angle in degrees between 0 and 360
   */
  public static double NormalizeAngle360(double angleDeg) {
    double result = angleDeg % 360.0;
    if (result < 0.0) {
      result += 360.0;
    }
    return result;
  }

  // ===========================================================================================
  // Angle Difference Calculations
  // ===========================================================================================

  /**
   * Calculates the signed difference between two angles.
   * Returns the shortest angular path from current to target.
   * 
   * @param targetDegrees target angle in degrees
   * @param currentDegrees current angle in degrees
   * @return signed angular difference in range [-180, 180]
   */
  public static double signedAngleDifference(double targetDegrees, double currentDegrees) {
    double diff = targetDegrees - currentDegrees;

    // Wrap to [-180, 180]
    while (diff > 180) diff -= 360;
    while (diff <= -180) diff += 360;

    return diff;
  }

  /**
   * Calculates the shortest angular distance between two angles.
   * Normalizes both angles to [0, 360) before calculating.
   * 
   * @param targetDegrees target angle (0-360)
   * @param currentDegrees current angle (0-360)
   * @return shortest angular distance in range [-180, 180], positive = CCW, negative = CW
   */
  public static double shortestAngularDistance(double targetDegrees, double currentDegrees) {
    // Normalize both angles to [0, 360)
    double target = NormalizeAngle360(targetDegrees);
    double current = NormalizeAngle360(currentDegrees);

    double diff = target - current;

    // Convert to shortest path
    if (diff > 180.0) {
      diff -= 360.0;
    } else if (diff < -180.0) {
      diff += 360.0;
    }

    return diff;
  }
}