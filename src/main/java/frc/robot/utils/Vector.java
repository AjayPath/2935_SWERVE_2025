// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Represents a 2D vector with x, y components and polar coordinates (magnitude and angle).
 * Supports vector operations including addition, subtraction, and coordinate transformations.
 */
public class Vector {
  
  private double x = 0.0;
  private double y = 0.0;
  private double mag = 0.0;
  private Rotation2d angle = Rotation2d.fromDegrees(0);

  // ===========================================================================================
  // Constructors
  // ===========================================================================================

  /**
   * Creates a vector from Cartesian coordinates.
   * Calculates magnitude and angle from x and y components.
   * 
   * @param _x the x component (inches)
   * @param _y the y component (inches)
   */
  public Vector(double _x, double _y) {
    x = _x;
    y = _y;
    angle = new Rotation2d(Math.atan2(y, x));
    mag = Math.sqrt((Math.pow(x, 2)) + Math.pow(y, 2));
  }

  /**
   * Creates a vector from Cartesian coordinates with fallback angle.
   * Uses provided angle if both x and y are effectively zero.
   * 
   * @param _x the x component (inches)
   * @param _y the y component (inches)
   * @param _angle fallback angle if x and y are both zero
   */
  public Vector(double _x, double _y, Rotation2d _angle) {
    x = _x;
    y = _y;

    // Use provided angle if vector is at origin
    if (Math.abs(x) < 1e-9 && Math.abs(y) < 1e-9) {
      angle = _angle;
    } else {
      angle = new Rotation2d(Math.atan2(y, x));
    }
    mag = Math.sqrt((Math.pow(x, 2)) + Math.pow(y, 2));
  }

  /**
   * Creates a vector from polar coordinates.
   * Calculates x and y components from magnitude and angle.
   * 
   * @param _mag the magnitude (inches)
   * @param _angle the angle
   */
  public Vector(double _mag, Rotation2d _angle) {
    mag = _mag;
    angle = Calculations.NormalizeAngle(_angle);
    x = mag * Math.cos(angle.getRadians());
    y = mag * Math.sin(angle.getRadians());
  }

  /**
   * Default constructor for empty vector.
   */
  Vector() {}

  // ===========================================================================================
  // Vector Operations
  // ===========================================================================================

  /**
   * Subtracts a vector from this vector and returns the result.
   * 
   * @param v the vector to subtract
   * @return new vector representing the difference
   */
  public Vector SubtractVector(Vector v) {
    x -= v.GetXValue();
    y -= v.GetYValue();
    angle = new Rotation2d(Math.atan2(y, x));
    mag = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

    return new Vector(x, y);
  }

  /**
   * Adds a vector to this vector and returns the result.
   * 
   * @param v the vector to add
   * @return new vector representing the sum
   */
  public Vector AddVector(Vector v) {
    x += v.GetXValue();
    y += v.GetYValue();
    angle = new Rotation2d(Math.atan2(y, x));
    mag = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

    return new Vector(x, y);
  }

  /**
   * Transforms the vector from robot-relative to field-relative coordinates.
   * 
   * @param GyroAngle the robot's heading in degrees
   * @return transformed vector in field coordinates
   */
  public Vector Transform(double GyroAngle) {
    angle = angle.plus(Rotation2d.fromDegrees(GyroAngle));
    x = mag * Math.cos(angle.getRadians());
    y = mag * Math.sin(angle.getRadians());
    return new Vector(x, y);
  }

  /**
   * Reflects the vector across the y-axis.
   */
  public void ReflectY() {
    x = -x;
    angle = new Rotation2d(Math.atan2(Math.sin(angle.getRadians()), -Math.cos(angle.getRadians())));
  }

  /**
   * Gets the x component.
   * 
   * @return x component in inches
   */
  public double GetXValue() {
    return x;
  }

  /**
   * Gets the y component.
   * 
   * @return y component in inches
   */
  public double GetYValue() {
    return y;
  }

  /**
   * Gets the magnitude.
   * 
   * @return magnitude in inches
   */
  public double GetMag() {
    return mag;
  }

  /**
   * Gets the angle.
   * 
   * @return angle as Rotation2d
   */
  public Rotation2d GetAngle() {
    return angle;
  }

  // ===========================================================================================
  // Setters
  // ===========================================================================================

  /**
   * Sets the x component and recalculates angle and magnitude.
   * 
   * @param _x new x component in inches
   */
  public void SetX(double _x) {
    x = _x;
    angle = new Rotation2d(Math.atan2(y, x));
    mag = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
  }

  /**
   * Sets the y component and recalculates angle and magnitude.
   * 
   * @param _y new y component in inches
   */
  public void SetY(double _y) {
    y = _y;
    angle = new Rotation2d(Math.atan2(y, x));
    mag = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
  }

  /**
   * Sets the magnitude and recalculates x and y components.
   * 
   * @param _mag new magnitude in inches
   */
  public void SetMag(double _mag) {
    x = _mag * Math.cos(angle.getRadians());
    y = _mag * Math.sin(angle.getRadians());
    mag = _mag;
  }

  /**
   * Sets the angle and recalculates x and y components.
   * 
   * @param _angle new angle
   */
  public void SetAngle(Rotation2d _angle) {
    x = mag * Math.cos(_angle.getRadians());
    y = mag * Math.sin(_angle.getRadians());
    angle = _angle;
  }

  // ===========================================================================================
  // Utility Methods
  // ===========================================================================================

  /**
   * Prints the vector with a prefix string.
   * 
   * @param pre prefix string to display before vector values
   */
  public void print(String pre) {
    System.out.printf("%s: x:%.3f y:%.3f a:%.3f\n", pre, x, y, angle.getDegrees());
  }

  /**
   * Prints the vector without a prefix.
   */
  public void print() {
    System.out.printf("x:%.3f y:%.3f a:%.3f\n", x, y, angle.getDegrees());
  }
}