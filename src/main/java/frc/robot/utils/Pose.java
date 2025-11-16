// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/**
 * Represents a 2D pose with x, y position and heading angle.
 * Stores raw angle values without automatic wrapping for consistent odometry tracking.
 */
public class Pose {
  
  private double x;
  private double y;
  private double angle;

  // ===========================================================================================
  // Constructors
  // ===========================================================================================

  /**
   * Creates a pose with specified position and angle.
   * 
   * @param _x x component in inches
   * @param _y y component in inches
   * @param _angle heading angle in degrees
   */
  public Pose(double _x, double _y, double _angle) {
    x = _x;
    y = _y;
    angle = _angle;
  }

  /**
   * Creates a pose at origin with 0 degree heading.
   */
  public Pose() {
    x = 0;
    y = 0;
    angle = 0;
  }

  /**
   * Copy constructor - creates a new pose from an existing pose.
   * 
   * @param other the pose to copy
   */
  public Pose(Pose other) {
    this.x = other.x;
    this.y = other.y;
    this.angle = other.angle;
  }

  // ===========================================================================================
  // Pose Operations
  // ===========================================================================================

  /**
   * Reflects the pose across the y-axis.
   */
  public void reflectY() {
    this.x = -this.x;
    this.angle = 180 - this.angle;
    this.angle = Calculations.NormalizeAngle(this.angle);
  }

  /**
   * Adds a vector to the pose position.
   * 
   * @param v the vector to add
   */
  public void AddVector(Vector v) {
    x += v.GetXValue();
    y += v.GetYValue();
  }

  /**
   * Subtracts a vector from the pose position.
   * 
   * @param v the vector to subtract
   */
  public void SubtractVector(Vector v) {
    x -= v.GetXValue();
    y -= v.GetYValue();
  }

  /**
   * Calculates the displacement vector from another pose to this pose.
   * 
   * @param p the pose to subtract from this pose
   * @return displacement vector
   */
  public Vector Subtract(Pose p) {
    return new Vector(x - p.GetXValue(), y - p.GetYValue());
  }

  /**
   * Transforms the pose from robot-relative to field-relative coordinates.
   * 
   * @param GyroAngle the robot's heading in degrees
   */
  public void Transform(double GyroAngle) {
    angle = angle + GyroAngle;
  }

  // ===========================================================================================
  // Getters
  // ===========================================================================================

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
   * Gets the raw angle without wrapping.
   * 
   * @return angle in degrees
   */
  public double GetAngleValue() {
    return angle;
  }

  /**
   * Gets the angle wrapped to [-180, 180] for display or comparison.
   * 
   * @return wrapped angle in degrees
   */
  public double GetWrappedAngle() {
    return wrapTo180(angle);
  }

  // ===========================================================================================
  // Setters
  // ===========================================================================================

  /**
   * Sets the x component.
   * 
   * @param _x new x component in inches
   */
  public void SetX(double _x) {
    x = _x;
  }

  /**
   * Sets the y component.
   * 
   * @param _y new y component in inches
   */
  public void SetY(double _y) {
    y = _y;
  }

  /**
   * Sets the angle without wrapping.
   * 
   * @param _angle new angle in degrees
   */
  public void SetAngle(double _angle) {
    angle = _angle;
  }

  /**
   * Sets all pose components.
   * 
   * @param _x new x component in inches
   * @param _y new y component in inches
   * @param _angle new angle in degrees
   */
  public void SetPose(double _x, double _y, double _angle) {
    x = _x;
    y = _y;
    angle = _angle;
  }

  /**
   * Sets this pose to match another pose.
   * 
   * @param other the pose to copy from
   */
  public void SetPose(Pose other) {
    x = other.GetXValue();
    y = other.GetYValue();
    angle = other.GetAngleValue();
  }

  // ===========================================================================================
  // Utility Methods
  // ===========================================================================================

  /**
   * Wraps angle to [-180, 180] range.
   * 
   * @param angle angle to wrap
   * @return wrapped angle in degrees
   */
  private double wrapTo180(double angle) {
    while (angle <= -180) angle += 360;
    while (angle > 180) angle -= 360;
    return angle;
  }

  /**
   * Prints the pose with a prefix string.
   * 
   * @param pre prefix string to display before pose values
   */
  public void Print(String pre) {
    System.out.printf("%s: x:%.2f y:%.2f a:%.2f\n", pre, x, y, angle);
  }

  /**
   * Prints the pose with a prefix string and number.
   * 
   * @param pre prefix string to display before pose values
   * @param i number to display with prefix
   */
  public void Print(String pre, int i) {
    System.out.printf("%s %d: x:%.2f y:%.2f a:%.2f\n", pre, i, x, y, angle);
  }

  /**
   * Prints the pose with only a number prefix.
   * 
   * @param i number to display before pose values
   */
  public void Print(int i) {
    System.out.printf("%d: x:%.2f y:%.2f a:%.2f\n", i, x, y, angle);
  }

  /**
   * Prints the pose without any prefix.
   */
  public void Print() {
    System.out.printf("x:%.2f y:%.2f a:%.2f\n", x, y, angle);
  }
}