// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.MAXSwerveModule;

/**
 * Custom odometry implementation for swerve drive robots.
 * 
 * Tracking Method:
 * - Tracks each wheel module's position independently
 * - Calculates robot center position by averaging all wheel positions
 * - Uses continuous angle tracking (no wrapping) for accurate integration
 * - Derives velocity from position changes over time
 * 
 * Coordinate System:
 * - X: Forward/backward position (meters)
 * - Y: Left/right position (meters)
 * - Angle: Heading from gyro (degrees, continuous)
 */
public class APOdometry {

  // ===========================================================================================
  // Singleton Instance
  // ===========================================================================================

  private static APOdometry instance;

  // ===========================================================================================
  // Hardware References
  // ===========================================================================================

  private final Pigeon2 gyro;
  private final List<MAXSwerveModule> swerveMods;

  // ===========================================================================================
  // Module Tracking
  // ===========================================================================================

  private final List<Pose> modulePoses;      // Current position/angle of each wheel module
  private final List<Vector> wheelOffsets;   // Fixed offset from robot center to each wheel
  private final double[] lastWheelDistances; // Last recorded distance for delta calculation

  // ===========================================================================================
  // Robot Center Tracking
  // ===========================================================================================

  private Pose lastCenter;     // Last calculated robot center pose
  private double lastTime;     // Timestamp of last update (for velocity calculation)
  private Vector lastVelocity; // Current robot velocity vector

  // ===========================================================================================
  // Constructor
  // ===========================================================================================

  /**
   * Private constructor for singleton pattern.
   * Initializes odometry with wheel offsets based on robot geometry.
   * 
   * @param swerveMods list of swerve modules (FL, FR, BL, BR order)
   * @param gyro the robot's gyroscope
   */
  private APOdometry(List<MAXSwerveModule> swerveMods, Pigeon2 gyro) {
    this.swerveMods = swerveMods;
    this.gyro = gyro;
    this.lastWheelDistances = new double[swerveMods.size()];
    this.modulePoses = new ArrayList<>(swerveMods.size());

    // Calculate wheel offsets from robot center
    // Uses half of wheelbase (front-back) and track width (left-right)
    double a = DriveConstants.kWheelBase / 2;
    double b = DriveConstants.kTrackWidth / 2;

    wheelOffsets = List.of(
        new Vector(a, b),   // Front Left
        new Vector(a, -b),  // Front Right
        new Vector(-a, b),  // Back Left
        new Vector(-a, -b)  // Back Right
    );

    // Initialize each module pose at origin with current wheel angle
    for (int i = 0; i < swerveMods.size(); i++) {
      double initialAngle = swerveMods.get(i).getPosition().angle.getDegrees();
      modulePoses.add(new Pose(0, 0, initialAngle));
    }

    // Initialize robot center pose with continuous gyro angle
    lastCenter = new Pose(0, 0, gyro.getRotation2d().getDegrees());
    lastTime = Timer.getFPGATimestamp();
  }

  /**
   * Gets the singleton instance of APOdometry.
   * Creates new instance if it doesn't exist.
   * 
   * @param swerveMods list of swerve modules
   * @param gyro the robot's gyroscope
   * @return the APOdometry instance
   */
  public static APOdometry getInstance(List<MAXSwerveModule> swerveMods, Pigeon2 gyro) {
    if (instance == null) {
      instance = new APOdometry(swerveMods, gyro);
    }
    return instance;
  }

  // ===========================================================================================
  // Odometry Update
  // ===========================================================================================

/**
 * Updates odometry by integrating wheel movements in field-relative coordinates.
 * 
 * Process:
 * 1. For each wheel: Calculate distance traveled since last update
 * 2. Convert distance to X/Y displacement using wheel angle
 * 3. ROTATE displacement by robot heading to convert to field frame
 * 4. Update each wheel's position in field coordinates
 * 5. Calculate robot center from average of all wheels
 * 6. Calculate velocity from position change over time
 * 
 * Should be called periodically (every 20ms recommended)
 */
public void update() {
  // Get current robot heading for field-relative conversion
  double robotHeading = gyro.getRotation2d().getDegrees();
  double robotHeadingRad = Math.toRadians(robotHeading);
  
  // Update each module's position based on wheel movement
  for (int i = 0; i < swerveMods.size(); i++) {
    // Get current wheel distance and calculate change
    double currentDistance = swerveMods.get(i).getPosition().distanceMeters;
    double deltaDistance = currentDistance - lastWheelDistances[i];
    lastWheelDistances[i] = currentDistance;

    // Get wheel angle (this is in robot-relative frame)
    double wheelAngle = swerveMods.get(i).getPosition().angle.getDegrees();
    double wheelAngleRad = Math.toRadians(wheelAngle);

    // Calculate displacement in robot-relative frame
    double dx_robot = Math.cos(wheelAngleRad) * deltaDistance;
    double dy_robot = Math.sin(wheelAngleRad) * deltaDistance;

    // Convert to field-relative frame by rotating by robot heading
    double dx_field = dx_robot * Math.cos(robotHeadingRad) - dy_robot * Math.sin(robotHeadingRad);
    double dy_field = dx_robot * Math.sin(robotHeadingRad) + dy_robot * Math.cos(robotHeadingRad);

    // Update wheel pose in field coordinates
    modulePoses.get(i).SetX(modulePoses.get(i).GetXValue() + dx_field);
    modulePoses.get(i).SetY(modulePoses.get(i).GetYValue() + dy_field);
    modulePoses.get(i).SetAngle(wheelAngle);
  }

  // Calculate robot center and velocity
  Pose currentCenter = calculateCenter();
  double currentTime = Timer.getFPGATimestamp();

  double dt = currentTime - lastTime;
  if (dt > 0) {
    // Calculate displacement since last update (already in field frame)
    double dx = currentCenter.GetXValue() - lastCenter.GetXValue();
    double dy = currentCenter.GetYValue() - lastCenter.GetYValue();

    // Calculate velocity magnitude and direction
    double mag = Math.sqrt(dx * dx + dy * dy) / dt;
    Rotation2d angle = new Rotation2d(Math.atan2(dy, dx));

    lastVelocity = new Vector(mag, angle);
  }

  lastCenter = currentCenter;
  lastTime = currentTime;
}

  // ===========================================================================================
  // Position Calculation
  // ===========================================================================================

  /**
   * Calculates the robot center position from a single wheel's position.
   * 
   * Process:
   * 1. Get wheel's offset vector from robot center
   * 2. Rotate offset by current robot heading
   * 3. Subtract rotated offset from wheel position
   * 
   * @param wheelPose the wheel's current pose
   * @param wheelOffset the wheel's fixed offset from center
   * @return calculated robot center pose
   */
  public Pose getCenterFromWheel(Pose wheelPose, Vector wheelOffset) {
    double yaw = gyro.getRotation2d().getDegrees();

    // Rotate wheel offset by robot heading
    double offsetAngleRad = Math.toRadians(yaw + wheelOffset.GetAngle().getDegrees());
    double offsetX = wheelOffset.GetMag() * Math.cos(offsetAngleRad);
    double offsetY = wheelOffset.GetMag() * Math.sin(offsetAngleRad);

    // Calculate center by subtracting offset from wheel position
    Pose centerPose = new Pose(
        wheelPose.GetXValue() - offsetX,
        wheelPose.GetYValue() - offsetY,
        yaw);

    return centerPose;
  }

  /**
   * Calculates the robot center position by averaging all wheel positions.
   * Each wheel's position is converted to an estimated center position,
   * then all estimates are averaged for accuracy.
   * 
   * @return the calculated robot center pose
   */
  public Pose calculateCenter() {
    double sumX = 0;
    double sumY = 0;

    // Calculate center estimate from each wheel
    for (int i = 0; i < modulePoses.size(); i++) {
      Pose centerFromThisWheel = getCenterFromWheel(modulePoses.get(i), wheelOffsets.get(i));
      sumX += centerFromThisWheel.GetXValue();
      sumY += centerFromThisWheel.GetYValue();
    }

    // Average all estimates
    double avgX = sumX / modulePoses.size();
    double avgY = sumY / modulePoses.size();
    double avgAngle = gyro.getRotation2d().getDegrees(); // Use continuous gyro angle

    return new Pose(avgX, avgY, avgAngle);
  }

  // ===========================================================================================
  // Getters
  // ===========================================================================================

  /**
   * Gets the current robot pose with continuous angle tracking.
   * Use this for internal calculations and control loops.
   * 
   * @return pose with continuous angle (can exceed 360°)
   */
  public Pose getPoseContinuous() {
    return lastCenter;
  }

  /**
   * Gets the current robot pose with normalized angle [0, 360).
   * Use this for display and human-readable output.
   * 
   * @return pose with angle normalized to [0, 360)
   */
  public Pose getPose() {
    Pose normalizedPose = new Pose(lastCenter);
    normalizedPose.SetAngle(Calculations.NormalizeAngle360(lastCenter.GetAngleValue()));
    return normalizedPose;
  }

  /**
   * Gets the current robot velocity vector.
   * 
   * @return velocity vector (magnitude in m/s, direction in degrees)
   */
  public Vector getVelocity() {
    return lastVelocity;
  }

  /**
   * Gets a copy of all module poses.
   * 
   * @return list of module poses
   */
  public List<Pose> getModulePoses() {
    return new ArrayList<>(modulePoses);
  }

  /**
   * Gets a specific module's pose.
   * 
   * @param index module index (0-3)
   * @return the module's pose
   */
  public Pose getModulePose(int index) {
    return modulePoses.get(index);
  }

  // ===========================================================================================
  // Setters and Reset
  // ===========================================================================================

  /**
   * Sets the robot pose to a specific position.
   * Updates all wheel positions to match the new pose.
   * 
   * @param newPose the desired robot pose
   */
  public void setPose(Pose newPose) {
    for (int i = 0; i < swerveMods.size(); i++) {
      Vector offset = wheelOffsets.get(i);
      double steerAngle = swerveMods.get(i).getPosition().angle.getDegrees();

      // Set wheel position based on robot center + offset
      modulePoses.get(i).SetPose(
          offset.GetXValue() + newPose.GetXValue(),
          offset.GetYValue() + newPose.GetYValue(),
          steerAngle);

      lastWheelDistances[i] = swerveMods.get(i).getPosition().distanceMeters;
    }

    lastCenter = newPose;
    calculateCenter();
  }

  /**
   * Resets odometry to origin (0, 0) with current gyro heading.
   * Preserves wheel angles and resets distance tracking.
   */
  public void reset() {
    for (int i = 0; i < swerveMods.size(); i++) {
      Vector offset = wheelOffsets.get(i);
      double steerAngle = swerveMods.get(i).getPosition().angle.getDegrees();

      // Reset wheel to offset position from origin
      modulePoses.get(i).SetPose(
          offset.GetXValue(),
          offset.GetYValue(),
          steerAngle);

      lastWheelDistances[i] = swerveMods.get(i).getPosition().distanceMeters;
    }

    lastCenter = new Pose(0, 0, gyro.getRotation2d().getDegrees());
  }

  // ===========================================================================================
  // Dashboard Logging
  // ===========================================================================================

  /**
   * Logs all wheel positions to SmartDashboard for debugging.
   */
  public void logWheelPoses() {
    for (int i = 0; i < swerveMods.size(); i++) {
      Pose p = modulePoses.get(i);
      SmartDashboard.putNumber("Wheel " + i + " X", p.GetXValue());
      SmartDashboard.putNumber("Wheel " + i + " Y", p.GetYValue());
      SmartDashboard.putNumber("Wheel " + i + " Angle", p.GetAngleValue());
    }
  }

  /**
   * Logs robot center position and velocity to SmartDashboard.
   * Shows both normalized angle (0-360) and continuous angle for debugging.
   */
  public void logCenterPose() {
    Pose center = getPose(); // Normalized angle
    Vector robotVel = getVelocity();

    SmartDashboard.putNumber("Center X", center.GetXValue());
    SmartDashboard.putNumber("Center Y", center.GetYValue());
    SmartDashboard.putNumber("Center Angle", center.GetAngleValue());
    SmartDashboard.putNumber("Center Angle Continuous", lastCenter.GetAngleValue());

    SmartDashboard.putNumber("Speed", robotVel.GetMag());
    SmartDashboard.putNumber("Direction", robotVel.GetAngle().getDegrees());
  }
}