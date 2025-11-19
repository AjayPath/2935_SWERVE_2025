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
 * Field-oriented odometry for swerve drive.
 * Tracks each wheel independently, calculates robot center by averaging.
 * All positions/velocities are in field coordinates.
 */
public class APOdometry {

  private static APOdometry instance;

  // Hardware
  private final Pigeon2 gyro;
  private final List<MAXSwerveModule> swerveMods;

  // Tracking state
  private final List<Pose> modulePoses;      // Each wheel's field position
  private final List<Vector> wheelOffsets;   // Fixed offsets from robot center
  private final double[] lastWheelDistances; // For delta calculation

  private Pose lastCenter;     // Last robot center pose
  private double lastTime;     // For velocity calculation
  private Vector lastVelocity; // Current velocity vector

  /**
   * Private constructor - use getInstance()
   * @param swerveMods - list of modules in FL, FR, BL, BR order
   * @param gyro - robot gyroscope
   */
  private APOdometry(List<MAXSwerveModule> swerveMods, Pigeon2 gyro) {
    this.swerveMods = swerveMods;
    this.gyro = gyro;
    this.lastWheelDistances = new double[swerveMods.size()];
    this.modulePoses = new ArrayList<>(swerveMods.size());

    // Calculate wheel offsets from robot center
    double a = DriveConstants.kWheelBase / 2;  // Half wheelbase
    double b = DriveConstants.kTrackWidth / 2; // Half track width

    wheelOffsets = List.of(
        new Vector(a, b),   // Front Left
        new Vector(a, -b),  // Front Right
        new Vector(-a, b),  // Back Left
        new Vector(-a, -b)  // Back Right
    );

    // Initialize module poses at origin
    for (int i = 0; i < swerveMods.size(); i++) {
      double initialAngle = swerveMods.get(i).getPosition().angle.getDegrees();
      modulePoses.add(new Pose(0, 0, initialAngle));
    }

    lastCenter = new Pose(0, 0, gyro.getRotation2d().getDegrees());
    lastTime = Timer.getFPGATimestamp();
  }

  /**
   * Gets singleton instance.
   * @return APOdometry instance
   */
  public static APOdometry getInstance(List<MAXSwerveModule> swerveMods, Pigeon2 gyro) {
    if (instance == null) {
      instance = new APOdometry(swerveMods, gyro);
    }
    return instance;
  }

  /**
   * Updates odometry from wheel movements.
   * Call this every robot loop (20ms recommended).
   * 
   * Process:
   * 1. Calculate each wheel's movement since last update
   * 2. Convert wheel movement to field coordinates using robot heading
   * 3. Update wheel positions in field frame
   * 4. Calculate robot center from wheel average
   * 5. Calculate velocity from position change
   */
  public void update() {
    double robotHeading = gyro.getRotation2d().getDegrees();
    double robotHeadingRad = Math.toRadians(robotHeading);
    
    // Update each wheel's field position
    for (int i = 0; i < swerveMods.size(); i++) {
      // Calculate distance traveled since last update
      double currentDistance = swerveMods.get(i).getPosition().distanceMeters;
      double deltaDistance = currentDistance - lastWheelDistances[i];
      lastWheelDistances[i] = currentDistance;

      // Get wheel angle (robot-relative)
      double wheelAngle = swerveMods.get(i).getPosition().angle.getDegrees();
      double wheelAngleRad = Math.toRadians(wheelAngle);

      // Calculate displacement in robot frame
      double dx_robot = Math.cos(wheelAngleRad) * deltaDistance;
      double dy_robot = Math.sin(wheelAngleRad) * deltaDistance;

      // Rotate to field frame using robot heading
      double dx_field = dx_robot * Math.cos(robotHeadingRad) - dy_robot * Math.sin(robotHeadingRad);
      double dy_field = dx_robot * Math.sin(robotHeadingRad) + dy_robot * Math.cos(robotHeadingRad);

      // Update wheel position in field coordinates
      modulePoses.get(i).SetX(modulePoses.get(i).GetXValue() + dx_field);
      modulePoses.get(i).SetY(modulePoses.get(i).GetYValue() + dy_field);
      modulePoses.get(i).SetAngle(wheelAngle);
    }

    // Calculate robot center and velocity
    Pose currentCenter = calculateCenter();
    double currentTime = Timer.getFPGATimestamp();

    double dt = currentTime - lastTime;
    if (dt > 0) {
      // Velocity from position change (already in field frame)
      double dx = currentCenter.GetXValue() - lastCenter.GetXValue();
      double dy = currentCenter.GetYValue() - lastCenter.GetYValue();

      double mag = Math.sqrt(dx * dx + dy * dy) / dt;
      Rotation2d angle = new Rotation2d(Math.atan2(dy, dx));

      lastVelocity = new Vector(mag, angle);
    }

    lastCenter = currentCenter;
    lastTime = currentTime;
  }

  /**
   * Calculates robot center from one wheel's position.
   * @param wheelPose - wheel's current pose
   * @param wheelOffset - wheel's offset from center
   * @return calculated center pose
   */
  public Pose getCenterFromWheel(Pose wheelPose, Vector wheelOffset) {
    double yaw = gyro.getRotation2d().getDegrees();

    // Rotate offset by robot heading to get field-relative offset
    double offsetAngleRad = Math.toRadians(yaw + wheelOffset.GetAngle().getDegrees());
    double offsetX = wheelOffset.GetMag() * Math.cos(offsetAngleRad);
    double offsetY = wheelOffset.GetMag() * Math.sin(offsetAngleRad);

    // Center = wheel position - offset
    Pose centerPose = new Pose(
        wheelPose.GetXValue() - offsetX,
        wheelPose.GetYValue() - offsetY,
        yaw);

    return centerPose;
  }

  /**
   * Calculates robot center by averaging all wheel positions.
   * @return robot center pose in field coordinates
   */
  public Pose calculateCenter() {
    double sumX = 0;
    double sumY = 0;

    // Get center estimate from each wheel and average
    for (int i = 0; i < modulePoses.size(); i++) {
      Pose centerFromThisWheel = getCenterFromWheel(modulePoses.get(i), wheelOffsets.get(i));
      sumX += centerFromThisWheel.GetXValue();
      sumY += centerFromThisWheel.GetYValue();
    }

    double avgX = sumX / modulePoses.size();
    double avgY = sumY / modulePoses.size();
    double avgAngle = gyro.getRotation2d().getDegrees();

    return new Pose(avgX, avgY, avgAngle);
  }

  /**
   * Gets robot pose with continuous angle (can exceed 360°).
   * Use for control calculations.
   * @return pose with continuous angle
   */
  public Pose getPoseContinuous() {
    return lastCenter;
  }

  /**
   * Gets robot pose with normalized angle [0, 360).
   * Use for display and logging.
   * @return pose with normalized angle
   */
  public Pose getPose() {
    Pose normalizedPose = new Pose(lastCenter);
    normalizedPose.SetAngle(Calculations.NormalizeAngle360(lastCenter.GetAngleValue()));
    return normalizedPose;
  }

  /**
   * Gets current velocity vector.
   * @return velocity (magnitude in m/s, direction in degrees)
   */
  public Vector getVelocity() {
    return lastVelocity;
  }

  /**
   * Gets all module poses.
   * @return list of module poses
   */
  public List<Pose> getModulePoses() {
    return new ArrayList<>(modulePoses);
  }

  /**
   * Gets specific module pose.
   * @param index - module index (0-3)
   * @return module pose
   */
  public Pose getModulePose(int index) {
    return modulePoses.get(index);
  }

  /**
   * Sets robot pose to specific position.
   * Updates all wheel positions to match.
   * @param newPose - desired robot pose
   */
  public void setPose(Pose newPose) {
    for (int i = 0; i < swerveMods.size(); i++) {
      Vector offset = wheelOffsets.get(i);
      double steerAngle = swerveMods.get(i).getPosition().angle.getDegrees();

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
   * Resets odometry to origin with current gyro heading.
   */
  public void reset() {
    for (int i = 0; i < swerveMods.size(); i++) {
      Vector offset = wheelOffsets.get(i);
      double steerAngle = swerveMods.get(i).getPosition().angle.getDegrees();

      modulePoses.get(i).SetPose(
          offset.GetXValue(),
          offset.GetYValue(),
          steerAngle);

      lastWheelDistances[i] = swerveMods.get(i).getPosition().distanceMeters;
    }

    lastCenter = new Pose(0, 0, gyro.getRotation2d().getDegrees());
  }

  // Dashboard logging
  public void logWheelPoses() {
    for (int i = 0; i < swerveMods.size(); i++) {
      Pose p = modulePoses.get(i);
      SmartDashboard.putNumber("Wheel " + i + " X", p.GetXValue());
      SmartDashboard.putNumber("Wheel " + i + " Y", p.GetYValue());
      SmartDashboard.putNumber("Wheel " + i + " Angle", p.GetAngleValue());
    }
  }

  public void logCenterPose() {
    Pose center = getPose();
    Vector robotVel = getVelocity();

    SmartDashboard.putNumber("Center X", center.GetXValue());
    SmartDashboard.putNumber("Center Y", center.GetYValue());
    SmartDashboard.putNumber("Center Angle", center.GetAngleValue());
    SmartDashboard.putNumber("Center Angle Continuous", lastCenter.GetAngleValue());
    SmartDashboard.putNumber("Speed", robotVel.GetMag());
    SmartDashboard.putNumber("Direction", robotVel.GetAngle().getDegrees());
  }
}