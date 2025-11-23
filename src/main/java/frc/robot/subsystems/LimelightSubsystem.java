// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.fieldConstants;
import frc.robot.utils.Pose;

/**
 * Vision subsystem for AprilTag tracking and robot localization.
 * Uses Limelight to detect AprilTags and calculate robot field position.
 */
public class LimelightSubsystem extends SubsystemBase {
  
  private final NetworkTable limelight;
  private final DriveSubsystem m_drive;
  
  public LimelightSubsystem(DriveSubsystem m_drive) {
    this.m_drive = m_drive;
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Valid ID", hasValidTarget());
    
    if (hasValidTarget()) {
      double distance = getDistanceFromTag(getTA());
      
      SmartDashboard.putNumber("TAG ID", getTID());
      SmartDashboard.putNumber("TX", getTX());
      SmartDashboard.putNumber("TY", getTY());
      SmartDashboard.putNumber("TA", getTA());
      SmartDashboard.putNumber("Distance From Tag", distance);
      SmartDashboard.putNumber("Gyro Heading", m_drive.getHeading());
      
      calculateRobotFieldPose(distance);
    }
  }

  // Returns true if limelight sees a valid AprilTag
  public boolean hasValidTarget() {
    return limelight.getEntry("tv").getDouble(0) == 1.0;
  }

  // Horizontal offset from crosshair to target (degrees, -29.8 to 29.8)
  public double getTX() {
    return limelight.getEntry("tx").getDouble(0);
  }

  // Vertical offset from crosshair to target (degrees, -24.85 to 24.85)
  public double getTY() {
    return limelight.getEntry("ty").getDouble(0);
  }

  // Target area as percentage of image (0% to 100%)
  public double getTA() {
    return limelight.getEntry("ta").getDouble(0);
  }

  // AprilTag ID of the primary detected target
  public int getTID() {
    return (int)limelight.getEntry("tid").getDouble(0);
  }

  /**
   * Calculates distance from robot to AprilTag using tag area.
   * @param ta - tag area percentage (0-100)
   * @return distance in meters
   */
  public double getDistanceFromTag(double ta) {
    double scale = 2.297; // Calibration constant (m)
    double distance = Math.sqrt(scale / ta);
    return distance;
  }

  /**
   * Calculates robot's field position using AprilTag detection.
   * @param gyroHeading - robot's current heading in degrees
   * @return robot pose in field coordinates, or null if no valid target
   */
  public Pose calculateRobotFieldPose(double gyroHeading) {
    if (!hasValidTarget()) {
      return null;
    }

    int tid = getTID();
    Pose tagPose = fieldConstants.getTagPose(tid);

    if (tagPose == null) {
      return null;
    }

    SmartDashboard.putNumber("Tag Pose X", tagPose.GetXValue());
    SmartDashboard.putNumber("Tag Pose Y", tagPose.GetYValue());
    SmartDashboard.putNumber("Tag Pose Angle", tagPose.GetAngleValue());

    // Calculate robot position relative to tag
    double distance = getDistanceFromTag(getTA());
    double tagHeading = tagPose.GetAngleValue();
    double tx = getTX();
    double fieldAngleDegrees = gyroHeading - tx;
    double fieldAngleRadians = Math.toRadians(fieldAngleDegrees);

    // Convert polar coordinates to cartesian displacement
    double displacementX = distance * Math.cos(fieldAngleRadians);
    double displacementY = distance * Math.sin(fieldAngleRadians);

    SmartDashboard.putNumber("Displacement X", displacementX);
    SmartDashboard.putNumber("Displacement Y", displacementY);

    // Calculate robot position by offsetting from tag position
    double robotX = tagPose.GetXValue() - displacementX;
    double robotY = tagPose.GetYValue() - displacementY;

    Pose robotPose = new Pose(robotX, robotY, gyroHeading);

    SmartDashboard.putNumber("Robot Pose X", robotPose.GetXValue());
    SmartDashboard.putNumber("Robot Pose Y", robotPose.GetYValue());
    SmartDashboard.putNumber("Robot Pose Angle", robotPose.GetAngleValue());

    return robotPose;
  }
}