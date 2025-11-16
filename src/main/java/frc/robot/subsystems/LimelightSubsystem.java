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

public class LimelightSubsystem extends SubsystemBase {
  
  private final NetworkTable limelight;
  private final DriveSubsystem m_drive;


  
  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem(DriveSubsystem m_drive) {
    this.m_drive = m_drive;
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Valid ID", hasValidTarget());
    double distance = getDistanceFromTag(getTA());
    
    if (hasValidTarget()) {
      SmartDashboard.putNumber("TAG ID", getTID());
      SmartDashboard.putNumber("TX", getTX());
      SmartDashboard.putNumber("TY", getTY());
      SmartDashboard.putNumber("TA", getTA());
      SmartDashboard.putNumber("Distance From Tag", distance);
      SmartDashboard.putNumber("Gyro Heading", m_drive.getHeading());
      calculateRobotFieldPose(distance);
    }

    
  }

  public boolean hasValidTarget() {
    return limelight.getEntry("tv").getDouble(0) == 1.0;
  }

  public double getTX() {
    return limelight.getEntry("tx").getDouble(0);
  }

  public double getTY() {
    return limelight.getEntry("ty").getDouble(0);
  }

  public double getTA() {
    return limelight.getEntry("ta").getDouble(0);
  }

  public int getTID() {
    return (int)limelight.getEntry("tid").getDouble(0);
  }

  public double getDistanceFromTag(double ta) {
    double scale = 2.297; // m
    double distance = Math.sqrt(scale / ta);
    return distance;
  }

  public Pose calculateRobotFieldPose (double gyroHeading) {

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

    double distance = getDistanceFromTag(getTA());
    double tagHeading = tagPose.GetAngleValue();
    double tx = getTX();
    double fieldAngleDegrees = gyroHeading + tx;
    double fieldAngleRadians = Math.toRadians(fieldAngleDegrees);

    double displacementX = distance * Math.cos(fieldAngleRadians);
    double displacementY = distance * Math.sin(fieldAngleRadians);

    SmartDashboard.putNumber("Displacement X", displacementX);
    SmartDashboard.putNumber("Displacement Y", displacementY);

    double robotX = tagPose.GetXValue() - displacementX;
    double robotY = tagPose.GetYValue() - displacementY;



    Pose robotPose = new Pose(robotX, robotY, tagHeading);

    SmartDashboard.putNumber("Robot Pose X", robotPose.GetXValue());
    SmartDashboard.putNumber("Robot Pose Y", robotPose.GetYValue());
    SmartDashboard.putNumber("Robot Pose Angle", robotPose.GetAngleValue());

    return robotPose;

  }

}