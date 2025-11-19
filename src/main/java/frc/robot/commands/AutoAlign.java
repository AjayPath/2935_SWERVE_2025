// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.fieldConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.utils.Pose;

/**
 * Auto-aligns robot to scoring position in front of AprilTag.
 * 
 * Strategy:
 * 1. Update odometry from vision at start
 * 2. Calculate target pose (distance from tag)
 * 3. Drive to target using odometry (DriveToPoint)
 * 4. Update odometry from vision after arrival
 */
public class AutoAlign extends Command {
  
  private final DriveSubsystem driveSubsystem;
  private final LimelightSubsystem limelightSubsystem;
  private final double scoringDistance; // How far from tag to stop (meters)
  
  private DriveToPoint driveCommand;
  private boolean hasCalculatedTarget;
  
  /**
   * Creates AutoAlign command.
   * @param drive - drive subsystem
   * @param limelight - limelight subsystem
   * @param scoringDistance - distance from tag to stop (meters)
   */
  public AutoAlign(DriveSubsystem drive, LimelightSubsystem limelight, double scoringDistance) {
    this.driveSubsystem = drive;
    this.limelightSubsystem = limelight;
    this.scoringDistance = scoringDistance;
    
    addRequirements(drive);
  }
  
  @Override
  public void initialize() {
    hasCalculatedTarget = false;
    driveCommand = null;
    
    // Initialize odometry from vision
    if (limelightSubsystem.hasValidTarget()) {
      double gyroHeading = driveSubsystem.getHeading();
      Pose visionPose = limelightSubsystem.calculateRobotFieldPose(gyroHeading);
      
      if (visionPose != null) {
        driveSubsystem.setOdom(
            visionPose.GetXValue(),
            visionPose.GetYValue(),
            //visionPose.GetAngleValue()
            gyroHeading
        );
      }
    }
  }
  
  @Override
  public void execute() {
    // Calculate target once
    if (!hasCalculatedTarget) {
      
      if (!limelightSubsystem.hasValidTarget()) {
        System.out.println("AutoAlign: No valid target");
        return;
      }
      
      // Get tag pose from field map
      int tid = limelightSubsystem.getTID();
      Pose tagPose = fieldConstants.getTagPose(tid);
      
      if (tagPose == null) {
        System.out.println("AutoAlign: Unknown tag ID " + tid);
        return;
      }
      
      // Calculate scoring pose in front of tag
      Pose targetPose = calculateScoringPose(tagPose);
      
      // Create and start DriveToPoint
      driveCommand = new DriveToPoint(
          driveSubsystem,
          targetPose.GetXValue(),
          targetPose.GetYValue(),
          targetPose.GetAngleValue()
      );
      
      driveCommand.initialize();
      hasCalculatedTarget = true;
    }
    
    // Run drive command (uses odometry)
    if (driveCommand != null) {
      driveCommand.execute();
    }
  }
  
  @Override
  public boolean isFinished() {
    return driveCommand != null && driveCommand.isFinished();
  }
  
  @Override
  public void end(boolean interrupted) {
    if (driveCommand != null) {
      driveCommand.end(interrupted);
    }
  }
  
  /**
   * Calculates target scoring pose in front of tag.
   * Robot will be 'scoringDistance' away, facing same direction as tag.
   * @param tagPose - AprilTag's field position
   * @return target pose for scoring
   */
  private Pose calculateScoringPose(Pose tagPose) {
    double tagHeading = tagPose.GetAngleValue();
    double tagHeadingRad = Math.toRadians(tagHeading);
    
    // Position 'scoringDistance' in front of tag
    // Move backwards along tag's facing direction
    double targetX = tagPose.GetXValue() - scoringDistance * Math.cos(tagHeadingRad);
    double targetY = tagPose.GetYValue() - scoringDistance * Math.sin(tagHeadingRad);
    
    // Face same direction as tag
    double targetHeading = tagHeading;
    
    return new Pose(targetX, targetY, targetHeading);
  }
}