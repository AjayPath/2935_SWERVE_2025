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
 * Strategy (following 2056):
 * 1. Use vision to measure current position and update odometry
 * 2. Calculate target scoring pose (distance from tag)
 * 3. Drive using odometry (DriveToPoint)
 * 4. Update odometry with vision after arrival
 */
public class AutoAlign extends Command {
  
  private final DriveSubsystem driveSubsystem;
  private final LimelightSubsystem limelightSubsystem;
  private final double scoringDistance;
  
  private DriveToPoint driveCommand;
  private boolean hasCalculatedTarget;
  
  /**
   * Creates an AutoAlign command.
   * 
   * @param drive the drive subsystem
   * @param limelight the limelight subsystem
   * @param scoringDistance how far from tag to stop (meters)
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
    
    // IMPORTANT: Initialize odometry from vision at start
    if (limelightSubsystem.hasValidTarget()) {
      double gyroHeading = driveSubsystem.getHeading();
      Pose visionPose = limelightSubsystem.calculateRobotFieldPose(gyroHeading);
      
      if (visionPose != null) {
        driveSubsystem.setOdom(
            visionPose.GetXValue(),
            visionPose.GetYValue(),
            visionPose.GetAngleValue()
        );
        System.out.println("AutoAlign: Initialized odometry from vision at (" +
                           visionPose.GetXValue() + ", " +
                           visionPose.GetYValue() + ", " +
                           visionPose.GetAngleValue() + "°)");
      }
    }
  }
  
  @Override
  public void execute() {
    // Only calculate target pose once at start
    if (!hasCalculatedTarget) {
      
      // Check for valid target
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
      
      // Calculate target scoring pose (in front of tag)
      Pose targetPose = calculateScoringPose(tagPose);
      
      // Create and start DriveToPoint command
      driveCommand = new DriveToPoint(
          driveSubsystem,
          targetPose.GetXValue(),
          targetPose.GetYValue(),
          targetPose.GetAngleValue()
      );
      
      driveCommand.initialize();
      hasCalculatedTarget = true;
      
      System.out.println("AutoAlign: Targeting (" + 
                         targetPose.GetXValue() + ", " + 
                         targetPose.GetYValue() + ", " + 
                         targetPose.GetAngleValue() + "°)");
    }
    
    // Run the drive command (uses odometry)
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
    
    // Update odometry from vision after arrival (2056 approach)
    if (!interrupted && limelightSubsystem.hasValidTarget()) {
      double gyroHeading = driveSubsystem.getHeading();
      Pose visionPose = limelightSubsystem.calculateRobotFieldPose(gyroHeading);
      
      if (visionPose != null) {
        driveSubsystem.setOdom(
            visionPose.GetXValue(),
            visionPose.GetYValue(),
            visionPose.GetAngleValue()
        );
        System.out.println("AutoAlign: Updated odometry from vision at end");
      }
    }
  }
  
  /**
   * Calculates target scoring pose in front of the tag.
   * Robot will be 'scoringDistance' away, facing perpendicular to tag.
   * 
   * @param tagPose the AprilTag's field position
   * @return target pose for scoring
   */
  private Pose calculateScoringPose(Pose tagPose) {
    // Get tag's heading (which way it faces)
    double tagHeading = tagPose.GetAngleValue();
    double tagHeadingRad = Math.toRadians(tagHeading);
    
    // Calculate position 'scoringDistance' in front of tag
    // Move backwards along tag's facing direction
    double targetX = tagPose.GetXValue() - scoringDistance * Math.cos(tagHeadingRad);
    double targetY = tagPose.GetYValue() - scoringDistance * Math.sin(tagHeadingRad);
    
    // Face same direction as tag (perpendicular approach)
    double targetHeading = tagHeading;
    
    return new Pose(targetX, targetY, targetHeading);
  }
}