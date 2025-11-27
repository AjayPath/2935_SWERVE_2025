// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeSubsystem;
import frc.robot.utils.Pose;

/**
 * Auto-align to AprilTag using Team 1640's approach:
 * 1. Reset odometry so tag is at origin (0, 0)
 * 2. Use DTP to drive to alignment position
 * 3. Continuously correct odometry as we approach
 */
public class AutoAlignToTag extends Command {
  
  private final DriveSubsystem driveSubsystem;
  private final LimeSubsystem limeSubsystem;
  
  private DTP dtpCommand;
  
  private int consecutiveFailures = 0;
  private static final int MAX_CONSECUTIVE_FAILURES = 50;

  public AutoAlignToTag(DriveSubsystem driveSubsystem, LimeSubsystem limeSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.limeSubsystem = limeSubsystem;
    
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("AutoAlignToTag: Starting alignment...");
    consecutiveFailures = 0;

    // Reset odometry from tag (puts tag at origin)
    if (!limeSubsystem.resetOdometryFromTag()) {
      System.out.println("AutoAlignToTag: WARNING - No valid tag for initial reset!");
      dtpCommand = null;
      return;
    }

    // Get alignment target (offset from tag)
    Pose alignmentTarget = limeSubsystem.getAlignmentTarget();

    // Create DTP command to alignment position
    dtpCommand = new DTP(
      driveSubsystem,
      alignmentTarget.GetXValue(),
      alignmentTarget.GetYValue(),
      alignmentTarget.GetAngleValue()
    );
    
    dtpCommand.initialize();

    // Debug logging
    Pose currentPose = driveSubsystem.getCustomPose();
    System.out.println(String.format(
      "AutoAlignToTag: Current = (%.2f, %.2f, %.1f°)",
      currentPose.GetXValue(),
      currentPose.GetYValue(),
      currentPose.GetAngleValue()
    ));
    System.out.println(String.format(
      "AutoAlignToTag: Target = (%.2f, %.2f, %.1f°)",
      alignmentTarget.GetXValue(),
      alignmentTarget.GetYValue(),
      alignmentTarget.GetAngleValue()
    ));
  }

  @Override
  public void execute() {
    if (dtpCommand == null) {
      return;
    }

    // Continuously reset odometry (corrects drift!)
    boolean resetSuccess = limeSubsystem.resetOdometryFromTag();
    
    if (!resetSuccess) {
      consecutiveFailures++;
      SmartDashboard.putNumber("AutoAlign: Consecutive Failures", consecutiveFailures);
    } else {
      consecutiveFailures = 0;
    }

    // Let DTP handle the driving
    dtpCommand.execute();
    
    SmartDashboard.putBoolean("AutoAlign: Has Valid Target", limeSubsystem.isVisionConfident());
  }

  @Override
  public void end(boolean interrupted) {
    if (dtpCommand != null) {
      dtpCommand.end(interrupted);
    }
    
    if (interrupted) {
      System.out.println("AutoAlignToTag: Interrupted!");
    } else {
      System.out.println("AutoAlignToTag: Finished successfully!");
    }
  }

  @Override
  public boolean isFinished() {
    // Can't finish if we never got a target
    if (dtpCommand == null) {
      return true;
    }

    // Lost target for too long - abort
    if (consecutiveFailures > MAX_CONSECUTIVE_FAILURES) {
      System.out.println("AutoAlignToTag: Lost target for too long, aborting!");
      return true;
    }

    // Let DTP decide when we're done
    return dtpCommand.isFinished();
  }
}