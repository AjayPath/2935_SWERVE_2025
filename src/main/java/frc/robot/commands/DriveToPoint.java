// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.APPID;
import frc.robot.utils.Calculations;
import frc.robot.utils.Pose;
import frc.robot.utils.SlewRateLimiter;

/**
 * Drives robot to specific field position and orientation.
 * Uses independent X/Y/rotation PID controllers with slew rate limiting.
 * Field-relative control, snaps odometry to target on completion.
 */
public class DriveToPoint extends Command {

  private final DriveSubsystem driveSubsystem;

  // Controllers
  private final APPID xPID;
  private final APPID yPID;
  private final APPID turnPID;

  private final SlewRateLimiter xLimiter;
  private final SlewRateLimiter yLimiter;
  private final SlewRateLimiter rotLimiter;

  private final Pose targetPose;
  private final double positionTolerance; // meters
  private final double angleTolerance;    // degrees

  // PID tuning - X axis
  private static final double kXP = 0.6;
  private static final double kXI = 0.0;
  private static final double kXD = 0.05;
  private static final double kXMaxSpeed = 0.5;

  // PID tuning - Y axis
  private static final double kYP = 0.6;
  private static final double kYI = 0.0;
  private static final double kYD = 0.05;
  private static final double kYMaxSpeed = 0.5;

  // PID tuning - Rotation
  private static final double kTurnP = 0.02;
  private static final double kTurnI = 0.0;
  private static final double kTurnD = 0.0;
  private static final double kMaxRotationSpeed = 0.5;

  // Slew rate limits
  private static final double kTranslationRateLimit = 2.0;  // m/s²
  private static final double kTranslationJerkLimit = 5.0;  // m/s³
  private static final double kRotationRateLimit = 3.0;     // rad/s²
  private static final double kRotationJerkLimit = 8.0;     // rad/s³

  /**
   * Creates command to drive to target.
   * @param driveSubsystem - drive subsystem
   * @param targetX - target x (meters)
   * @param targetY - target y (meters)
   * @param targetAngle - target heading (degrees)
   * @param positionTolerance - position error tolerance (meters)
   * @param angleTolerance - angle error tolerance (degrees)
   */
  public DriveToPoint(DriveSubsystem driveSubsystem, double targetX, double targetY, 
                      double targetAngle, double positionTolerance, double angleTolerance) {
    this.driveSubsystem = driveSubsystem;
    this.targetPose = new Pose(targetX, targetY, targetAngle);
    this.positionTolerance = positionTolerance;
    this.angleTolerance = angleTolerance;

    // Initialize X PID
    this.xPID = new APPID(kXP, kXI, kXD, positionTolerance);
    this.xPID.setMaxOutput(kXMaxSpeed);

    // Initialize Y PID
    this.yPID = new APPID(kYP, kYI, kYD, positionTolerance);
    this.yPID.setMaxOutput(kYMaxSpeed);

    // Initialize rotation PID
    this.turnPID = new APPID(kTurnP, kTurnI, kTurnD, angleTolerance);
    this.turnPID.setMaxOutput(kMaxRotationSpeed);

    // Initialize slew limiters
    this.xLimiter = new SlewRateLimiter(kTranslationRateLimit, kTranslationJerkLimit);
    this.yLimiter = new SlewRateLimiter(kTranslationRateLimit, kTranslationJerkLimit);
    this.rotLimiter = new SlewRateLimiter(kRotationRateLimit, kRotationJerkLimit);

    addRequirements(driveSubsystem);
  }

  /**
   * Creates command with default tolerances (0.1m, 2°).
   */
  public DriveToPoint(DriveSubsystem driveSubsystem, double targetX, double targetY, double targetAngle) {
    this(driveSubsystem, targetX, targetY, targetAngle, 0.2, 2.0);
  }

  @Override
  public void initialize() {
    xPID.reset();
    yPID.reset();
    turnPID.reset();

    xLimiter.ResetSlewRate(0.0);
    yLimiter.ResetSlewRate(0.0);
    rotLimiter.ResetSlewRate(0.0);
  }

  @Override
  public void execute() {
    Pose currentPose = driveSubsystem.getCustomPose();

    // Translation control - independent X and Y
    double xError = targetPose.GetXValue() - currentPose.GetXValue();
    double yError = targetPose.GetYValue() - currentPose.GetYValue();

    // Drive errors to zero
    xPID.setDesiredValue(0);
    yPID.setDesiredValue(0);

    double xSpeed = -xPID.calcPID(xError);
    double ySpeed = -yPID.calcPID(yError);

    // Apply slew limiting
    double xVel = xLimiter.CalculateSlewRate(xSpeed);
    double yVel = yLimiter.CalculateSlewRate(ySpeed);

    // Rotation control with shortest path
    double currentAngle = Calculations.NormalizeAngle360(currentPose.GetAngleValue());
    double targetAngle = Calculations.NormalizeAngle360(targetPose.GetAngleValue());
    double angleError = Calculations.shortestAngularDistance(currentAngle, targetAngle);

    turnPID.setDesiredValue(0);
    double rotationSpeed = turnPID.calcPID(angleError);
    double rotationOutput = rotLimiter.CalculateSlewRate(rotationSpeed);

    // Drive robot (field-relative)
    driveSubsystem.drive(xVel, yVel, rotationOutput, true);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0.0, 0.0, 0.0, true);

  }

  @Override
  public boolean isFinished() {
    Pose currentPose = driveSubsystem.getCustomPose();

    // Check position error
    double xError = targetPose.GetXValue() - currentPose.GetXValue();
    double yError = targetPose.GetYValue() - currentPose.GetYValue();
    double distanceToTarget = Math.sqrt(xError * xError + yError * yError);
    boolean positionOnTarget = distanceToTarget <= positionTolerance;

    // Check angle error (shortest path)
    double currentAngle = Calculations.NormalizeAngle360(currentPose.GetAngleValue());
    double targetAngle = Calculations.NormalizeAngle360(targetPose.GetAngleValue());
    double angleError = Math.abs(Calculations.shortestAngularDistance(currentAngle, targetAngle));
    boolean angleOnTarget = angleError <= angleTolerance;

    return positionOnTarget && angleOnTarget;
  }

  /**
   * Gets distance to target.
   * @return distance in meters
   */
  public double getDistanceToTarget() {
    Pose currentPose = driveSubsystem.getCustomPose();
    double xError = targetPose.GetXValue() - currentPose.GetXValue();
    double yError = targetPose.GetYValue() - currentPose.GetYValue();
    return Math.sqrt(xError * xError + yError * yError);
  }

  /**
   * Gets angle error (shortest path).
   * @return error in degrees (+ = rotate CCW)
   */
  public double getAngleError() {
    Pose currentPose = driveSubsystem.getCustomPose();
    double currentAngle = Calculations.NormalizeAngle360(currentPose.GetAngleValue());
    double targetAngle = Calculations.NormalizeAngle360(targetPose.GetAngleValue());
    return Calculations.shortestAngularDistance(currentAngle, targetAngle);
  }

  public Pose getTargetPose() {
    return new Pose(targetPose);
  }
}