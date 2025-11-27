// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.APPID;
import frc.robot.utils.Calculations;
import frc.robot.utils.Pose;
import frc.robot.utils.SlewRateLimiter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DTP extends Command {
  /** Creates a new DTP. */

  private final DriveSubsystem driveSubsystem;

  private final APPID drivePID;
  private final APPID turnPID;

  private final SlewRateLimiter driveLimiter;
  private final SlewRateLimiter turnLimiter;

  private final Pose targetPose;
  private final double positionTolerance;
  private final double angleTolerance;

  private static final double kDriveP = 0.6;
  private static final double kDriveI = 0;
  private static final double kDriveD = 0;
  private static final double kMaxDriveSpeed = 0.325;

  private static final double kTurnP = 0.02;
  private static final double kTurnI = 0;
  private static final double kTurnD = 0;
  private static final double kMaxTurnSpeed = 0.125;

  private static final double kTranslationRateLimit = 5;
  private static final double kRotationRateLimit = 3;
  private static final double kTranslationJerkLimit = 5;
  private static final double kRotationJerkLimit = 5;

  public DTP(
    DriveSubsystem driveSubsystem,
    double targetX,
    double targetY,
    double targetAngle,
    double positionTolerance,
    double angleTolerance
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    this.targetPose = new Pose(targetX, targetY, targetAngle);
    this.positionTolerance = positionTolerance;
    this.angleTolerance = angleTolerance;

    this.drivePID = new APPID(kDriveP, kDriveI, kDriveD, positionTolerance);
    this.drivePID.setMaxOutput(kMaxDriveSpeed);

    this.turnPID = new APPID(kTurnP, kTurnI, kTurnD, angleTolerance);
    this.turnPID.setMaxOutput(kMaxTurnSpeed);

    this.driveLimiter = new SlewRateLimiter(kTranslationRateLimit, kTranslationJerkLimit);
    this.turnLimiter = new SlewRateLimiter(kRotationRateLimit, kRotationJerkLimit);

    addRequirements(driveSubsystem);
  }

  public DTP(
    DriveSubsystem driveSubsystem,
    double targetX,
    double targetY,
    double targetAngle
  ) {
    this(driveSubsystem, targetX, targetY, targetAngle, 0.02, 2.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivePID.reset();
    turnPID.reset();
    driveLimiter.ResetSlewRate(0.0);
    turnLimiter.ResetSlewRate(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Pose currentPose = driveSubsystem.getCustomPose();

    double xError = targetPose.GetXValue() - currentPose.GetXValue();
    double yError = targetPose.GetYValue() - currentPose.GetYValue();

    double distanceToTarget = Math.sqrt(xError * xError + yError * yError);
    double angleToTarget = Math.toDegrees(Math.atan2(yError, xError));

    drivePID.setDesiredValue(0);
    double driveSpeed = -drivePID.calcPID(distanceToTarget);
    double limitDriveSpeed = driveLimiter.CalculateSlewRate(driveSpeed);

    double angleToTargetRad = Math.toRadians(angleToTarget);
    double xVel = limitDriveSpeed * Math.cos(angleToTargetRad);
    double yVel = limitDriveSpeed * Math.sin(angleToTargetRad);

    //-----------------------------

    double currentAngle = Calculations.NormalizeAngle360(currentPose.GetAngleValue());
    double targetAngle = Calculations.NormalizeAngle360(targetPose.GetAngleValue());
    double angleError = Calculations.shortestAngularDistance(currentAngle, targetAngle);

    turnPID.setDesiredValue(0);
    double rotationSpeed = turnPID.calcPID(angleError);
    double rotationOuput = turnLimiter.CalculateSlewRate(rotationSpeed);

    driveSubsystem.drive(xVel, yVel, rotationOuput, true);

    SmartDashboard.putNumber("X", currentPose.GetXValue());
    SmartDashboard.putNumber("Y", currentPose.GetYValue());
    SmartDashboard.putNumber("Angle", currentPose.GetAngleValue());

    SmartDashboard.putNumber("Current X", currentPose.GetXValue());
    SmartDashboard.putNumber("Current Y", currentPose.GetYValue());
    SmartDashboard.putNumber("Current Angle", currentPose.GetAngleValue());
    SmartDashboard.putNumber("Target X", targetPose.GetXValue());
    SmartDashboard.putNumber("Target Y", targetPose.GetYValue());
    SmartDashboard.putNumber("Angle to Target", angleToTarget);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Pose currentPose = driveSubsystem.getCustomPose();
    double xError = targetPose.GetXValue() - currentPose.GetXValue();
    double yError = targetPose.GetYValue() - currentPose.GetYValue();
    double distanceToTarget = Math.sqrt(xError * xError + yError * yError);
    boolean positionOnTarget = distanceToTarget <= positionTolerance;

    double currentAngle = Calculations.NormalizeAngle360(currentPose.GetAngleValue());
    double targetAngle = Calculations.NormalizeAngle360(targetPose.GetAngleValue());
    double angleError = Calculations.shortestAngularDistance(currentAngle, targetAngle);
    //boolean angleOnTarget = angleError <= angleTolerance;
    boolean angleOnTarget = Math.abs(angleError) <= angleTolerance;
    return positionOnTarget && angleOnTarget;
  }

  public void setTarget(double x, double y, double angle) {
    targetPose.SetX(x);
    targetPose.SetY(y);
    targetPose.SetAngle(angle);
  }

}