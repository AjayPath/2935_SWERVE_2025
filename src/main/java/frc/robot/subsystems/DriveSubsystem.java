// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.utils.APOdometry;
import frc.robot.utils.Pose;
import frc.robot.utils.SlewRateLimiter;

/**
 * Swerve drive subsystem for a MAXSwerve robot.
 * 
 * Hardware:
 * - Four MAXSwerve modules (FL, FR, BL, BR)
 * - Pigeon2 gyroscope for heading
 * 
 * Odometry:
 * - Uses custom APOdometry for position tracking
 * - Tracks each wheel module independently
 * - Averages wheel positions for robot center pose
 * 
 * Control:
 * - Field-relative and robot-relative drive modes
 * - Slew rate limiting for smooth acceleration
 * - X-formation for defensive positioning
 * - Automatic wheel speed desaturation
 */
public class DriveSubsystem extends SubsystemBase {

  // ===========================================================================================
  // Swerve Modules
  // ===========================================================================================

  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // ===========================================================================================
  // Sensors
  // ===========================================================================================

  private final Pigeon2 m_gyro = new Pigeon2(DriveConstants.kGryoID);

  // ===========================================================================================
  // Odometry
  // ===========================================================================================

  private final APOdometry APOdom = APOdometry.getInstance(
      List.of(m_frontLeft, m_frontRight, m_rearLeft, m_rearRight),
      m_gyro);

  // ===========================================================================================
  // Rate Limiting
  // ===========================================================================================

  private final SlewRateLimiter xLimiter = new SlewRateLimiter(500, 20000);  // X-axis (forward/backward)
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(500, 20000);  // Y-axis (left/right)
  private final SlewRateLimiter rLimiter = new SlewRateLimiter(24, 240);  // Rotation

  // ===========================================================================================
  // Constructor
  // ===========================================================================================

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
  }

  // ===========================================================================================
  // Periodic
  // ===========================================================================================

  @Override
  public void periodic() {
    // Update custom odometry every cycle (20ms)
    APOdom.update();
  }

  // ===========================================================================================
  // Drive Control
  // ===========================================================================================

  /**
   * Drives the robot with specified velocities.
   * Applies slew rate limiting for smooth acceleration.
   * 
   * @param xSpeed speed in x direction (forward/backward), normalized -1 to 1
   * @param ySpeed speed in y direction (left/right), normalized -1 to 1
   * @param rot angular rate (rotation), normalized -1 to 1
   * @param fieldRelative true for field-relative control, false for robot-relative
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Apply slew rate limiting to prevent sudden acceleration
    //xSpeed = xLimiter.CalculateSlewRate(xSpeed);
    //ySpeed = yLimiter.CalculateSlewRate(ySpeed);
    //rot = rLimiter.CalculateSlewRate(rot);

    // Convert normalized inputs to physical units
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    // Calculate module states from desired chassis speeds
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedDelivered, ySpeedDelivered, rotDelivered, m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

    // Ensure no wheel exceeds maximum speed
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    // Command each module to desired state
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   * Useful for defense or preventing being pushed.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Directly sets the swerve module states.
   * Used for advanced control (e.g., trajectory following).
   * 
   * @param desiredStates array of desired states [FL, FR, BL, BR]
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  // ===========================================================================================
  // Odometry - Custom Pose (Primary)
  // ===========================================================================================

  /**
   * Gets the current robot pose from custom odometry.
   * Returns pose with normalized angle [0, 360).
   * 
   * @return current robot pose (x, y in meters, angle in degrees)
   */
  public Pose getCustomPose() {
    return APOdom.getPose();
  }

  /**
   * Gets the current robot pose with continuous angle tracking.
   * Use this for control loops that need unwrapped angles.
   * 
   * @return current robot pose with continuous angle
   */
  public Pose getCustomPoseContinuous() {
    return APOdom.getPoseContinuous();
  }

  /**
   * Sets the robot pose to a specific position.
   * Updates odometry to the exact specified pose.
   * 
   * @param x x position in meters
   * @param y y position in meters
   * @param angleDeg heading angle in degrees
   */
  public void setOdom(double x, double y, double angleDeg) {
    Pose newPose = new Pose(x, y, angleDeg);
    APOdom.setPose(newPose);
  }

  // ===========================================================================================
  // Odometry - WPILib Pose2d Compatibility
  // ===========================================================================================

  /**
   * Gets the current robot pose as WPILib Pose2d.
   * Provided for compatibility with WPILib utilities (PathPlanner, etc.).
   * 
   * @return current pose as Pose2d
   */
  public Pose2d getPose() {
    Pose customPose = APOdom.getPose();
    return new Pose2d(
        customPose.GetXValue(),
        customPose.GetYValue(),
        Rotation2d.fromDegrees(customPose.GetAngleValue()));
  }

  /**
   * Resets odometry to a specific WPILib Pose2d.
   * Provided for compatibility with WPILib utilities.
   * 
   * @param pose the desired robot pose
   */
  public void resetOdometry(Pose2d pose) {
    setOdom(
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  // ===========================================================================================
  // Odometry Reset
  // ===========================================================================================

  /**
   * Resets all odometry to zero (origin).
   * Waits for gyro to provide valid readings before resetting.
   * Sets robot pose to (0, 0, 0°).
   */
  public void resetAllOdometryToZero() {
    // Wait for gyro to initialize (max 2 seconds)
    Timer t = new Timer();
    t.start();
    while (Double.isNaN(m_gyro.getRotation2d().getDegrees()) && t.get() < 2.0) {
      Timer.delay(0.01);
    }

    // Reset gyro to zero
    m_gyro.setYaw(0, 100);

    // Reset custom odometry to origin
    setOdom(0.0, 0.0, 0.0);
  }

  /**
   * Initializes odometry with current gyro heading at origin.
   * Used during robot startup after gyro stabilization.
   */
  public void initializeOdometry() {
    double heading = m_gyro.getRotation2d().getDegrees();
    if (!Double.isNaN(heading)) {
      APOdom.setPose(new Pose(0, 0, heading));
    }
  }

  // ===========================================================================================
  // Encoder Management
  // ===========================================================================================

  /**
   * Resets all drive encoders to zero.
   * Does not affect odometry - only resets encoder positions.
   */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearLeft.resetEncoders();
    m_rearRight.resetEncoders();
  }

  // ===========================================================================================
  // Gyro Access
  // ===========================================================================================

  /**
   * Gets the current robot heading.
   * 
   * @return heading in degrees (continuous, may exceed 360°)
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Gets the current gyro rotation as Rotation2d.
   * 
   * @return current gyro rotation
   */
  public Rotation2d getGyro() {
    return m_gyro.getRotation2d();
  }

  /**
   * Gets the pigeon isntance.
   * 
   * @return pigeon
   */
  public Pigeon2 getPigeon() {
    return m_gyro;
  }

  /**
   * Resets the gyro to zero heading.
   * Does not reset odometry - only zeros the gyro sensor.
   */
  public void resetGyro() {
    m_gyro.setYaw(0, 100); // 100ms timeout
  }

  /**
   * Gets the current turn rate of the robot.
   * 
   * @return angular velocity signal (degrees per second)
   */
  public StatusSignal<AngularVelocity> getTurnRate() {
    return m_gyro.getAngularVelocityZWorld();
  }

  // ===========================================================================================
  // Rate Limiter Management
  // ===========================================================================================

  /**
   * Resets all slew rate limiters to zero.
   * Call this at the start of teleop to prevent sudden jumps from previous values.
   */
  public void resetSlewRateLimiters() {
    xLimiter.ResetSlewRate(0.0);
    yLimiter.ResetSlewRate(0.0);
    rLimiter.ResetSlewRate(0.0);
  }
}