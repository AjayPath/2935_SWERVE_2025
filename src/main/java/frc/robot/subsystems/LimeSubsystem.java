// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.utils.Pose;

// public class LimeSubsystem extends SubsystemBase {
//   /** Creates a new LimeSubsystem. */
//   private final NetworkTable m_limelight;
//   private final DriveSubsystem m_drive;

//   private static final double CAMERA_OFFSET_FROM_CENTER = -0.2;

//   private static final double ALIGNMENT_OFFSET = 0.5;

//   // Cache
//   private double lastValidTX = 0;
//   private double lastValidTY = 0;
//   private double lastValidTA = 0;
//   private double lastValidDistance = 0;
//   private double lastValidTime = 0;
//   private static  final double VISION_CACHE_TIMEOUT = 0.3;

//   private static final double MIN_TARGET_AREA = 0.01;

//   private static final double[][] DISTANCE_LOOKUP_TABLE = {
//     {7.53, 0.3556},
//     {1.0, 0.5398},
//     {-2.63, 0.7366},
//     {-5.64, 1.0795},
//     {-7.2, 1.4859},
//     {-8.4, 2.1463},
//     {-9.37, 2.7432}
//   };

//   public LimeSubsystem(DriveSubsystem m_drive) {
//     this.m_drive = m_drive;
//     m_limelight = NetworkTableInstance.getDefault().getTable("limelight");
//     lastValidTime = Timer.getFPGATimestamp();
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run

//     SmartDashboard.putBoolean("VALID TARGET", hasValidTarget());

//     if (hasValidTarget()) {
//       SmartDashboard.putNumber("TAG ID", getTID());
//       SmartDashboard.putNumber("TX", getTX());
//       SmartDashboard.putNumber("TY", getTY());
//       SmartDashboard.putNumber("TA", getTA());
//       SmartDashboard.putNumber("DISTANCE TO TAG", getDistanceFromTag());

//       Pose targetRelative = getTargetPoseRobotRelative();
//       if (targetRelative != null) {
//         SmartDashboard.putNumber("TARGET X (RR)", targetRelative.GetXValue());
//         SmartDashboard.putNumber("TARGET Y (RR)", targetRelative.GetYValue());
//         SmartDashboard.putNumber("TARGET ANGLE", targetRelative.GetAngleValue());
//       }
//     }

//   }

//   public boolean hasValidTarget() {
//     return m_limelight.getEntry("tv").getDouble(0) == 1.0;
//   }

//   public double getTX() {
//     return m_limelight.getEntry("tx").getDouble(0);
//   }

//   public double getTY() {
//     return m_limelight.getEntry("ty").getDouble(0);
//   }

//   public double getTA() {
//     return m_limelight.getEntry("ta").getDouble(0);
//   }

//   public int getTID() {
//     return (int)m_limelight.getEntry("tid").getDouble(0);
//   }

//   public double getTXCached() {
//     double currentTime = Timer.getFPGATimestamp();

//     if (hasValidTarget()) {
//       lastValidTX = getTX();
//       lastValidTime = currentTime;
//       return lastValidTX;
//     } else if ((currentTime - lastValidTime) < VISION_CACHE_TIMEOUT) {
//       return lastValidTX;
//     } else {
//       return 0;
//     }
//   }

//   public double getTYCached() {
//     double currentTime = Timer.getFPGATimestamp();

//     if (hasValidTarget()) {
//       lastValidTY = getTY();
//       lastValidTime = currentTime;
//       return lastValidTY;
//     } else if ((currentTime - lastValidTime) < VISION_CACHE_TIMEOUT) {
//       return lastValidTY;
//     } else {
//       return 0;
//     }
//   }

//   public double getTACached() {
//     double currentTime = Timer.getFPGATimestamp();

//     if (hasValidTarget()) {
//       lastValidTA = getTA();
//       lastValidTime = currentTime;
//       return lastValidTA;
//     } else if ((currentTime - lastValidTime) < VISION_CACHE_TIMEOUT) {
//       return lastValidTA;
//     } else {
//       return 0;
//     }
//   }

//   private double interpolateDistance(double ty) {
//     if (ty >= DISTANCE_LOOKUP_TABLE[0][0]) {
//       return DISTANCE_LOOKUP_TABLE[0][1];
//     }
//     if (ty <= DISTANCE_LOOKUP_TABLE[DISTANCE_LOOKUP_TABLE.length - 1][0]) {
//       return DISTANCE_LOOKUP_TABLE[DISTANCE_LOOKUP_TABLE.length - 1][1];
//     }
//     for (int i = 0; i < DISTANCE_LOOKUP_TABLE.length - 1; i++) {
//       double ty1 = DISTANCE_LOOKUP_TABLE[i][0];
//       double ty2 = DISTANCE_LOOKUP_TABLE[i + 1][0];

//       if (ty < ty1 && ty >= ty2) {
//         double dist1 = DISTANCE_LOOKUP_TABLE[i][1];
//         double dist2 = DISTANCE_LOOKUP_TABLE[i + 1][1];
//         double t = (ty - ty1) / (ty2 - ty1);
//         return dist1 + t * (dist2 - dist1);
//       }
//     }
//     return 0.0;
//   }

//   public double getDistanceFromTag() {
//     if (!hasValidTarget()) {
//       return lastValidDistance;
//     }

//     double ty = getTY();
//     double distance = interpolateDistance(ty);

//     distance -= CAMERA_OFFSET_FROM_CENTER;

//     lastValidDistance = distance;
//     return distance;
//   }

//   public boolean isVisionConfident() {
//     if (!hasValidTarget()) {
//       return false;
//     }
//     double ta = getTA();
//     return ta > MIN_TARGET_AREA;
//   }

//   public Pose getTargetPoseRobotRelative() {
//     if (!isVisionConfident()) {
//       return null;
//     }

//     double distanceToTag = getDistanceFromTag();
//     double tx = getTXCached();

//     double targetDistance = distanceToTag - ALIGNMENT_OFFSET;

//     double txRAD = Math.toRadians(tx);

//     double targetX = targetDistance * Math.cos(txRAD);
//     double targetY = targetDistance * Math.sin(txRAD);

//     double targetAngle = 0.0;

//     return new Pose(targetX, targetY, targetAngle);
//   }

// }

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.fieldConstants;
import frc.robot.utils.Pose;

public class LimeSubsystem extends SubsystemBase {
  
  private final NetworkTable m_limelight;
  private final DriveSubsystem m_drive;

  // Physical camera offset from robot center (meters)
  // Negative = camera is behind center, Positive = camera is in front
  private static final double CAMERA_OFFSET_FROM_CENTER = -0.2;

  // How far in front of the tag you want to stop (meters)
  private static final double ALIGNMENT_OFFSET = 0.5;

  // Cache for when target is lost
  private double lastValidTX = 0;
  private double lastValidTY = 0;
  private double lastValidTA = 0;
  private double lastValidDistance = 0;
  private double lastValidTime = 0;
  private double lastValidHeading = 0; // Store robot heading when we had valid target
  private static final double VISION_CACHE_TIMEOUT = 1.5;

  // Vision confidence
  private static final double MIN_TARGET_AREA = 0.01;

  // Distance lookup table [TY, Distance in meters]
  private static final double[][] DISTANCE_LOOKUP_TABLE = {
    {7.53, 0.3556},
    {1.0, 0.5398},
    {-2.63, 0.7366},
    {-5.64, 1.0795},
    {-7.2, 1.4859},
    {-8.4, 2.1463},
    {-9.37, 2.7432}
  };

  public LimeSubsystem(DriveSubsystem m_drive) {
    this.m_drive = m_drive;
    m_limelight = NetworkTableInstance.getDefault().getTable("limelight");
    lastValidTime = Timer.getFPGATimestamp();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("VALID TARGET", hasValidTarget());

    if (hasValidTarget()) {
      SmartDashboard.putNumber("TAG ID", getTID());
      SmartDashboard.putNumber("TX", getTX());
      SmartDashboard.putNumber("TY", getTY());
      SmartDashboard.putNumber("TA", getTA());
      SmartDashboard.putNumber("DISTANCE TO TAG", getDistanceFromTag());
    }

    if (hasValidTarget()) {
      int tagID = getTID();
      Pose tagPose = fieldConstants.getTagPose(tagID);
      if (tagPose != null) {
          SmartDashboard.putNumber("Tag Facing Angle", tagPose.GetAngleValue());
      }
  }
  }

  // ========================================================================
  // RAW NETWORK TABLE GETTERS
  // ========================================================================

  public boolean hasValidTarget() {
    return m_limelight.getEntry("tv").getDouble(0) == 1.0;
  }

  public double getTX() {
    return m_limelight.getEntry("tx").getDouble(0);
  }

  public double getTY() {
    return m_limelight.getEntry("ty").getDouble(0);
  }

  public double getTA() {
    return m_limelight.getEntry("ta").getDouble(0);
  }

  public int getTID() {
    return (int)m_limelight.getEntry("tid").getDouble(0);
  }

  // ========================================================================
  // CACHED GETTERS (WITH GYRO COMPENSATION)
  // ========================================================================

  /**
   * Gets TX with caching and gyro compensation.
   * If target is lost, returns last valid TX adjusted for robot rotation.
   * This is the KEY feature from Team 1640!
   */
  public double getTXCached() {
    double currentTime = Timer.getFPGATimestamp();

    if (hasValidTarget()) {
      lastValidTX = getTX();
      lastValidTime = currentTime;
      lastValidHeading = m_drive.getHeading(); // Store current heading
      return lastValidTX;
    } else if ((currentTime - lastValidTime) < VISION_CACHE_TIMEOUT) {
      // Compensate for robot rotation since we lost target
      double headingChange = m_drive.getHeading() - lastValidHeading;
      return lastValidTX - headingChange;
    } else {
      return 0;
    }
  }

  public double getTYCached() {
    double currentTime = Timer.getFPGATimestamp();

    if (hasValidTarget()) {
      lastValidTY = getTY();
      lastValidTime = currentTime;
      return lastValidTY;
    } else if ((currentTime - lastValidTime) < VISION_CACHE_TIMEOUT) {
      return lastValidTY;
    } else {
      return 0;
    }
  }

  public double getTACached() {
    double currentTime = Timer.getFPGATimestamp();

    if (hasValidTarget()) {
      lastValidTA = getTA();
      lastValidTime = currentTime;
      return lastValidTA;
    } else if ((currentTime - lastValidTime) < VISION_CACHE_TIMEOUT) {
      return lastValidTA;
    } else {
      return 0;
    }
  }

  // ========================================================================
  // DISTANCE CALCULATION
  // ========================================================================

  /**
   * Interpolates distance from TY using lookup table
   */
  private double interpolateDistance(double ty) {
    // Check if beyond table bounds
    if (ty >= DISTANCE_LOOKUP_TABLE[0][0]) {
      return DISTANCE_LOOKUP_TABLE[0][1];
    }
    if (ty <= DISTANCE_LOOKUP_TABLE[DISTANCE_LOOKUP_TABLE.length - 1][0]) {
      return DISTANCE_LOOKUP_TABLE[DISTANCE_LOOKUP_TABLE.length - 1][1];
    }

    // Linear interpolation between two points
    for (int i = 0; i < DISTANCE_LOOKUP_TABLE.length - 1; i++) {
      double ty1 = DISTANCE_LOOKUP_TABLE[i][0];
      double ty2 = DISTANCE_LOOKUP_TABLE[i + 1][0];

      if (ty < ty1 && ty >= ty2) {
        double dist1 = DISTANCE_LOOKUP_TABLE[i][1];
        double dist2 = DISTANCE_LOOKUP_TABLE[i + 1][1];
        double t = (ty - ty1) / (ty2 - ty1);
        return dist1 + t * (dist2 - dist1);
      }
    }
    return 0.0;
  }

  /**
   * Gets distance to tag in meters, accounting for camera offset
   */
  public double getDistanceFromTag() {
    if (!hasValidTarget()) {
      return lastValidDistance;
    }

    double ty = getTY();
    double distance = interpolateDistance(ty);

    // Adjust for camera being offset from robot center
    distance -= CAMERA_OFFSET_FROM_CENTER;

    lastValidDistance = distance;
    return distance;
  }

  // ========================================================================
  // VISION CONFIDENCE
  // ========================================================================

  public boolean isVisionConfident() {
    if (!hasValidTarget()) {
      return false;
    }
    double ta = getTA();
    return ta > MIN_TARGET_AREA;
  }

  // ========================================================================
  // ODOMETRY RESET (TEAM 1640 APPROACH)
  // ========================================================================

  /**
   * Resets odometry based on AprilTag detection.
   * Uses tag's known field position to calculate robot's actual position.
   * This is the magic that makes Team 1640's approach work!
   */
  public boolean resetOdometryFromTag() {
    if (!isVisionConfident()) {
      return false;
    }
  
    double distance = getDistanceFromTag();
    double tx = getTXCached();
    double robotHeading = m_drive.getHeading();
  
    // Calculate robot position with TAG AT ORIGIN (0,0)
    // NEGATE distance so robot is in front of tag, not behind
    double angleRad = Math.toRadians(robotHeading - tx);
    
    double robotX = -distance * Math.cos(angleRad);  // NEGATIVE!
    double robotY = distance * Math.sin(angleRad);  // NEGATIVE!
  
    m_drive.setOdom(robotX, robotY, robotHeading);
  
    SmartDashboard.putNumber("Odom Reset X", robotX);
    SmartDashboard.putNumber("Odom Reset Y", robotY);
  
    return true;
  }

  // ========================================================================
  // ALIGNMENT TARGET CALCULATION
  // ========================================================================

  /**
   * Gets the field position where robot should go to align with tag.
   * Returns the pose that is ALIGNMENT_OFFSET meters in front of the tag,
   * facing toward the tag.
   */
  public Pose getAlignmentTargetPose() {
    if (!isVisionConfident()) {
      return null;
    }

    int tagID = getTID();
    Pose tagPose = fieldConstants.getTagPose(tagID);
    
    if (tagPose == null) {
      return null;
    }

    // Calculate position ALIGNMENT_OFFSET meters in front of tag
    // "In front" means in the direction opposite to where tag faces
    double targetAngle = tagPose.GetAngleValue() + 180; // Face toward tag
    double targetAngleRad = Math.toRadians(targetAngle);

    double targetX = tagPose.GetXValue() + ALIGNMENT_OFFSET * Math.cos(targetAngleRad);
    double targetY = tagPose.GetYValue() + ALIGNMENT_OFFSET * Math.sin(targetAngleRad);

    return new Pose(targetX, targetY, targetAngle);
  }

  /**
   * Gets distance to alignment target (not to tag itself)
   */
  public double getDistanceToAlignmentTarget() {
    Pose target = getAlignmentTargetPose();
    if (target == null) {
      return -1;
    }

    Pose current = m_drive.getCustomPose();
    double dx = target.GetXValue() - current.GetXValue();
    double dy = target.GetYValue() - current.GetYValue();
    
    return Math.sqrt(dx * dx + dy * dy);
  }

  /**
 * Gets the alignment target position (offset from tag).
 * Since tag is at origin after odometry reset, this returns
 * the offset position where robot should stop.
 */
public Pose getAlignmentTarget() {
  // Tag is at (0, 0, 0°) after odometry reset
  // We want to stop ALIGNMENT_OFFSET meters back from tag
  // Negative X = back from tag (since tag is at origin)
  return new Pose(-ALIGNMENT_OFFSET, 0.0, 0.0);
}
}