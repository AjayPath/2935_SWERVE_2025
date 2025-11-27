  // Copyright (c) FIRST and other WPILib contributors.
  // Open Source Software; you can modify and/or share it under the terms of
  // the WPILib BSD license file in the root directory of this project.

  package frc.robot;

  import edu.wpi.first.math.geometry.Translation2d;
  import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
  import edu.wpi.first.math.util.Units;
  import frc.robot.utils.Pose;


  /**
   * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
   * constants. This class should not be used for any other purpose. All constants should be declared
   * globally (i.e. public static). Do not put anything functional in this class.
   *
   * <p>It is advised to statically import this class (or one of its inner classes) wherever the
   * constants are needed, to reduce verbosity.
   */
  public final class Constants {
    
    public static final class DriveConstants {
      // Maximum allowed speeds (not hardware limits)
      public static final double kMaxSpeedMetersPerSecond = 4.8;
      public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

      // Chassis dimensions
      public static final double kTrackWidth = Units.inchesToMeters(26.5);  // Left to right wheel distance
      public static final double kWheelBase = Units.inchesToMeters(26.5);   // Front to back wheel distance
      
      // Swerve drive kinematics (module positions: FL, FR, BL, BR)
      public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
          new Translation2d(kWheelBase / 2, kTrackWidth / 2),
          new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
          new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
          new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

      // Module angular offsets relative to chassis (radians)
      public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
      public static final double kFrontRightChassisAngularOffset = 0;
      public static final double kBackLeftChassisAngularOffset = Math.PI;
      public static final double kBackRightChassisAngularOffset = Math.PI / 2;

      // Drive motor CAN IDs
      public static final int kFrontLeftDrivingCanId = 11;
      public static final int kRearLeftDrivingCanId = 13;
      public static final int kFrontRightDrivingCanId = 15;
      public static final int kRearRightDrivingCanId = 17;

      // Turning motor CAN IDs
      public static final int kFrontLeftTurningCanId = 10;
      public static final int kRearLeftTurningCanId = 12;
      public static final int kFrontRightTurningCanId = 14;
      public static final int kRearRightTurningCanId = 16;

      // Gyro configuration
      public static final int kGryoID = 24;
      public static final boolean kGyroReversed = false;
    }

    public static final class ModuleConstants {
      // MAXSwerve module pinion gear (12T, 13T, or 14T options)
      public static final int kDrivingMotorPinionTeeth = 14;

      // Motor and wheel specifications
      public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
      public static final double kWheelDiameterMeters = 0.0762;
      public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
      
      // Gear reduction: (45 teeth wheel bevel * 22 teeth spur) / (pinion teeth * 15 teeth bevel)
      public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
      public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
          / kDrivingMotorReduction;
    }

    public static final class OIConstants {
      public static final int kDriverControllerPort = 0;
      public static final double kDriveDeadband = 0.05;
    }

    public static final class AutoConstants {

    }

    public static final class NeoMotorConstants {
      public static final double kFreeSpeedRpm = 5676;
    }

    public static final class fieldConstants {
      public static final Pose TAG_1 = new Pose(2, 0, 0);

      public static Pose getTagPose(int tagID) {
        switch(tagID) {
          case 1: return TAG_1;
          default: return null;
        }
      }
    }
  }