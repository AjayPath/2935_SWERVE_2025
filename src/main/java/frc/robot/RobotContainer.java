// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.DriveToPoint;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.utils.Pose;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  // Subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final LimelightSubsystem m_limelight = new LimelightSubsystem(m_robotDrive);

  // Controllers
  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();
    configureDefaultCommands();
  }

  /**
   * Configures the default commands for subsystems.
   */
  private void configureDefaultCommands() {
    // Set default drive command: left stick for translation, right stick X for rotation
    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }

  /**
   * Defines button-to-command mappings. Buttons can be created by instantiating a
   * {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, XboxController.Button.kA.value)
      .whileTrue(
        new AutoAlign(m_robotDrive, m_limelight, 0.5));

    new JoystickButton(m_driverController, XboxController.Button.kB.value)
      .whileTrue(
          new DriveToPoint(m_robotDrive, 2.1, -1.7, 0));

    new JoystickButton(m_driverController, XboxController.Button.kY.value)
      .whileTrue(
        new SequentialCommandGroup(
          new DriveToPoint(m_robotDrive, 2.1, -1.7, 0),
          new AutoAlign(m_robotDrive, m_limelight, 0.5)
        )
      );
  }

  /**
   * Returns the autonomous command to run.
   *
   * @return the command to run in autonomous mode
   */
  public Command getAutonomousCommand() {
    return null;
  }

  /**
   * Returns the drive subsystem instance.
   *
   * @return the robot's drive subsystem
   */
  public DriveSubsystem getDriveSubsystem() {
    return m_robotDrive;
  }

  /**
   * Returns the drive subsystem instance.
   *
   * @return the robot's drive subsystem
   */
  public LimelightSubsystem getLimelight() {
    return m_limelight;
  }

  /**
   * Logs current drive status to SmartDashboard for debugging and monitoring.
   */
  public void logDriveStatus() {
    // Get current odometry pose
    Pose customPose = m_robotDrive.getCustomPose();

    // Log pose data to dashboard
    SmartDashboard.putNumber("APOdometry X", customPose.GetXValue());
    SmartDashboard.putNumber("APOdometry Y", customPose.GetYValue());
    SmartDashboard.putNumber("APOdometry Angle", customPose.GetAngleValue());
  }
}