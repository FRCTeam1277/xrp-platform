// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveDistanceHeading extends Command {
  private static final double kHeadingKp = 0.02;
  private static final double kMaxRotate = 0.5;

  private final Drivetrain m_drive;
  private final double m_distance;
  private final double m_speed;
  private final boolean m_captureHeading;
  private double m_targetHeading;

  /**
   * Creates a new DriveDistanceHeading. This command will drive your robot a desired distance at a
   * desired speed while holding the heading it had when the command started.
   *
   * @param speed The speed at which the robot will drive. Negative is in reverse.
   * @param inches The number of inches the robot will drive
   * @param drive The drivetrain subsystem on which this command will run
   */
  public DriveDistanceHeading(double speed, double inches, Drivetrain drive) {
    this(speed, inches, Double.NaN, drive);
  }

  /**
   * Creates a new DriveDistanceHeading. This command will drive your robot a desired distance at a
   * desired speed while holding the given heading.
   *
   * @param speed The speed at which the robot will drive. Negative is in reverse.
   * @param inches The number of inches the robot will drive
   * @param heading The gyro angle, in degrees, to hold while driving
   * @param drive The drivetrain subsystem on which this command will run
   */
  public DriveDistanceHeading(double speed, double inches, double heading, Drivetrain drive) {
    m_speed = speed;
    m_distance = inches;
    m_targetHeading = heading;
    m_captureHeading = Double.isNaN(heading);
    m_drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.arcadeDrive(0, 0);
    m_drive.resetEncoders();
    if (m_captureHeading) {
      m_targetHeading = m_drive.getGyroAngleZ();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error = m_targetHeading - m_drive.getGyroAngleZ();
    // Normalize to the shortest way around
    error = ((error + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;

    double rotate = kHeadingKp * error;
    rotate = Math.max(-kMaxRotate, Math.min(kMaxRotate, rotate));

    m_drive.arcadeDrive(m_speed, rotate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_drive.getAverageDistanceInch()) >= Math.abs(m_distance);
  }
}
