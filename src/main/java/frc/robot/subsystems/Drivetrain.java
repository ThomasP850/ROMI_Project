// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.IO;

import frc.robot.subsystems.SubsystemManager;

public class Drivetrain extends SubsystemBase {
  // Track width in meters
  private static final double TRACK_WIDTH = 0.1397;

  private Spark leftSpark;
  private Spark rightSpark;

  private Encoder leftEncoder;
  private Encoder rightEncoder;

  // Wheel diameter in meters
  private static final double WHEEL_DIAMETER = 0.070000114;

  private static final double TICKS_PER_REVOLUTION = 1440;

  private DifferentialDrive drivetrain;

  private DifferentialDriveKinematics kinematics;
  private DifferentialDriveOdometry odometry;

  /** Creates a new Drivetrain. */
  public Drivetrain() {}

  /**
   * Make sure to initialize the gyro before the drivetrain.
   */
  public void init(int leftMotorPort, int rightMotorPort, int leftEncoderPort1, int leftEncoderPort2, int rightEncoderPort1, int rightEncoderPort2) {
    leftSpark = new Spark(leftMotorPort);
    rightSpark = new Spark(rightMotorPort);

    leftEncoder = new Encoder(leftEncoderPort1, leftEncoderPort2);
    rightEncoder = new Encoder(rightEncoderPort1, rightEncoderPort2);

    leftEncoder.setDistancePerPulse((Math.PI * WHEEL_DIAMETER) / TICKS_PER_REVOLUTION);
    rightEncoder.setDistancePerPulse((Math.PI * WHEEL_DIAMETER) / TICKS_PER_REVOLUTION);

    drivetrain = new DifferentialDrive(leftSpark, rightSpark);

    kinematics = new DifferentialDriveKinematics(TRACK_WIDTH);

    odometry = new DifferentialDriveOdometry(
      Rotation2d.fromDegrees(SubsystemManager.getInstance().getTelemetry().getGyro().getAngleZ())
    );
  }

  @Override
  public void periodic() {
    if(odometry != null) {
      odometry.update(
        Rotation2d.fromDegrees(SubsystemManager.getInstance().getTelemetry().getGyro().getAngleZ()),
        leftEncoder.getDistance(),
        rightEncoder.getDistance()
      );
    }
    if(DriverStation.getInstance().isOperatorControl()) {
      IO io = SubsystemManager.getInstance().getIO();
      drive(new Translation2d(io.getStrafeX(), io.getStrafeY()), io.getRotationX());
    }
  }

  public void resetOdometry() {
    odometry.resetPosition(getPosition(), Rotation2d.fromDegrees(SubsystemManager.getInstance().getTelemetry().getGyro().getAngleZ()));
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public Pose2d getPosition() {
    return odometry.getPoseMeters();
  }

  public void drive(Translation2d translation, double rotation) {
    ChassisSpeeds speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    leftSpark.set(wheelSpeeds.leftMetersPerSecond);
    rightSpark.set(wheelSpeeds.rightMetersPerSecond);
  }
}
