// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.telemetry;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.telemetry.sensors.Gyro;

public class Telemetry extends SubsystemBase {

  private Gyro gyro;

  /** Creates a new Telemetry. */
  public Telemetry() {}

  public void init() {
    gyro = new Gyro();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Gyro getGyro() {
    return gyro;
  }
}
