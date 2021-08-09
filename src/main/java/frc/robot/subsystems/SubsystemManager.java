// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.telemetry.Telemetry;
import frc.robot.subsystems.Drivetrain;

public class SubsystemManager extends SubsystemBase {

  private static SubsystemManager instance;

  private Telemetry telemetry;
  private Drivetrain drivetrain;
  private IO io;

  /** Creates a new SubsystemManager. */
  public SubsystemManager() {
    if(instance == null) {
      instance = this;
    }
  }

  public static SubsystemManager getInstance() {
    if(instance == null) {
      instance = new SubsystemManager();
      instance.init();
    }
    return instance;
  }

  public void init() {

    io = new IO();
    io.init();

    telemetry = new Telemetry();
    telemetry.init();

    drivetrain = new Drivetrain();
    drivetrain.init(0, 1, 4, 5, 6, 7);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Telemetry getTelemetry() {
    return telemetry;
  }

  public Drivetrain getDrivetrain() {
    return drivetrain;
  }

  public IO getIO() {
    return io;
  }
}
