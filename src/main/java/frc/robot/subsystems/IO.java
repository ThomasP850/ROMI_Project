// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class IO extends SubsystemBase {

  private Joystick controller;

  /** Creates a new IO. */
  public IO() {}

  public void init() {
    controller = new Joystick(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getStrafeX() {
    if(controller != null) {
      return controller.getX(Hand.kLeft);
    }
    return 0;
  }
  
  public double getStrafeY() {
    if(controller != null) {
      return controller.getY(Hand.kLeft);
    }
    return 0;
  }

  public double getRotationX() {
    if(controller != null) {
      return controller.getX(Hand.kRight);
    }
    return 0;
  }
}
