// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  public Vision() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

// ---PLACEHOLDERS---
// public boolean targetIsVisible() {
//     //The data for this will come from the Jetson Nano via network tables.
//     return NetworkTableInstance.getDefault().getEntry("vision/vision_target_detected").getBoolean(false);
//   }
//   public double targetDistanceFromCenter() {
//     //returns the number of pixels from the center of the screen to the center of the vision target. 
//     //The data for this will come from the Jetson Nano via network tables.
//     return NetworkTableInstance.getDefault().getEntry("vision/vision_target_distance_from_center").getDouble(0.0);
//   }
  
}
