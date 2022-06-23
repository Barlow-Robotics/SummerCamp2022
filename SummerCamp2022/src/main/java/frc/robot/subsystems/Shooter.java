// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Shooter extends SubsystemBase {
  // Creates a new Shooter

WPI_TalonSRX flyWheelMotor;
WPI_TalonSRX hoodMotor;

boolean isShooting = false;

  public Shooter() {
    flyWheelMotor = new WPI_TalonSRX(Constants.ShooterConstants.ID_FlyWheelMotor);
    hoodMotor = new WPI_TalonSRX(Constants.ShooterConstants.ID_HoodMotor);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
