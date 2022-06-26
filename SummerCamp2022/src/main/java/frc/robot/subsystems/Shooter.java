// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Shooter extends SubsystemBase {
  // Creates a new Shooter

  WPI_TalonFX flywheelMotor;
  WPI_TalonSRX hoodMotor;
  WPI_TalonSRX rotateMotor;

  boolean isShooting = false;

  public Shooter() {
    flywheelMotor = new WPI_TalonFX(Constants.ShooterConstants.ID_FlyWheelMotor);
    hoodMotor = new WPI_TalonSRX(Constants.ShooterConstants.ID_HoodMotor);
    rotateMotor = new WPI_TalonSRX(Constants.ShooterConstants.ID_RotateMotor);

    setMotorConfig(flywheelMotor);
    setMotorConfig(hoodMotor);
    setMotorConfig(rotateMotor);
  }

  public void startShooting() {
    flywheelMotor.set(TalonFXControlMode.Velocity, Constants.ShooterConstants.shootMotorVelocity);
    isShooting = true;
  }

  public void stopShooting() {
    flywheelMotor.set(TalonFXControlMode.Velocity, 0);
    isShooting = false;
  }

  public void adjustTurretHoodAnglePos() {
    hoodMotor.set(TalonSRXControlMode.Velocity, Constants.ShooterConstants.ManualHoodAdjustmentSpeed);
  }

  public void adjustTurretHoodAngleNeg() {
    hoodMotor.set(TalonSRXControlMode.Velocity, -Constants.ShooterConstants.ManualHoodAdjustmentSpeed);
  }
  
  public void rotateTurretPos() {
    rotateMotor.set(TalonSRXControlMode.Velocity, Constants.ShooterConstants.ManualRotateTurretSpeed);
  }

  public void rotateTurretNeg() {
    rotateMotor.set(TalonSRXControlMode.Velocity, -Constants.ShooterConstants.ManualRotateTurretSpeed);
  }

  public double getFlywheelSpeed() {
    double s = flywheelMotor.getSelectedSensorVelocity() * 10.0 * Constants.ShooterConstants.Meters_Per_Count;
    return (s); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void setMotorConfig(WPI_TalonSRX motor) {
    motor.configFactoryDefault();
    motor.configClosedloopRamp(Constants.ShooterConstants.closedVoltageRampingConstant);
    motor.configOpenloopRamp(Constants.ShooterConstants.manualVoltageRampingConstant);
    motor.config_kF(Constants.ShooterConstants.PID_id, Constants.ShooterConstants.kF);
    motor.config_kP(Constants.ShooterConstants.PID_id, Constants.ShooterConstants.kP);
    motor.config_kI(Constants.ShooterConstants.PID_id, Constants.ShooterConstants.kI);
    motor.config_kD(Constants.ShooterConstants.PID_id, Constants.ShooterConstants.kD);
    motor.setNeutralMode(NeutralMode.Brake);
  }

  private void setMotorConfig(WPI_TalonFX motor) {
    motor.configFactoryDefault();
    motor.configClosedloopRamp(Constants.ShooterConstants.closedVoltageRampingConstant);
    motor.configOpenloopRamp(Constants.ShooterConstants.manualVoltageRampingConstant);
    motor.config_kF(Constants.ShooterConstants.PID_id, Constants.ShooterConstants.kF);
    motor.config_kP(Constants.ShooterConstants.PID_id, Constants.ShooterConstants.kP);
    motor.config_kI(Constants.ShooterConstants.PID_id, Constants.ShooterConstants.kI);
    motor.config_kD(Constants.ShooterConstants.PID_id, Constants.ShooterConstants.kD);
    motor.setNeutralMode(NeutralMode.Brake);
  }
}
