// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Index extends SubsystemBase {

  WPI_TalonSRX hopperMotor;
  WPI_TalonSRX conveyorMotor;

  boolean conveyorIsRunning = false;

  /** Creates a new Index. */
  public Index() {
    hopperMotor = new WPI_TalonSRX(Constants.IndexConstants.ID_HopperMotor);
    conveyorMotor = new WPI_TalonSRX(Constants.IndexConstants.ID_ConveyorMotor);
  
    setMotorConfig(hopperMotor);
    setMotorConfig(conveyorMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void startIndex() {
    conveyorMotor.set(TalonSRXControlMode.Velocity, Constants.IndexConstants.ConveyorMotorSpeed);
    conveyorIsRunning = true;
  }

  public void stopIndex() {
    conveyorMotor.set(TalonSRXControlMode.Velocity, 0);
    conveyorIsRunning = false;
  }
  
  public void startHopper() {
    if(conveyorIsRunning = true) {
      hopperMotor.set(TalonSRXControlMode.Velocity, Constants.IndexConstants.HopperMotorSpeed);
    }
  }

  public void stopHopper() {
    if(conveyorIsRunning = false) {
      hopperMotor.set(TalonSRXControlMode.Velocity, 0);
    }
  }

  private void setMotorConfig(WPI_TalonSRX motor) {
    motor.configFactoryDefault();
    motor.configClosedloopRamp(Constants.IndexConstants.closedVoltageRampingConstant);
    motor.configOpenloopRamp(Constants.IndexConstants.manualVoltageRampingConstant);
    motor.config_kF(Constants.IndexConstants.PID_id, Constants.ShooterConstants.kF);
    motor.config_kP(Constants.IndexConstants.PID_id, Constants.ShooterConstants.kP);
    motor.config_kI(Constants.IndexConstants.PID_id, Constants.ShooterConstants.kI);
    motor.config_kD(Constants.IndexConstants.PID_id, Constants.ShooterConstants.kD);
    motor.setNeutralMode(NeutralMode.Brake);
  }
}
