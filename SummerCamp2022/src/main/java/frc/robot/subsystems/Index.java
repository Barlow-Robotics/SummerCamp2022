// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.sim.PhysicsSim;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Index extends SubsystemBase {

  WPI_TalonSRX m_hopperMotor;
  WPI_TalonSRX m_conveyorMotor;

  boolean conveyorIsRunning = false;  
  boolean simulationInitialized = false;

  /** Creates a new Index. */
  public Index() {
    m_hopperMotor = new WPI_TalonSRX(Constants.IndexConstants.ID_HopperMotor);
    m_conveyorMotor = new WPI_TalonSRX(Constants.IndexConstants.ID_ConveyorMotor);
  
    setMotorConfig(m_hopperMotor);
    setMotorConfig(m_conveyorMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void startIndex() {
    m_conveyorMotor.set(TalonSRXControlMode.PercentOutput, Constants.IndexConstants.conveyorMotorSpeed);
    conveyorIsRunning = true;
  }

  public void stopIndex() {
    m_conveyorMotor.set(TalonSRXControlMode.PercentOutput, 0);
    conveyorIsRunning = false;
  }
  
  public void startHopper() {
    m_hopperMotor.set(TalonSRXControlMode.PercentOutput, Constants.IndexConstants.hopperMotorSpeed);
  }

  public void stopHopper() {
      m_hopperMotor.set(TalonSRXControlMode.PercentOutput, 0);
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

  public void simulationInit() {
      PhysicsSim.getInstance().addTalonSRX(m_conveyorMotor, 0.5, 6800);
      PhysicsSim.getInstance().addTalonSRX(m_hopperMotor, 0.5, 6800);
    }

  @Override
  public void simulationPeriodic() {
      if (!simulationInitialized) {
          simulationInit();
          simulationInitialized = true;
      }
  }
}
