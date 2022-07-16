// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.sim.PhysicsSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Shooter extends SubsystemBase {
  // Creates a new Shooter

  WPI_TalonFX m_flywheelMotor;
  WPI_TalonSRX m_turretMotor;

  boolean isShooting = false;
  boolean simulationInitialized = false;

  public Shooter() {
    m_flywheelMotor = new WPI_TalonFX(Constants.ShooterConstants.Flywheel.ID_Motor);
    m_turretMotor = new WPI_TalonSRX(Constants.ShooterConstants.Turret.ID_Motor);

    setMotorConfig(m_flywheelMotor);
    m_flywheelMotor.setInverted(TalonFXInvertType.Clockwise);
//    setMotorConfig(m_hoodMotor);
    setMotorConfig(m_turretMotor);
  }

  public void startShooting() {
    m_flywheelMotor.set(TalonFXControlMode.Velocity, Constants.ShooterConstants.Flywheel.flywheelMotorVelocity);
    isShooting = true;
  }

  public void stopShooting() {
    m_flywheelMotor.set(TalonFXControlMode.Velocity, 0);
    isShooting = false;
  }

  public void adjustTurretHoodAngle(double hoodVelocity) {
//    m_hoodMotor.set(TalonSRXControlMode.PercentOutput, hoodVelocity /* * Constants.DriveConstants.MotorVelocityOneMeterPerSecond*/ );
    }
  
  public void rotateTurret(double rotateVelocity) {
    m_turretMotor.set(TalonSRXControlMode.PercentOutput, rotateVelocity /* * Constants.DriveConstants.MotorVelocityOneMeterPerSecond*/ );
  }

  // public double getFlywheelSpeed() {
  //   double s = m_flywheelMotor.getSelectedSensorVelocity() * 10.0 * Constants.ShooterConstants.Meters_Per_Count;
  //   return (s); 
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void setMotorConfig(WPI_TalonSRX motor) {
    motor.configFactoryDefault();
    motor.configClosedloopRamp(Constants.ShooterConstants.closedVoltageRampingConstant);
    motor.configOpenloopRamp(Constants.ShooterConstants.manualVoltageRampingConstant);
    // motor.config_kF(Constants.ShooterConstants.Turret.PID_id, Constants.ShooterConstants.Turret.kf);
    // motor.config_kP(Constants.ShooterConstants.Turret.PID_id, Constants.ShooterConstants.Turret.kP);
    // motor.config_kI(Constants.ShooterConstants.Turret.PID_id, Constants.ShooterConstants.Turret.kI);
    // motor.config_kD(Constants.ShooterConstants.Turret.PID_id, Constants.ShooterConstants.Turret.kD);
    motor.setNeutralMode(NeutralMode.Brake);
  }

  private void setMotorConfig(WPI_TalonFX motor) {
    motor.configFactoryDefault();
    motor.configOpenloopRamp(0.1) ;
    motor.configClosedloopRamp(Constants.ShooterConstants.closedVoltageRampingConstant);
    motor.configOpenloopRamp(Constants.ShooterConstants.manualVoltageRampingConstant);
    motor.config_kF(Constants.ShooterConstants.Flywheel.PID_id, Constants.ShooterConstants.Flywheel.kF);
    motor.config_kP(Constants.ShooterConstants.Flywheel.PID_id, Constants.ShooterConstants.Flywheel.kP);
    motor.config_kI(Constants.ShooterConstants.Flywheel.PID_id, Constants.ShooterConstants.Flywheel.kI);
    motor.config_kD(Constants.ShooterConstants.Flywheel.PID_id, Constants.ShooterConstants.Flywheel.kD);
    motor.setNeutralMode(NeutralMode.Coast);
  }

  public void simulationInit() {
    PhysicsSim.getInstance().addTalonFX(m_flywheelMotor, 0.5, 6800);  
    PhysicsSim.getInstance().addTalonSRX(m_turretMotor, 0.5, 6800);
//    PhysicsSim.getInstance().addTalonSRX(m_hoodMotor, 0.5, 6800);
    }

  @Override
  public void simulationPeriodic() {
      if (!simulationInitialized) {
          simulationInit();
          simulationInitialized = true;
      }
  }
}
