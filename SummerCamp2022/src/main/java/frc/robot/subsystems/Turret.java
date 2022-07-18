// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.Constants;
import frc.robot.sim.PhysicsSim;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Turret extends SubsystemBase {

    WPI_TalonSRX m_turretMotor;

    /** Creates a new Turret. */
    public Turret() {
        m_turretMotor = new WPI_TalonSRX(Constants.ShooterConstants.Turret.ID_Motor);
        setMotorConfig(m_turretMotor);
        m_turretMotor.setInverted(InvertType.InvertMotorOutput);
    }

    public void rotate(double rotateVelocity) {
        m_turretMotor.set(TalonSRXControlMode.PercentOutput, rotateVelocity /*
                                                                             * * Constants.DriveConstants.
                                                                             * MotorVelocityOneMeterPerSecond
                                                                             */ );
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        NetworkTableInstance.getDefault().getEntry("turret/encoder_position").setDouble(m_turretMotor.getSelectedSensorPosition());
    }

    private void setMotorConfig(WPI_TalonSRX motor) {
        motor.configFactoryDefault();
        motor.configClosedloopRamp(Constants.ShooterConstants.closedVoltageRampingConstant);
        motor.configOpenloopRamp(Constants.ShooterConstants.manualVoltageRampingConstant);
        // motor.config_kF(Constants.ShooterConstants.Turret.PID_id,
        // Constants.ShooterConstants.Turret.kf);
        // motor.config_kP(Constants.ShooterConstants.Turret.PID_id,
        // Constants.ShooterConstants.Turret.kP);
        // motor.config_kI(Constants.ShooterConstants.Turret.PID_id,
        // Constants.ShooterConstants.Turret.kI);
        // motor.config_kD(Constants.ShooterConstants.Turret.PID_id,
        // Constants.ShooterConstants.Turret.kD);
        motor.setNeutralMode(NeutralMode.Brake);
    }

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonSRX(m_turretMotor, 0.5, 6800);
    }

}
