// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents a differential drive style drivetrain. */
public class DriveSubsystem extends SubsystemBase{
  WPI_TalonSRX m_leftLeader;
  WPI_VictorSPX m_leftFollower;
  WPI_TalonSRX m_rightLeader;
  WPI_VictorSPX m_rightFollower;
  
  DifferentialDrive diffDrive = new DifferentialDrive(m_leftLeader, m_rightLeader);

  private final AnalogGyro m_gyro = new AnalogGyro(0);

  private final DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(Constants.DriveConstants.kTrackWidth);

  private final DifferentialDriveOdometry m_odometry;

 // private Navigation navigationSubsystem;

  // Gains are for example purposes only - must be determined for your own robot!
  //private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

  /**
   * Constructs a differential drive object. Sets the encoder distance per pulse and resets the
   * gyro.
   */
  public DriveSubsystem() {
    
    m_leftLeader = new WPI_TalonSRX(Constants.DriveConstants.ID_leftLeaderMotor);
    m_leftFollower = new WPI_VictorSPX(Constants.DriveConstants.ID_leftFollowerMotor);
    m_rightLeader = new WPI_TalonSRX(Constants.DriveConstants.ID_rightLeaderMotor);
    m_rightFollower = new WPI_VictorSPX(Constants.DriveConstants.ID_rightFollowerMotor);

    // functions for setMotorConfig() ?
    
    m_gyro.reset();

    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  }

  // public void periodic() { 
  //   NetworkTableInstance.getDefault().getEntry("drive/left_motor_distance").setDouble(getLeftDistance());
  //   NetworkTableInstance.getDefault().getEntry("drive/right_motor_distance").setDouble(getRightDistance());
  //   NetworkTableInstance.getDefault().getEntry("drive/rotation").setDouble(navSubSystem.getRotation2d().getDegrees());
  //   NetworkTableInstance.getDefault().getEntry("drive/leftSpeed").setDouble(getLeftSpeed());
  //   NetworkTableInstance.getDefault().getEntry("drive/rightSpeed").setDouble(getRightSpeed());

  //   // Update the odometry in the periodic block
  //   m_odometry.update( navSubSystem.getRotation2d(), getLeftDistance(), getRightDistance());
  //   NetworkTableInstance.getDefault().getEntry("drive/pose/x").setDouble(m_odometry.getPoseMeters().getX());
  //   NetworkTableInstance.getDefault().getEntry("drive/pose/y").setDouble(m_odometry.getPoseMeters().getY());
  //   NetworkTableInstance.getDefault().getEntry("drive/pose/rotation")
  //           .setDouble(m_odometry.getPoseMeters().getRotation().getDegrees());
  // }

  /**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    m_leftLeader.set(Constants.DriveConstants.DriveSpeed);
    m_rightLeader.set(Constants.DriveConstants.DriveSpeed);
  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot Angular velocity in rad/s.
   * @param squareInputs Decreases input sensitivity at low speeds.
   */
   @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double rot, boolean squareInputs) {
    NetworkTableInstance.getDefault().getEntry("drive/xSpeed").setDouble(m_leftLeader.getSelectedSensorPosition());
    NetworkTableInstance.getDefault().getEntry("drive/rot").setDouble(m_leftLeader.getSelectedSensorPosition());
    NetworkTableInstance.getDefault().getEntry("drive/arcadeDrive").setDouble(100.0);
    diffDrive.arcadeDrive(xSpeed, rot, squareInputs);
  }
  
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  private void CreateNetworkTableEntries() {
    NetworkTableInstance.getDefault().getEntry("drive/left_motor_distance").setDouble(0.0);
    NetworkTableInstance.getDefault().getEntry("drive/right_motor_distance").setDouble(0.0);
    NetworkTableInstance.getDefault().getEntry("drive/rotation").setDouble(0.0);

    NetworkTableInstance.getDefault().getEntry("drive/leftSpeed").setDouble(0.0);
    NetworkTableInstance.getDefault().getEntry("drive/rightSpeed").setDouble(0.0);

    NetworkTableInstance.getDefault().getEntry("drive/xSpeed").setDouble(0.0);
    NetworkTableInstance.getDefault().getEntry("drive/rot").setDouble(0.0);
    NetworkTableInstance.getDefault().getEntry("drive/arcadeDrive").setDouble(0.0);
    NetworkTableInstance.getDefault().getEntry("drive/tankDrive").setDouble(0.0);

    NetworkTableInstance.getDefault().getEntry("drive/leftVolts").setDouble(0.0);
    NetworkTableInstance.getDefault().getEntry("drive/rightVolts").setDouble(0.0);

    NetworkTableInstance.getDefault().getEntry("drive/pose/x").setDouble(0.0);
    NetworkTableInstance.getDefault().getEntry("drive/pose/y").setDouble(0.0);
    NetworkTableInstance.getDefault().getEntry("drive/pose/rotation").setDouble(0.0);
  }

  private void setMotorConfig() {
    m_leftLeader.configFactoryDefault();
    m_leftFollower.configFactoryDefault();
    m_rightLeader.configFactoryDefault();
    m_rightFollower.configFactoryDefault();

    m_leftFollower.follow(m_leftLeader);
    m_rightFollower.follow(m_rightLeader);

    m_leftLeader.setInverted(TalonSRXInvertType.CounterClockwise);
    m_rightLeader.setInverted(TalonSRXInvertType.Clockwise);
    
    m_leftFollower.setInverted(InvertType.FollowMaster);
    m_rightFollower.setInverted(InvertType.FollowMaster);

    diffDrive.setRightSideInverted(false);
  }
}
