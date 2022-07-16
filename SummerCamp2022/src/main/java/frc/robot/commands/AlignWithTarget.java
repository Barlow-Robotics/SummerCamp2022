// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Vision;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AlignWithTarget extends CommandBase {

  private PIDController pid = new PIDController(Constants.VisionConstants.vision_kp, 0, 0);
  private Vision m_vision;
  private Shooter m_shooter;

  int missedFrames = 0;
  private double error;
  private boolean alignmentComplete = false;

  /** Creates a new AlignWithTarget. */
  public AlignWithTarget(Shooter s, Vision v) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_vision = v;
    m_shooter = s;
    addRequirements(m_vision, m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.reset();
    missedFrames = 0;
    alignmentComplete = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (m_vision.visionTargetIsVisible()) {
    //   System.out.println("Target is visible");
    // } else {
    //   System.out.println("Target is not visible");
    // }

    double rotateVelocity = 0.0;

    if (m_vision.visionTargetIsVisible()) {
      error = m_vision.visionTargetDistanceFromCenter();
      if (Math.abs(error) < Constants.VisionConstants.AlignmentTolerence) {
        alignmentComplete = true;
      } else {
        double adjustment = pid.calculate(error);
        adjustment = Math.signum(adjustment)
            * Math.min(Math.abs(adjustment), Constants.ShooterConstants.Turret.maxTurretOutput );
        rotateVelocity = adjustment;
      }
      m_shooter.rotateTurret(rotateVelocity);
      alignmentComplete = true;
    } else {
      alignmentComplete = false;
      missedFrames++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.rotateTurret(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (missedFrames > 10){
      return true;
    }
    else{
      return false;
    }
  }
}
