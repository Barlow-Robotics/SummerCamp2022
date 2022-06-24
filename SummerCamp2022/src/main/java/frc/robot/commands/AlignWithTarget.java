// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AlignWithTarget extends CommandBase {

  private PIDController pid = new PIDController(0.01, 0, 0);
  private Vision m_vision;
  private Shooter m_shooter;

  private double error;
  private double leftVelocity;
  private double rightVelocity;
  private int missedFrames = 0;
  private double adjustment;

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
    missedFrames = 0 ;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
