// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Vision;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AlignWithTarget extends CommandBase {

    private PIDController pid = new PIDController(Constants.ShooterConstants.Turret.kp, 0, 0);
    private Vision m_vision;
    private Turret m_turret;

    int missedFrames = 0;
    private double error;
    private boolean alignmentComplete = false;

    /** Creates a new AlignWithTarget. */
    public AlignWithTarget(Turret t, Vision v) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_vision = v;
        m_turret = t;
        addRequirements(m_vision, m_turret);
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
        // System.out.println("Target is visible");
        // } else {
        // System.out.println("Target is not visible");
        // }

        double rotateVelocity = 0.0;

        if (m_vision.visionTargetIsVisible()) {
            error = -m_vision.visionTargetDistanceFromCenter();
            if (Math.abs(error) < Constants.ShooterConstants.Turret.AlignmentTolerence) {
                alignmentComplete = true;
            } else {
                double adjustment = pid.calculate(error);
                adjustment = Math.signum(adjustment)
                        * Math.min(Math.abs(adjustment), Constants.ShooterConstants.Turret.maxTurretOutput);
                rotateVelocity = adjustment;
            }
            //m_turret.rotate(0.0);
            //System.out.println("Alignment adjustment is " + rotateVelocity);
            m_turret.rotate(rotateVelocity);
        } else {
            //System.out.println("Target not visible");
            missedFrames++;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_turret.rotate(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false ;
        // if (missedFrames > 10) {
        //     return true;
        // } else {
        //     return false;
        // }
    }
}
