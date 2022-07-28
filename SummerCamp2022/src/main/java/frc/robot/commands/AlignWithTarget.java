// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Vision;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Turret;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AlignWithTarget extends CommandBase {

    private PIDController pid = new PIDController(Constants.ShooterConstants.Turret.kp, 0, 0);
    private Vision m_vision;
    private Turret m_turret;
    private Hood m_hood ;

    private double hoodPositionLUT[] = new double[50] ;


    int missedFrames = 0;
    private boolean alignmentComplete = false;

    /** Creates a new AlignWithTarget. */
    public AlignWithTarget(Turret t, Vision v, Hood h) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_vision = v;
        m_turret = t;
        m_hood = h ;

        addRequirements(m_turret, m_hood);

        for ( int i = 0 ; i < 10; i++) {
            hoodPositionLUT[i] = 0.0 ;
        }
        //wpk need to update this
        hoodPositionLUT[10] = 1.0 ;
        hoodPositionLUT[11] = 1.0 ;
        hoodPositionLUT[12] = 1.0 ;
        hoodPositionLUT[13] = 1.0 ;
        hoodPositionLUT[14] = 1.0 ;
        hoodPositionLUT[15] = 1.0 ;
        hoodPositionLUT[16] = 1.0 ;
        hoodPositionLUT[17] = 1.0 ;
        hoodPositionLUT[18] = 1.0 ;
        hoodPositionLUT[19] = 1.0 ;
        hoodPositionLUT[20] = 1.0 ;
        hoodPositionLUT[21] = 1.0 ;
        hoodPositionLUT[22] = 1.0 ;
        hoodPositionLUT[23] = 1.0 ;
        hoodPositionLUT[24] = 1.0 ;
        hoodPositionLUT[25] = 1.0 ;
        hoodPositionLUT[26] = 1.0 ;
        hoodPositionLUT[27] = 1.0 ;
        hoodPositionLUT[28] = 1.0 ;
        hoodPositionLUT[29] = 1.0 ;
        hoodPositionLUT[30] = 1.0 ;
        for ( int i = 31 ; i < hoodPositionLUT.length ; i++) {
            hoodPositionLUT[i] = 0.0 ;
        }

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        pid.reset();
        missedFrames = 0;
        alignmentComplete = false;
    }



    double getDistanceToTarget( double boxHeight, double boxYPos ) {
        // add logic here
        return 10.0 ;
    }


    double getHoodPosition(double distanceToTarget) {

        int lowerBound = (int) distanceToTarget ;
        int upperBound = (int) (distanceToTarget+1.0) ;

        double percentRange = distanceToTarget - lowerBound ;

        // wpk need to finish calculations

        return hoodPositionLUT[lowerBound] ;
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
        double azimuthError = 0.0 ;

        if (m_vision.visionTargetIsVisible()) {
            azimuthError = -m_vision.visionTargetDistanceFromCenter();
            if (Math.abs(azimuthError) < Constants.ShooterConstants.Turret.AlignmentTolerence) {
                alignmentComplete = true;
            } else {
                double adjustment = pid.calculate(azimuthError);
                adjustment = Math.signum(adjustment)
                        * Math.min(Math.abs(adjustment), Constants.ShooterConstants.Turret.maxTurretOutput);
                rotateVelocity = adjustment;
            }
            //m_turret.rotate(0.0);
            //System.out.println("Alignment adjustment is " + rotateVelocity);
            m_turret.rotateTurret(rotateVelocity);

            //wpk need to update
//            m_hood.setServoPosition(getHoodPosition(getDistanceToTarget(100.0, 0.2)));
            m_hood.setServoPosition(0.00);


        } else {
            //System.out.println("Target not visible");
            missedFrames++;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_turret.rotateTurret(0.0);
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
