// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AdjustTurretHoodAngle;
import frc.robot.commands.AlignWithTarget;
import frc.robot.commands.RotateTurret;
import frc.robot.commands.StartIndexAndShooter;
import frc.robot.commands.StopIndexAndShooter;
import frc.robot.commands.TurnOffUnderGlow;
import frc.robot.commands.TurnOnUnderGlow;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.UnderGlow;
import frc.robot.subsystems.Vision;

import java.util.HashMap;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final Index m_index = new Index();
  private final Shooter m_shooter = new Shooter();
  private final UnderGlow m_underGlow = new UnderGlow();
  private final Vision m_vision = new Vision();
  private final Turret m_turret = new Turret();
  private final Hood m_hood = new Hood();

  private final AlignWithTarget alignWithTargetCommand = new AlignWithTarget(m_turret, m_vision, m_hood);
  private final AdjustTurretHoodAngle adjustTurretHoodAngleCommand = new AdjustTurretHoodAngle(m_hood);
  //private final RotateTurret rotateTurretCommand = new RotateTurret(m_turret);
  private final StartIndexAndShooter startIndexAndShooterCommand = new StartIndexAndShooter(m_shooter, m_index);
  private final StopIndexAndShooter stopIndexAndShooterCommand = new StopIndexAndShooter(m_index, m_shooter);
  // private final TurnOffUnderGlow turnOffUnderGlowCommand = new TurnOffUnderGlow(m_underGlow);
  // private final TurnOnUnderGlow turnOnUnderGlowCommand = new TurnOnUnderGlow(m_underGlow);

  Joystick m_driverController; // Joystick 1
  Joystick m_operatorController; // Joystick 2

  private JoystickButton alignWithTargetButton;
  private JoystickButton indexAndShooterButton;

  // HashMap<String, PathPlannerTrajectory> trajectories;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    // loadTrajectories();
    // createAutonomousCommands();

    m_drive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand( // new instance
            () -> {
              double x = -m_driverController.getRawAxis(Constants.Logitech_Dual_Action.Left_Stick_Y);
              double yaw = m_driverController.getRawAxis(Constants.Logitech_Dual_Action.Right_Stick_X);
              // fancy exponential formulas to shape the controller inputs to be flat when
              // only
              // pressed a little, and ramp up as stick pushed more.
              double speed = 0.0;
              if (x != 0) {
                speed = (Math.abs(x) / x) * (Math.exp(-400.0 * Math.pow(x / 3.0, 4.0)))
                    + (-Math.abs(x) / x);
              }
              double turn = -yaw ;
              // double turn = 0.0;
              // if (yaw != 0) {
              //   turn = (Math.abs(yaw) / yaw) * (Math.exp(-400.0 * Math.pow(yaw / 3.0, 4.0)))
              //       + (-Math.abs(yaw) / yaw);
              // }
              // The turn input results in really quick movement of the bot, so
              // let's reduce the turn input and make it even less if we are going faster
              // This is a simple y = mx + b equation to adjust the turn input based on the
              // speed.
              //turn = turn * (-0.4 * Math.abs(speed) + 0.5);

              m_drive.drive(-speed, -turn * 0.4, false);
            },
            m_drive));

            m_turret.setDefaultCommand(
              // A split-stick arcade command, with forward/backward controlled by the left
              // hand, and turning controlled by the right.
              new RunCommand( // new instance
                  () -> {
                    double yaw = m_operatorController.getRawAxis(Constants.Logitech_Dual_Action.Right_Stick_X) * Constants.ShooterConstants.Turret.maxTurretOutput ;
                    m_turret.rotateTurret(yaw);
                  },
                  m_turret));
      
            m_hood.setDefaultCommand(
              new RunCommand(
                   () -> {
                    double servoIncrement = 0;
                    if ((m_operatorController.getRawAxis(Constants.Logitech_Dual_Action.Left_Stick_Y)) > 0.2){
                      servoIncrement = 0.01;
                    }
                    else if ((m_operatorController.getRawAxis(Constants.Logitech_Dual_Action.Left_Stick_Y)) < -0.2){
                      servoIncrement = -0.01;
                    }
                    m_hood.movePosition(servoIncrement);
                   },
                   m_hood));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    if (m_driverController == null) {
      System.out.println("Null driver controller, using joystick 1");
      m_driverController = new Joystick(1);
    }

    if (m_operatorController == null) {
      System.out.println("Null operator controller, using joystick 2");
      m_operatorController = new Joystick(2);
    }

    String controllerType = m_driverController.getName();
    System.out.println("The controller name is " + controllerType);
    //boolean controllerFound = false;

    alignWithTargetButton = new JoystickButton(m_operatorController, Constants.Logitech_Dual_Action.Left_Bumper);
    indexAndShooterButton = new JoystickButton(m_operatorController, Constants.Logitech_Dual_Action.Right_Bumper);

    alignWithTargetButton.whileHeld(alignWithTargetCommand);
    indexAndShooterButton.whenPressed(startIndexAndShooterCommand).whenReleased(stopIndexAndShooterCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An ExampleCommand will run in autonomous
    
  //   return m_autoCommand;
  // }
}