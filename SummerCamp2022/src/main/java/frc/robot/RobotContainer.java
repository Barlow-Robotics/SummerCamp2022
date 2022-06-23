// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AdjustTurretHoodAngle;
import frc.robot.commands.AlignWithTarget;
import frc.robot.commands.RotateTurret;
import frc.robot.commands.StartIndex;
import frc.robot.commands.StartShooting;
import frc.robot.commands.StartSpinningHopper;
import frc.robot.commands.TurnOffLEDs;
import frc.robot.commands.TurnOnLEDs;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.UnderGlow;
import frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final Index m_index = new Index();
  private final Shooter m_shooter = new Shooter();
  private final UnderGlow m_underGlow = new UnderGlow();
  private final Vision m_vision = new Vision();

  private final AlignWithTarget alignWithTargetCommand = new AlignWithTarget(m_shooter, m_vision);
  private final AdjustTurretHoodAngle adjustTurretHoodAngleCommand = new AdjustTurretHoodAngle(m_shooter);
  private final RotateTurret rotateTurretCommand = new RotateTurret(m_shooter);
  private final StartIndex startIndexCommand = new StartIndex(m_index);
  private final StartShooting startShootingCommand = new StartShooting(m_shooter);
  private final StartSpinningHopper startSpinningHopperCommand = new StartSpinningHopper(m_index);
  private final TurnOffLEDs turnOffLEDsCommand = new TurnOffLEDs(m_underGlow);
  private final TurnOnLEDs turnOnLEDsCommand = new TurnOnLEDs(m_underGlow);
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
