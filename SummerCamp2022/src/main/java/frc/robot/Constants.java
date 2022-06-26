// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveConstants {
        // All variables are placeholders 
        public static final double kMaxSpeed = 3.0; // meters per second
        public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second
        public static final double DriveSpeed = 0.5;

        public static final double kTrackWidth = 0.381 * 2; // meters
        public static final double kWheelRadius = 0.0508; // meters
        public static final int kEncoderResolution = 4096;
        
        public static final double Gear_Ratio = 10.71;
        public static final double Counts_Per_Revolution = 2048.0 * Gear_Ratio;
        public static final double InchesToMeters = 0.0254;
        public static final double Wheel_Diameter = 6.0 * InchesToMeters;
        public static final double Meters_Per_Revolution = Wheel_Diameter * Math.PI ;
        public static final double Meters_Per_Count = Meters_Per_Revolution / Counts_Per_Revolution;

        public static int ID_leftLeaderMotor = 3;
        public static int ID_leftFollowerMotor = 4;
        public static int ID_rightLeaderMotor = 5;
        public static int ID_rightFollowerMotor = 6;
    }

    public static final class IndexConstants {
        // All variables are placeholders 
        public static final int ID_HopperMotor = 0;
        public static final int ID_ConveyorMotor = 0;
        public static final int PID_id = 0;

        public static final double closedVoltageRampingConstant = 0;
        public static final double manualVoltageRampingConstant = 0;
        public static final double HopperMotorSpeed = 0;
        public static final double ConveyorMotorSpeed = 0;
    }

    public static final class ShooterConstants {
        // All variables are placeholders 
        public static final int ID_FlyWheelMotor = 1;
        public static final int ID_HoodMotor = 2;
        public static final int ID_RotateMotor = 0;
        public static final double shootMotorVelocity = 0;
    
        public static final double kF = 0.048;
        public static final double kP = 0.001;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final int PID_id = 0;

        public static final double closedVoltageRampingConstant = 0.0;
        public static final double manualVoltageRampingConstant = 0.0;
        public static final double ManualHoodAdjustmentSpeed = 0;
        public static final double ManualRotateTurretSpeed = 0;
        
        public static final double Gear_Ratio = 10.71;
        public static final double Counts_Per_Revolution = 2048.0 * Gear_Ratio;
        public static final double InchesToMeters = 0.0254;
        public static final double Wheel_Diameter = 6.0 * InchesToMeters;
        public static final double Meters_Per_Revolution = Wheel_Diameter * Math.PI ;
        public static final double Meters_Per_Count = Meters_Per_Revolution / Counts_Per_Revolution;
    }

    public static final class UnderGlowConstants {
    }

    public static final class VisionConstants {
        public static final double vision_kp = 0.005;
        public static final double vision_ki = 0;
        public static final double vision_kd = 0;
    }

    public static final class AutoConstants {
    }

    public final class Logitech_F310_Controller {
        // Constants for Axes
        public static final int Left_Stick_X = 0;
        public static final int Left_Stick_Y = 1;
        public static final int Left_Trigger = 2;
        public static final int Right_Trigger = 3;
        public static final int Right_Stick_X = 4;
        public static final int Right_Stick_Y = 5;

        // Constants for buttons
        public static final int Button_A = 1;
        public static final int Button_B = 2;
        public static final int Button_X = 3;
        public static final int Button_Y = 4;
        public static final int Left_Bumper = 5;
        public static final int Right_Bumper = 6;
        public static final int Back_Button = 7;
        public static final int Start_Button = 8;
        public static final int Left_Stick = 9;
        public static final int Right_Stick = 10;

        public static final double Forward_Axis_Attenuation = -0.5 ;
        public static final double Lateral_Axis_Attenuation = 0.5 ;
        public static final double Yaw_Axis_Attenuation = 0.5 ;
    }

    public final class Logitech_Dual_Action {
        // Constants for Axes
        public static final int Left_Stick_X = 0;
        public static final int Left_Stick_Y = 1;
        public static final int Right_Stick_X = 2;
        public static final int Right_Stick_Y = 3;

        // Constants for buttons
        public static final int Left_Trigger = 7;
        public static final int Right_Trigger = 8;
        public static final int Button_A = 2;
        public static final int Button_B = 3;
        public static final int Button_X = 1;
        public static final int Button_Y = 4;
        public static final int Left_Bumper = 5;
        public static final int Right_Bumper = 6;
        public static final int Back_Button = 9;
        public static final int Start_Button = 10;
        public static final int Left_Stick = 11;
        public static final int Right_Stick = 12;

        public static final double Forward_Axis_Attenuation = -0.5 ;
        public static final double Lateral_Axis_Attenuation = 0.5 ;
        public static final double Yaw_Axis_Attenuation = 0.5 ;
    }
}
