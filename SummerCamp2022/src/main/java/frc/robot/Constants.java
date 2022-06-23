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
        public static final double kMaxSpeed = 3.0; // meters per second
        public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

        public static final double kTrackWidth = 0.381 * 2; // meters
        public static final double kWheelRadius = 0.0508; // meters
        public static final int kEncoderResolution = 4096;

        // Are these the same thing? Do we need both?
        public static final double Speed = 0.5;
        public static final double DriveSpeed = 0.5;
    }

    public static final class IndexConstants {
    }

    public static final class ShooterConstants {
        // IDs are placeholders
        public static final int ID_FlyWheelMotor = 1;
        public static final int ID_HoodMotor = 2;
    }

    public static final class UnderGlowConstants {
    }

    public static final class VisionConstants {
    }
}
