package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {

    public static final class ShooterConstants {
        public static final int ShooterCanID = 11;
        public static final double ShooterSpeed = .5;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort0 = 0;
        public static final int kDriverControllerPort1 = 1;
        public static final int kDriverControllerPort2 = 2;
        public static final double kDriveDeadband = 0.1;
    }

    public static final class MotorConstants {

        public static final int AmpLimit550 = 20;
        public static final int AmpLimitNeo = 50;

    }

    public static final class ElevatorConstants {
        // Elevator Can IDs
        public static final int ElevatorCanID = 4;
        public static final int ElevatorCanID2 = 5;

        // Eleavator Speed
        public static final double ElevatorSpeed = .3;

        // Elavator macanical constants

        public static boolean ElevatorReverse = false; // Set to true if the elevator is reversed
        public static double GearRedution = 15; // Gearbox reduction
        public static double PitchDia = 1.751; // Pitch diameter of the drive sprocket see:
                                               // https://tinyurl.com/25Sprockets

        // PID Constants **dangerous to change**
        public static double kp = 0.0;
        public static double ki = 0;
        public static double kd = 0;
        public static double kErrorTol = .02;

        // Elevator Setpoints
        public static double kL1 = 0;
        public static double kL2 = 0;
        public static double kL3 = 0;
        public static double kL4 = 0;

    }

    public static final class ClimberConstants {

        public static final int FrontClimbCanID = 2;
        public static final int BackClimbCanID = 3;
        public static final double ClimbSpeed = 0.4;
    }

    public static final class PivotConstants {

        public static final int PivotCanID = 7;
        public static final double PivotSpeed = .15;
        public static final double PivotMaxSpeed = .6;
    }

}
