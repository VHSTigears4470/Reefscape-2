package frc.robot.Constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Drive {
    public static final class Constants {
        //Maxmimum allowed speeds - update(?)
        public static final double k_maxSpeedMetersPerSecond = 4.46;
        public static final double k_maxAngularSpeed = 2 * Math.PI; //radians per second

        public static final double k_wheelBase = Units.inchesToMeters(24.5);
        public static final double k_trackWidth = Units.inchesToMeters(24.5);

        public static final SwerveDriveKinematics k_driveKinematics = new SwerveDriveKinematics(
            new Translation2d(k_wheelBase / 2, k_trackWidth / 2),    //Front Left
            new Translation2d(k_wheelBase / 2, -k_trackWidth / 2),   //Front Right
            new Translation2d(-k_wheelBase / 2, k_trackWidth / 2),   //Back Left
            new Translation2d(-k_wheelBase / 2, -k_trackWidth / 2)); //Back Right

        //update(?)
        public static final double k_frontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double k_backLeftChassisAngularOffset =  Math.PI;
        public static final double k_frontRightChassisAngularOffset = 0;
        public static final double k_backRightChassisAngularOffset = Math.PI / 2;


        //update
        public static final boolean k_frontLeftInverted = false; 
        public static final boolean k_backLeftInverted = false; 
        public static final boolean k_frontRightInverted = false; 
        public static final boolean k_backRightInverted = false; 


        public enum MotorLocation {
            FRONT_LEFT,
            FRONT_RIGHT,
            BACK_LEFT,
            BACK_RIGHT
        }

        public static final String Configs = null;;
    }

    //Update(?)
    public static final class ModuleConstants {
        //MAXSwerve modules are configured with one of three pinion gears;
        //12T, 13T, or 14T. More teeth leads to a faster chassis.
        public static final int k_driveMotorPinionTeeth = 13;
        public static final double k_wheelDiameters = Units.inchesToMeters(3);
        public static final double k_wheelCircumference = k_wheelDiameters * Math.PI;
        public static final double k_drivingMotorReduction = (45.0 * 22.0) / (k_driveMotorPinionTeeth * 15);
        public static final double k_driveMotorFreeSpeedRps = 5676.0 / 60.0;
        public static final double k_driveWheelFreeSpeedRps = (k_driveMotorFreeSpeedRps * k_wheelCircumference) / k_drivingMotorReduction;
    }
}
