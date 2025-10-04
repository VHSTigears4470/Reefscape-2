package frc.robot.Constants;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.IDs.ElevatorConstants;

public final class Configs {
    public static final class SwerveModule {
        public static final SparkMaxConfig k_turningConfig = new SparkMaxConfig();
        public static final SparkMaxConfig k_frontLeftDriveConfig = new SparkMaxConfig();
        public static final SparkMaxConfig k_frontRightDriveConfig = new SparkMaxConfig();
        public static final SparkMaxConfig k_backLeftDriveConfig = new SparkMaxConfig();
        public static final SparkMaxConfig k_backRightDriveConfig = new SparkMaxConfig();

        static {
            //Module constants used to calculate conversion factors and feed forward gain.
            double d_drivingFactor = Drive.ModuleConstants.k_wheelDiameters * Math.PI
                / Drive.ModuleConstants.k_drivingMotorReduction;
            double d_drivingVelocityFeedForward = 1 / Drive.ModuleConstants.k_driveWheelFreeSpeedRps;
            double d_turningFactor = 2 * Math.PI;

            //Update as needed
            k_turningConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(20);
            k_turningConfig.absoluteEncoder
                .inverted(true)
                .positionConversionFactor(d_turningFactor)
                .velocityConversionFactor(d_turningFactor / 60.0);
            k_turningConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(1,0,0) //update
                .outputRange(-1, 1)
                //PID wrap around optimizes angle for turning
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, d_turningFactor);

            k_frontLeftDriveConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50)
                .inverted(false);
            k_frontLeftDriveConfig.encoder
                .positionConversionFactor(d_drivingFactor) //meters
                .velocityConversionFactor(d_drivingFactor / 60.0);
            k_frontLeftDriveConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.04, 0,0)
                .velocityFF(d_drivingVelocityFeedForward)
                .outputRange(-1, 1);

            k_frontRightDriveConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50)
                .inverted(true);
            k_frontRightDriveConfig.encoder
                .positionConversionFactor(d_drivingFactor) //meters
                .velocityConversionFactor(d_drivingFactor / 60.0);
            k_frontRightDriveConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.04, 0,0)
                .velocityFF(d_drivingVelocityFeedForward)
                .outputRange(-1, 1);

            k_backLeftDriveConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50)
                .inverted(false);
            k_backLeftDriveConfig.encoder
                .positionConversionFactor(d_drivingFactor) //meters
                .velocityConversionFactor(d_drivingFactor / 60.0);
            k_backLeftDriveConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.04, 0,0)
                .velocityFF(d_drivingVelocityFeedForward)
                .outputRange(-1, 1);

            k_backRightDriveConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50)
                .inverted(false);
            k_backRightDriveConfig.encoder
                .positionConversionFactor(d_drivingFactor) //meters
                .velocityConversionFactor(d_drivingFactor / 60.0);
            k_backRightDriveConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.04, 0,0)
                .velocityFF(d_drivingVelocityFeedForward)
                .outputRange(-1, 1);
        }
    }

    public static final class Elevator {
        public static final SparkMaxConfig elevatorConfigRight = new SparkMaxConfig();
        public static final SparkMaxConfig elevatorConfigLeft = new SparkMaxConfig();


        static {
            elevatorConfigRight
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(50)
                .voltageCompensation(12);
            elevatorConfigRight.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control - Edit
                .p(0.1)
                .outputRange(-1, 1)
                .maxMotion
                // Set MAXMotion parameters for position control - Edit
                .maxVelocity(4200)
                .maxAcceleration(6000)
                .allowedClosedLoopError(0.5);
            elevatorConfigLeft
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(50)
                .voltageCompensation(12)
                .follow(ElevatorConstants.k_elevatorRightID);
            elevatorConfigLeft.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control - Edit
                .p(0.1)
                .outputRange(-1, 1)
                .maxMotion
                // Set MAXMotion parameters for position control - Edit
                .maxVelocity(4200)
                .maxAcceleration(6000)
                .allowedClosedLoopError(0.5);
        }
    }
}
