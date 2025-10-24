package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Configs;
import frc.robot.Constants.Drive;
import frc.robot.Constants.IDs;
import frc.robot.Constants.Operating;
import frc.robot.Constants.Drive.Constants.MotorLocation;

//Drive Subsystem
public class DriveSubsystem extends SubsystemBase{
    //Creates Swerve Modules
     private final SwerveModule m_frontLeft = new SwerveModule(
        IDs.DriveConstants.k_frontLeftDrivingID,
        IDs.DriveConstants.k_frontLeftTurningID,
        Drive.Constants.k_frontLeftChassisAngularOffset,
        Drive.Constants.k_frontLeftInverted,
        Configs.SwerveModule.k_frontLeftDriveConfig,
        MotorLocation.FRONT_LEFT);

    private final SwerveModule m_backLeft = new SwerveModule(
        IDs.DriveConstants.k_backLeftDrivingID,
        IDs.DriveConstants.k_backLeftTurningID,
        Drive.Constants.k_backLeftChassisAngularOffset,
        Drive.Constants.k_backLeftInverted,
        Configs.SwerveModule.k_backLeftDriveConfig,
        MotorLocation.BACK_LEFT);

    private final SwerveModule m_frontRight = new SwerveModule(
        IDs.DriveConstants.k_frontRightDrivingID,
        IDs.DriveConstants.k_frontRightTurningID,
        Drive.Constants.k_frontRightChassisAngularOffset,
        Drive.Constants.k_frontRightInverted,
        Configs.SwerveModule.k_frontRightDriveConfig,
        MotorLocation.FRONT_RIGHT);

    private final SwerveModule m_backRight = new SwerveModule(
        IDs.DriveConstants.k_backRightDrivingID,
        IDs.DriveConstants.k_backRightTurningID,
        Drive.Constants.k_backRightChassisAngularOffset,
        Drive.Constants.k_backRightInverted,
        Configs.SwerveModule.k_backRightDriveConfig,
        MotorLocation.BACK_RIGHT);

    private final Pigeon2 m_gyro = Operating.Constants.k_usingGyro ? new Pigeon2(IDs.DriveConstants.k_pigeon2ID) : null;

    private SwerveModuleState m_desiredStates[] = {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};
    private StructArrayPublisher<SwerveModuleState> publisherDesieredStates = NetworkTableInstance.getDefault().getStructArrayTopic("MyDesiredStates", SwerveModuleState.struct).publish();
    private StructArrayPublisher<SwerveModuleState> publisherActualStates = NetworkTableInstance.getDefault().getStructArrayTopic("MyActualStates", SwerveModuleState.struct).publish();
     
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
        Drive.Constants.k_driveKinematics,
        getRotation2d(),
        getSwerveModulePosition());

    //Constructs a new DriveSubsystem
    public DriveSubsystem() {
        //Usage reporting for MAXSwerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
        RobotConfig config;

        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            config = null;
            e.printStackTrace();
        }

        //Configure AutoBuilder here later
        configureAutoBuilder();
    }

    public void drive(double p_xSpeed, double p_ySpeed, double p_rot, boolean p_fieldRelative, String p_statusName) {
        //Convert speeds into proper units for drivetrain
        double d_multiplier = 0.5; //update(?)
        double d_xSpeedDelivered = p_xSpeed * Drive.Constants.k_maxSpeedMetersPerSecond * d_multiplier;
        double d_ySpeedDelivered = p_ySpeed * Drive.Constants.k_maxSpeedMetersPerSecond * d_multiplier;
        double d_rotDelivered = p_rot * Drive.Constants.k_maxAngularSpeed * d_multiplier;

        //was type 'var' beforehand, fix?
        SwerveModuleState[] m_swerveModuleStates = Drive.Constants.k_driveKinematics.toSwerveModuleStates(
            p_fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(d_xSpeedDelivered, d_ySpeedDelivered, d_rotDelivered,
                getRotation2d())
                : new ChassisSpeeds(d_xSpeedDelivered, d_ySpeedDelivered, d_rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(
            m_swerveModuleStates, Drive.Constants.k_maxSpeedMetersPerSecond);
        m_desiredStates = m_swerveModuleStates;
        m_frontLeft.setDesiredState(m_swerveModuleStates[0]);
        m_frontRight.setDesiredState(m_swerveModuleStates[1]);
        m_backLeft.setDesiredState(m_swerveModuleStates[2]);
        m_backRight.setDesiredState(m_swerveModuleStates[3]);

        SmartDashboard.putString("Drive Mode", p_statusName);
    }

    //WIP
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        double multipler = 1;
        double alt = 1;
        drive(-robotRelativeSpeeds.vxMetersPerSecond/Drive.Constants.k_maxSpeedMetersPerSecond * multipler, 
        -robotRelativeSpeeds.vyMetersPerSecond/Drive.Constants.k_maxSpeedMetersPerSecond * multipler, 
        -robotRelativeSpeeds.omegaRadiansPerSecond/Drive.Constants.k_maxAngularSpeed * alt, false, 
        "AutoBuilder");
        
        /*
         // Stripped from Template Pathplanner Github
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
  
        SwerveModuleState[] targetStates = DriveConstants.k_DriveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
         */
      }

    //returns a list of SwerveModulesStates
    public SwerveModuleState[] getSwerveModuleState() {
        return new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState()
        };
    }

    //returns a list of SwerveModulePositions
    public SwerveModulePosition[] getSwerveModulePosition() {
        return new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        };
    }

    //Returns the heading of the robot.
    public Rotation2d getRotation2d() {
        //return new Rotation2d(edu.wpi.first.math.util.Units.degreesToRadians(0)); //This way avoids issues
        if(Operating.Constants.k_usingGyro) 
            return new Rotation2d(m_gyro.getYaw().getValue());
        else 
            return new Rotation2d(0);
    }

    //Returns the currently-estimated pose of the robot
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    //WIP
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Drive.Constants.k_driveKinematics.toChassisSpeeds(
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState()
        );
   }

    //Resets odometry to the specified pose.
    public void resetOdometry(Pose2d p_pose) {
        m_odometry.resetPosition(
            getRotation2d(),
            getSwerveModulePosition(),
            p_pose);
    }

    //Resests drive encoders to read a position of 0.
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_backLeft.resetEncoders();
        m_backRight.resetEncoders();
    }

    //Zereos the heading of the robot.
    public void zeroHeading() {
        if(Operating.Constants.k_usingGyro) m_gyro.reset();
    }

    //Stops all motors on the DriveSubsystem.
    public void stopModules() {
        m_frontLeft.stopMotors();
        m_frontRight.stopMotors();
        m_backLeft.stopMotors();
        m_backRight.stopMotors();

        for(int i = 0; i < m_desiredStates.length; i++)
            m_desiredStates[i].speedMetersPerSecond = 0;
    }

    //WIP
    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                this::getPose,   // Supplier of current robot pose
                this::resetOdometry,         // Consumer for seeding pose against auto
                this::getRobotRelativeSpeeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                this::driveRobotRelative,
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(.04, 0, 0), //Change(?)
                    // PID constants for rotation
                    new PIDConstants(1, 0, 0) //Change(?)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    @Override
    public void periodic() {
        m_odometry.update(getRotation2d(), getSwerveModulePosition());

        if(Operating.Debugging.k_driveDebug) {
            updateSmartDashboard();
            //updateWheelPositions - add for extra debugging functionality
            publisherDesieredStates.set(m_desiredStates);
            publisherActualStates.set(getSwerveModuleState());

            m_frontLeft.updateSmartDashboard();
            m_frontRight.updateSmartDashboard();
            m_backLeft.updateSmartDashboard();
            m_backRight.updateSmartDashboard();
        }
    }

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("Robot Heading Yaw", getRotation2d().getDegrees());
        if(Operating.Constants.k_usingGyro) {
            SmartDashboard.putNumber("Roll", m_gyro.getRoll().getValue().in(Units.Degree));
            SmartDashboard.putNumber("Pitch", m_gyro.getPitch().getValue().in(Units.Degree));
        }
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    //Might want to add getWheelRotationSupplier() and any test methods if needed
}