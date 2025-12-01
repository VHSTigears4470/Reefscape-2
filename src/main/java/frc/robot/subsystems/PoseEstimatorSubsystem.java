package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Cameras;

public class PoseEstimatorSubsystem extends SubsystemBase{
 
    private final PhotonCamera[] m_cameras = {new PhotonCamera("camera1"), new PhotonCamera("camera2")};
    private final DriveSubsystem m_driveSubsystem;

    private static final AprilTagFieldLayout m_targetPoses = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static final Matrix<N3, N1> m_stateStdDevs = VecBuilder.fill(0.1,0.1,Units.degreesToRadians(5));
    public static final Matrix<N3, N1> m_visionMeasurementStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));
    private final SwerveDrivePoseEstimator m_poseEstimator;

    private final Field2d m_field2d = new Field2d();

    private final PhotonPipelineResult[] m_previousResult = {null, null};

    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
            .getStructTopic("MyPose", Pose2d.struct).publish();

    public PoseEstimatorSubsystem(DriveSubsystem p_driveSubsystem) {
        m_driveSubsystem = p_driveSubsystem;
        m_poseEstimator = new SwerveDrivePoseEstimator(
            Drive.Constants.k_driveKinematics,
            m_driveSubsystem.getRotation2d(),
            m_driveSubsystem.getSwerveModulePosition(),
            new Pose2d(),
            m_stateStdDevs,
            m_visionMeasurementStdDevs);  
    }

    @Override
    public void periodic() {
        for(int i = 0; i < m_cameras.length; i++){
            var pipelineResult = m_cameras[i].getLatestResult();
            if(!pipelineResult.equals(m_previousResult[i]) && pipelineResult.hasTargets()) {
                m_previousResult[i] = pipelineResult;
                double imageCaptureTime = pipelineResult.getTimestampSeconds();
                for(PhotonTrackedTarget target : pipelineResult.getTargets()) {
                    var ficidualID = target.getFiducialId();
                    Optional<Pose3d> targetPose3d = m_targetPoses.getTagPose(ficidualID);
                    if(!targetPose3d.isEmpty()) {
                        Pose2d targetPose = targetPose3d.get().toPose2d();
                        Transform3d camToTarget = target.getBestCameraToTarget();
                        Transform2d transform = new Transform2d(
                            camToTarget.getTranslation().toTranslation2d(),
                            camToTarget.getRotation().toRotation2d());
                        Pose2d camPose = targetPose.transformBy(transform.inverse()); //inverse?
                        var visionMeasurement = camPose.transformBy(Cameras.Constants.CAMERA_TO_ROBOT[i]); //GET CAMERA TRANSLATIONS
                        m_poseEstimator.addVisionMeasurement(visionMeasurement, imageCaptureTime);
                    }
                }
            }
            
        }
        m_poseEstimator.updateWithTime(
            Timer.getFPGATimestamp(),
            m_driveSubsystem.getRotation2d(),
            m_driveSubsystem.getSwerveModulePosition());
        m_field2d.setRobotPose(getCurrentPose());
        publisher.set(getCurrentPose());
    }

    public Pose2d getCurrentPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public void setCurrentPose(Pose2d newPose) {
        m_driveSubsystem.zeroHeading();
        m_poseEstimator.resetPosition(m_driveSubsystem.getRotation2d(),
            m_driveSubsystem.getSwerveModulePosition(),
            newPose);
    }

    public void resetFieldPosition() {
        m_driveSubsystem.zeroHeading();
        m_poseEstimator.resetPosition(m_driveSubsystem.getRotation2d(),
            m_driveSubsystem.getSwerveModulePosition(),
            new Pose2d());
    }

    //Add to shuffleboard maybe
    private String getFormattedPose() {
        var pose = getCurrentPose();
        return String.format("(%.2f, %.2f)",
             Units.inchesToMeters(pose.getX()),
             Units.inchesToMeters(pose.getY()));
    }
}
