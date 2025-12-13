package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision;

public class VisionIO extends SubsystemBase{
    private final PhotonCamera[] cameras;

    private final PhotonPoseEstimator[] photonEstimators;
    private List<Matrix<N3, N1>> curStdDevs;
    private Matrix<N3, N1> stdDevsIter;
    //Add second matrix?
    private final EstimateConsumer estConsumer;
  

    public VisionIO(EstimateConsumer estConsumer) {
        this.estConsumer = estConsumer;
        cameras = new PhotonCamera[]{new PhotonCamera("camera1"), new PhotonCamera("camera2")};

        photonEstimators = new PhotonPoseEstimator[2];
        photonEstimators[0] = new PhotonPoseEstimator(Vision.Constants.m_targetPoses, 
                                                      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                                                      Vision.Constants.CAMERA_TO_ROBOT[0]);
        photonEstimators[1] = new PhotonPoseEstimator(Vision.Constants.m_targetPoses,
                                                      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                                                      Vision.Constants.CAMERA_TO_ROBOT[1]);
        photonEstimators[0].setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        photonEstimators[1].setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        curStdDevs.add(null);
        curStdDevs.add(null);
    }

    @Override
    public void periodic() {
        List<Optional<EstimatedRobotPose>> visionEst = new ArrayList<>();
        visionEst.add(Optional.empty());
        visionEst.add(Optional.empty());

        for(int i = 0; i < visionEst.size(); i++) {
            for(var change : cameras[i].getAllUnreadResults()) {
                visionEst.set(i, photonEstimators[i].update(change));
                updateEstimationStdDevs(visionEst.get(i), change.getTargets(), i);

                stdDevsIter = curStdDevs.get(i);
                visionEst.get(i).ifPresent(
                    est -> {
                        var estStdDevs = stdDevsIter;
                        estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                    }
                );
            }
        }

    }

    private void updateEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets, int index) {

        if(estimatedPose.isEmpty()) {
            curStdDevs.set(index, Vision.Constants.kSingleTagStdDevs); //singlestddevs
        } else {
            var estStdDevs = Vision.Constants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            for(var tgt : targets) {
                var tagPose = photonEstimators[index].getFieldTags().getTagPose(tgt.getFiducialId());
                if(tagPose.isEmpty()) continue;
                numTags++;
                avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if(numTags == 0) {
                curStdDevs.set(index, Vision.Constants.kSingleTagStdDevs);
            } else {
                avgDist /= numTags;
                if(numTags > 1) estStdDevs = Vision.Constants.kMultiTagStdDevs;
                if(numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else 
                    estStdDevs = estStdDevs.times(1 + (avgDist * (avgDist / 30)));
                curStdDevs.set(index, estStdDevs);
            }
        }
    }

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }
}
