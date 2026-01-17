package frc.robot.Constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.struct.Struct;

import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision {
    public static final class Constants {
        //flip signs?
        public static final Transform3d[] CAMERA_TO_ROBOT = //Adjust
        {
            new Transform3d(new Translation3d(Units.inchesToMeters(16), Units.inchesToMeters(-8), Units.inchesToMeters(6)), 
                new Rotation3d(0, 3, 0)),
            new Transform3d(new Translation3d(Units.inchesToMeters(16), Units.inchesToMeters(8), Units.inchesToMeters(6)), 
                new Rotation3d(0, 3, 0))
        };
        public static final AprilTagFieldLayout m_targetPoses = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        //Adjust
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.1, 0.1, Units.radiansToDegrees(5));
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.1, 0.1, Units.radiansToDegrees(5));
    }

    public static class VisionIOInputs {
        public Pose3d cameraPoses[];
        public List<PhotonTrackedTarget>[] cameraTargets;
        public double timestamp = 0.0;
    }
}
