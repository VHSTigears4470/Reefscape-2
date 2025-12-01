package frc.robot.Constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Cameras {
    public static final class Constants {
        //flip signs?
        public static final Transform2d[] CAMERA_TO_ROBOT =
        {new Transform2d(new Translation2d(Units.inchesToMeters(16), Units.inchesToMeters(-8)), new Rotation2d()),
         new Transform2d(new Translation2d(Units.inchesToMeters(16), Units.inchesToMeters(8)), new Rotation2d())
        };
    }
}
