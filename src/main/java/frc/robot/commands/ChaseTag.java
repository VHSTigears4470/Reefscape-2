package frc.robot.commands;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Vision;
import frc.robot.subsystems.DriveSubsystem;

public class ChaseTag extends Command{
    //Update as needed
    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

    private static final int TAG_TO_CHASE = 2; //Swap as needed
    private static final Transform2d TAG_TO_GOAL = new Transform2d(new Translation2d(1,0), Rotation2d.fromDegrees(180));

    private final PhotonCamera[] cameras = {new PhotonCamera("camera1"), new PhotonCamera("camera2")};
    private final DriveSubsystem drivetrainSubsystem;
    private final Supplier<Pose2d> poseProvider;

    //Update as needed
    private final ProfiledPIDController xController = new ProfiledPIDController(.04, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(.04, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(1, 0, 0, OMEGA_CONSTRAINTS);

    private Pose2d[] goalPose;
    private PhotonTrackedTarget[] lastTarget;

    public ChaseTag(DriveSubsystem drivetrainSubsystem, Supplier<Pose2d> poseProvider) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.poseProvider = poseProvider;

        xController.setTolerance(0.1);
        yController.setTolerance(0.1);
        omegaController.setTolerance(Units.degreesToRadians(3));
        omegaController.enableContinuousInput(-1, 1);

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        goalPose = new Pose2d[]{null, null};
        lastTarget = new PhotonTrackedTarget[]{null, null};
        var robotPose = poseProvider.get();
        omegaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());        
    }

    @Override
    public void execute() {
        var robotPose = poseProvider.get();
        for(int i = 0; i < cameras.length; i++) {
            var photonRes = cameras[i].getLatestResult();
            if(photonRes.hasTargets()) {
                var targetOpt = photonRes.getTargets().stream()
                    .filter(t -> t.getFiducialId() == TAG_TO_CHASE)
                    .findFirst();
                if(targetOpt.isPresent()) {
                    var target = targetOpt.get();
                    if(!target.equals(lastTarget[i])) {
                        lastTarget[i] = target;
                        var camToTarget = target.getBestCameraToTarget();
                        var transform = new Transform2d(
                            camToTarget.getTranslation().toTranslation2d(),
                            camToTarget.getRotation().toRotation2d().minus(Rotation2d.fromDegrees(90)));
                        //var cameraPose = robotPose.transformBy(Vision.Constants.CAMERA_TO_ROBOT[i]); 
                        //Pose2d targetPose = cameraPose.transformBy(transform);
                        //goalPose[i] = targetPose.transformBy(TAG_TO_GOAL);
                    }
                    int numGoals = 0;
                    double xGoal = 0;
                    double yGoal = 0;
                    double omegaGoal = 0;
                    for(Pose2d pose : goalPose) {
                        if(pose != null) {
                            xGoal += pose.getX();
                            yGoal += pose.getY();
                            omegaGoal += pose.getRotation().getRadians();
                            numGoals++;
                        }
                    }
                    if (numGoals > 0) {
                        xController.setGoal(xGoal / numGoals);
                        yController.setGoal(yGoal / numGoals);
                        omegaController.setGoal(omegaGoal / numGoals);
                    }
                }
            }
        }
        var xSpeed = (xController.atGoal()) ? 0 : xController.calculate(robotPose.getX());
        var ySpeed = (yController.atGoal()) ? 0 : yController.calculate(robotPose.getY());
        var omegaSpeed = (omegaController.atGoal()) ? 0 : omegaController.calculate(robotPose.getRotation().getRadians());
        drivetrainSubsystem.driveRobotRelative(
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation())
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.stopModules();
    }

}
