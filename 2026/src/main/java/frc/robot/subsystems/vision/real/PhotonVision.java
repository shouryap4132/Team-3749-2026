package frc.robot.subsystems.vision.real;

import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.*;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionConstants.StandardDeviations;
import frc.robot.subsystems.vision.VisionIO;

public class PhotonVision implements VisionIO {

    private final PhotonCamera cam1 = new PhotonCamera("1");
    private final PhotonCamera cam2 = new PhotonCamera("2");
    private final PhotonCamera cam3 = new PhotonCamera("3");
    private final PhotonCamera cam4 = new PhotonCamera("4");

    private final PhotonCamera[] cameraList = { cam1, cam2, cam3, cam4 };
    private PhotonPoseEstimator poseEstimatorList[] = VisionConstants.CameraReal.poseEstimatorList;
    private final Double[] bestHubScore = new Double[VisionConstants.HubTags.hubTags.length];

    private VisionData visionData;

    private boolean disable3 = false;

    public PhotonVision(VisionData visionData) {
        int index = 0;
        for (PhotonPoseEstimator poseEstimator : poseEstimatorList) {
            poseEstimator.setPrimaryStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
            poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            poseEstimator.setTagModel(TargetModel.kAprilTag36h11);
            poseEstimator.setFieldTags(VisionConstants.aprilTagFieldLayout);
            index++;
        }

        this.visionData = visionData;
        for (int i = 0; i < VisionConstants.CameraReal.numCameras; i++) {
            logTarget(i);
        }
    }

    @Override
    public PhotonCamera getCamera(int index) {
        return cameraList[index];
    }

    @Override
    public void setDisable3(boolean disable) {
        disable3 = disable;
    }

    @Override
    public void updatePose() {
        visionData.targetsSeenIds.clear();
        Arrays.fill(bestHubScore, null);

        cameraUpdatePose(0);
        cameraUpdatePose(1);
        cameraUpdatePose(2);
        cameraUpdatePose(3);
    }

    public void cameraUpdatePose(int index) {

        // if (index == 1 || index == 2) {
        // poseEstimatorList[index].addHeadingData(Timer.getTimestamp(),
        // Robot.swerve.getRotation());
        // }
        poseEstimatorList[index].addHeadingData(Timer.getTimestamp(), Robot.swerve.getRotation());

        PhotonCamera camera = cameraList[index];
        PhotonPoseEstimator poseEstimator = poseEstimatorList[index];

        List<PhotonPipelineResult> pipelineResults = camera.getAllUnreadResults();

        for (PhotonPipelineResult pipelineResult : pipelineResults) {

            if (!pipelineResult.hasTargets()) {
                Logger.recordOutput("Vision/Cam" + (index + 1) + "/No targets", true);
                logBlank(index);
                continue;
            }

            double latencyMillis = pipelineResult.metadata.getLatencyMillis();

            visionData.latencyMillis[index] = latencyMillis;
            visionData.targetsSeen[index] = pipelineResult.getTargets().size();

            for (var target : pipelineResult.getTargets()) {
                visionData.targetsSeenIds.add(target.getFiducialId());
            }

            for (var target : pipelineResult.getTargets()) {
                for (int i = 0; i < VisionConstants.HubTags.hubTags.length; i++) {
                    if (target.getFiducialId() == VisionConstants.HubTags.hubTags[i]) {
                        Transform3d robotToTarget = VisionConstants.CameraPositions.cameraList[index]
                                .plus(target.bestCameraToTarget);
                        Transform3d targetToRobot = robotToTarget.inverse();
                        double distance = getHypotenuse(targetToRobot);
                        double ambiguity = target.getPoseAmbiguity();
                        double score = bestHubScore(distance, ambiguity, latencyMillis);
                        if (bestHubScore[i] == null || score < bestHubScore[i]) {
                            bestHubScore[i] = score;
                            visionData.visionEstimatedLocalPoses[i] = new Pose3d().transformBy(targetToRobot);

                            Logger.recordOutput("Vision/HubTag" + VisionConstants.HubTags.hubTags[i] + "/score", score);
                            Logger.recordOutput("Vision/HubTag" + VisionConstants.HubTags.hubTags[i] + "/distance",
                                    distance);
                            Logger.recordOutput("Vision/HubTag" + VisionConstants.HubTags.hubTags[i] + "/ambiguity",
                                    ambiguity);
                            Logger.recordOutput("Vision/HubTag" + VisionConstants.HubTags.hubTags[i] + "/latencyMs",
                                    latencyMillis);
                        }

                    }
                }
            }

            if (latencyMillis > VisionConstants.RejectionRequirements.maxLatencyMilliSec) {
                Logger.recordOutput("Vision/Cam" + (index + 1) + "/ High Latency", true);
                logBlank(index);

                continue;
            }

            if (pipelineResult.getTargets().size() == 1 &&
                    getHypotenuse(pipelineResult.getTargets().get(
                            0).bestCameraToTarget) > VisionConstants.RejectionRequirements.maxSingleTagDistanceMeters) {

                Logger.recordOutput("Vision/Cam" + (index + 1) + "/ single tag far", true);
                logBlank(index);

                continue;
            }

            visionData.distance[index] = getHypotenuse(pipelineResult.getTargets().get(0).bestCameraToTarget);

            if (pipelineResult.getBestTarget().poseAmbiguity > 0.2 && pipelineResult.getTargets().size() == 1) {
                Logger.recordOutput("Vision/Cam" + (index + 1) + "/ pose ambiguity rejection", true);
                logBlank(index);
            }

            poseEstimator.setReferencePose(getReferencePose());
            var optional_robotPose = poseEstimator.update(pipelineResult);

            if (optional_robotPose.isEmpty()) {
                Logger.recordOutput("Vision/Cam" + (index + 1) + "/ pose empty", true);
                logBlank(index);
                continue;
            }

            Pose3d robotPose = optional_robotPose.get().estimatedPose;
            Logger.recordOutput("Vision/Cam" + (index + 1) + "/pitch", robotPose.getRotation().getMeasureY());
            Logger.recordOutput("Vision/Cam" + (index + 1) + "/roll", robotPose.getRotation().getMeasureX());

            visionData.visionEstimatedPoses[index] = robotPose;

            updateStandardDeviations(pipelineResult, index);

            double timestamp = pipelineResult.getTimestampSeconds();
            Robot.swerve.visionUpdateOdometry(robotPose.toPose2d(), timestamp);
            logTarget(index);
        }
    }

    public void logTarget(int index) {
        Logger.recordOutput("Vision/Cam" + (index + 1) + "/latency", visionData.latencyMillis[index]);
        Logger.recordOutput("Vision/Cam" + (index + 1) + "/targetsSeen", visionData.targetsSeen[index]);
        Logger.recordOutput("Vision/Cam" + (index + 1) + "/pose", visionData.visionEstimatedPoses[index]);
        Logger.recordOutput("Vision/Cam" + (index + 1) + "/distance", visionData.distance[index]);
        Logger.recordOutput("Vision/Cam" + (index + 1) + "/pose ambiguity rejection", false);
        Logger.recordOutput("Vision/Cam" + (index + 1) + "/ No targets", false);
        Logger.recordOutput("Vision/Cam" + (index + 1) + "/High Latency", false);
        Logger.recordOutput("Vision/Cam" + (index + 1) + "/ pose empty", false);

        for (int i = 0; i < VisionConstants.HubTags.hubTags.length; i++) {
            Logger.recordOutput("Vision/HubTag" + VisionConstants.HubTags.hubTags[i] + "/local pose", visionData.visionEstimatedLocalPoses[i]);
        }
    }

    public void logBlank(int index) {
        Logger.recordOutput("Vision/Cam" + (index + 1) + "/latency", -1.0);
        Logger.recordOutput("Vision/Cam" + (index + 1) + "/targetsSeen", 1);
        Logger.recordOutput("Vision/Cam" + (index + 1) + "/pose", new Pose3d());
        Logger.recordOutput("Vision/Cam" + (index + 1) + "/local pose", new Pose3d());
    }

    public double getHypotenuse(Transform3d transform3d) {
        return Math.sqrt(
                Math.pow(transform3d.getX(), 2) + Math.pow(transform3d.getY(), 2) + Math.pow(transform3d.getZ(), 2));
    }

    public void updateStandardDeviations(PhotonPipelineResult result, int index) {
        SwerveDrivePoseEstimator poseEstimator = Robot.swerve.getPoseEstimator();

        if (result.getTargets().size() == 0) {
            return;
        }

        if (DriverStation.isDisabled()) {

            poseEstimator.setVisionMeasurementStdDevs(
                    VecBuilder.fill(StandardDeviations.PreMatch.xy, StandardDeviations.PreMatch.xy,
                            StandardDeviations.PreMatch.thetaRads));
        }

        if (result.getTargets().size() == 1) {
            double xyStdDev = StandardDeviations.OneTag.regression.apply(visionData.distance[index]);

            poseEstimator.setVisionMeasurementStdDevs(
                    VecBuilder.fill(xyStdDev, xyStdDev, StandardDeviations.OneTag.thetaRads));
        }

        if (result.getTargets().size() == 2) {
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(StandardDeviations.TwoTag.xy,
                    StandardDeviations.TwoTag.xy,
                    StandardDeviations.TwoTag.thetaRads));
        }

        if (result.getTargets().size() > 2) {
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(StandardDeviations.ManyTag.xy,
                    StandardDeviations.ManyTag.xy,
                    StandardDeviations.ManyTag.thetaRads));
        }
    }

    // lower score = better
    public double bestHubScore(double distance, double ambiguity, double latency) {
        // adjust weights later
        return distance + (ambiguity * 2.0) + (latency / 1000.0);
    }

    // I'm not sure what goes here yet
    @Override
    public void setStrategyCam12(PoseStrategy strat) {

    }
}
