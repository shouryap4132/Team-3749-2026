package frc.robot.subsystems.vision;

import java.util.function.Function;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {

    public class RejectionRequirements {
        public static final double maxLatencyMilliSec = 150; // not sure if this is the correct value for now
        public static final double maxSingleTagDistanceMeters = Units.inchesToMeters(80);
    }

    public class StandardDeviations {
        public class PreMatch {
            public static final double xy = 0.001;
            public static final double thetaRads = 0.0002;
        }

        public class OneTag {
            public static final double slope = 0.0021889;
            public static final double thetaRads = Units.degreesToRadians(7);
            public static final Function<Double, Double> regression = (distance) -> slope * distance;
        }

        public class TwoTag {
            public static final double xy = Math.hypot(0.005, 0.008);
            public static final double thetaRads = Units.degreesToRadians(2);
        }

        public class ManyTag {
            public static final double xy = Math.hypot(0.002, 0.003);
            public static final double thetaRads = Units.degreesToRadians(2);
        }

    }

    // I don't have the transform3d values yet so they are empty for now
    // make sure they are robot -> cam, not cam -> robot
    public class CameraPositions {
        public static Transform3d cam1 = new Transform3d(

        );
        public static Transform3d cam2 = new Transform3d(

        );
        public static Transform3d cam3 = new Transform3d(

        );
        public static Transform3d cam4 = new Transform3d(

        );
        public static Transform3d[] cameraList = { cam1, cam2, cam3, cam4 };

    }

    public class CameraReal {
        public static final int numCameras = 4;
        public static PhotonPoseEstimator poseEstimator1 = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, CameraPositions.cam1);
        public static PhotonPoseEstimator poseEstimator2 = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, CameraPositions.cam2);
        public static PhotonPoseEstimator poseEstimator3 = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, CameraPositions.cam3);
        public static PhotonPoseEstimator poseEstimator4 = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, CameraPositions.cam4);

        public static PhotonPoseEstimator[] poseEstimatorList = {
                poseEstimator1, poseEstimator2, poseEstimator3, poseEstimator4
        };
    }

    public class HubTags {
        public static int[] hubTags = { 9, 10, 25, 26 };
    }

    public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2026RebuiltWelded);
}
