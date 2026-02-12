package frc.robot.config;

import java.util.List;
import java.util.NoSuchElementException;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class MiscConfig {
    public static class FieldSpecs {
        public static final double HUB_HEIGHT_FROM_GROUND_METERS = Units.inchesToMeters(72);

        public static final Translation2d HUB_POSITION_BLUE_ALLIANCE =
                new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84));
        public static final Translation2d HUB_POSITION_RED_ALLIANCE =
                new Translation2d(Units.inchesToMeters(469.11), Units.inchesToMeters(158.84));

        public static final Double TRENCH_LOWER_Y = Units.inchesToMeters(49.84);
        public static final Double TRENCH_UPPER_Y = Units.inchesToMeters(267.85);
        public static final Double TRENCH_LEFT_BLUE_X = Units.inchesToMeters(156.61);
        public static final Double TRENCH_RIGHT_BLUE_X = Units.inchesToMeters(207.61);
        public static final Double TRENCH_LEFT_RED_X = Units.inchesToMeters(443.61);
        public static final Double TRENCH_RIGHT_RED_X = Units.inchesToMeters(494.61);


        public static final double TRENCH_TOLERANCE_X = Units.inchesToMeters(10);
        public static final double TRENCH_TOLERANCE_Y = Units.inchesToMeters(10);

        public static final Translation2d LOWER_BLUE_ALLIANCE_CENTER = 
            new Translation2d(Units.inchesToMeters(91.05), Units.inchesToMeters(50));
        public static final Translation2d UPPER_BLUE_ALLIANCE_CENTER = 
            new Translation2d(Units.inchesToMeters(91.05), Units.inchesToMeters(267.69));
        
        public static final Translation2d LOWER_RED_ALLIANCE_CENTER = 
            new Translation2d(Units.inchesToMeters(560.15), Units.inchesToMeters(50));
        public static final Translation2d UPPER_RED_ALLIANCE_CENTER = 
            new Translation2d(Units.inchesToMeters(560.15), Units.inchesToMeters(267.69));

        public static final double FIELD_CENTER_X = Units.inchesToMeters(325.61);
        public static final double FIELD_CENTER_Y = Units.inchesToMeters(158.84);

        public static boolean isNearAnyTrench(Pose2d robotPose) {
            Translation2d robot = robotPose.getTranslation();

            boolean x = (robot.getX() > (TRENCH_LEFT_BLUE_X - TRENCH_TOLERANCE_X) && robot.getX() < (TRENCH_RIGHT_BLUE_X + TRENCH_TOLERANCE_X)) ||
                        (robot.getX() > (TRENCH_LEFT_RED_X - TRENCH_TOLERANCE_X) && robot.getX() < (TRENCH_RIGHT_RED_X + TRENCH_TOLERANCE_X));

            boolean y = (robot.getY() < (TRENCH_LOWER_Y + TRENCH_TOLERANCE_Y) || robot.getY() > (TRENCH_UPPER_Y - TRENCH_TOLERANCE_Y));

            return x && y;
        }
    }
}
