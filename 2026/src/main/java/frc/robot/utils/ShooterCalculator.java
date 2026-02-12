package frc.robot.utils;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;
import frc.robot.config.HoodedShooterConfig.HoodedShooterSpecs;

/**
 * Compute a feasible shooter angle and flywheel velocity for a given robot
 * pose.
 * 
 * @return Optional containing the angle and the velocity found to work, empty
 *         if no solution is found
 */
public class ShooterCalculator {
    public record ShooterTarget(Rotation2d angle, LinearVelocity velocity) {
    }

    /**
     * Calculate the shooter angle and velocity needed to hit a target.
     * 
     * @param robotPose    the current robot pose
     * @param target       the target position (x, y)
     * @param targetHeight the height of the target in meters
     * @return Optional containing the ShooterTarget if a solution is found, empty
     *         otherwise
     */
    public static Optional<ShooterTarget> calculateSolution(Pose2d robotPose, Translation2d target,
            double targetHeight) {
        final double g = 9.81;

        Translation2d shooterPosition = robotPose.getTranslation().plus(HoodedShooterSpecs.MOUNT_OFFSET);

        double d = shooterPosition.getDistance(target) + HoodedShooterSpecs.DISTANCE_TUNING.in(Meters);

        double h1 = HoodedShooterSpecs.SHOOTER_HEIGHT_FROM_GROUND.in(Meters);
        double h2 = targetHeight;

        double[] velocities = {
                HoodedShooterSpecs.LOW_SPEED_OUTPUT_VELOCITY.in(MetersPerSecond),
                HoodedShooterSpecs.MEDIUM_SPEED_OUTPUT_VELOCITY.in(MetersPerSecond),
                HoodedShooterSpecs.HIGH_SPEED_OUTPUT_VELOCITY.in(MetersPerSecond)
        };

        for (double v : velocities) {
            double h = h2 - h1;

            double discriminant = Math.pow(v, 4)
                    - g * (g * d * d + 2 * h * v * v);

            if (discriminant < 0)
                continue;

            double sqrt = Math.sqrt(discriminant);

            double angleRad = Math.atan((v * v - sqrt) / (g * d));

            // Convert the ballistic launch angle (angle above horizontal) into the
            // mechanism's coordinate frame used by the hood. The hood's Rotation2d
            // values are specified around ~133..160 degrees, so we convert the
            // launch angle into the mechanism angle. Geometrically this is:
            // mechanismAngle = PI - launchAngle
            double mechanismAngle = Math.PI - angleRad;

            // Use mechanismAngle for bounds checking against the hood's limits.
            if (mechanismAngle < HoodedShooterSpecs.MIN_ANGLE.getRadians()
                    || mechanismAngle > HoodedShooterSpecs.MAX_ANGLE.getRadians()) {
                continue;
            }

            LinearVelocity velocity = MetersPerSecond.of(v);
            // Store the mechanism-frame angle (radians) so callers get an angle
            // compatible with HoodedShooter's expected coordinates.
            Rotation2d angle = new Rotation2d(mechanismAngle);

            return Optional.of(new ShooterTarget(angle, velocity));
        }

        return Optional.empty();
    }

    /**
     * Calculate the shooter angle and velocity using the current robot pose from
     * swerve.
     * 
     * @param target       the target position (x, y)
     * @param targetHeight the height of the target in meters
     * @return Optional containing the ShooterTarget if a solution is found, empty
     *         otherwise
     */
    public static Optional<ShooterTarget> calculateSolution(Translation2d target, double targetHeight) {
        return calculateSolution(Robot.swerve.getPose(), target, targetHeight);
    }
}
