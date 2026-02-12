package frc.robot.config;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.MiscUtils;

public class HoodedShooterConfig {
    public static class HoodedShooterSpecs {
        public static final double GEARING = 45;
        public static final Mass END_EFFECTOR_MASS = Pounds.of(1.19);
        public static final Distance HOODED_SHOOTER_LENGTH = Inches.of(7.785);
        public static final double MOMENT_OF_INERTIA_KG_M2 = 0.0008194;

        public static final Distance SHOOTER_HEIGHT_FROM_GROUND = Inches.of(20.153);

        public static final LinearVelocity LOW_SPEED_OUTPUT_VELOCITY = MetersPerSecond.of(5);
        public static final LinearVelocity MEDIUM_SPEED_OUTPUT_VELOCITY = MetersPerSecond.of(10);
        public static final LinearVelocity HIGH_SPEED_OUTPUT_VELOCITY = MetersPerSecond.of(15);

        public static final Distance DISTANCE_TUNING = Inches.of(0);

        public static final Translation2d MOUNT_OFFSET = new Translation2d(
                Inches.of(11.28).in(Meters),
                Inches.of(0).in(Meters));
        public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(160.13);
        public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(133.13);
        public static final Rotation2d TEST_ANGLE = Rotation2d.fromDegrees(145);
        public static final Rotation2d START_ANGLE = Rotation2d.fromDegrees(134);

        public static final boolean SIMULATE_GRAVITY = true;
        public static final boolean IS_INVERTED = false;
    }

    public static class HoodedShooterControl {
        public static final MiscUtils.ControlConfigBuilder CONTROL_CONFIG = new MiscUtils.ControlConfigBuilder()
                .kG(0, 0.006248)
                .kP(0, 20)
                .kI(0, 0.2)
                .kD(0, 1)
                .kS(0)
                .kV(0, 0.887)
                .kA(0);

        public static final AngularVelocity MAX_VELOCITY = RadiansPerSecond.of(767.76);
        public static final AngularAcceleration MAX_ACCEL = RadiansPerSecondPerSecond.of(10000);
    }

    public enum HoodedShooterStates {
        STOW(HoodedShooterSpecs.MIN_ANGLE),
        AUTOAIM(),
        PASS(),
        MAX(HoodedShooterSpecs.MAX_ANGLE),
        MEDIUM(HoodedShooterSpecs.TEST_ANGLE),
        STOPPED();

        public Rotation2d angle;
        
        private HoodedShooterStates(Rotation2d s_angle) {
            this.angle = s_angle;
        }

        private HoodedShooterStates() {
            this.angle = null;
        }
    }
}
