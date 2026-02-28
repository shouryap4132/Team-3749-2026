package frc.robot.config;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.utils.MiscUtils.ControlConfigBuilder;
import edu.wpi.first.units.Units;


public class RollerConfig {

    /**
     * Enum for different roller states
     */
    public enum RollerStates {
        INTAKE(RadiansPerSecond.of(70)),
        OUTTAKE(RadiansPerSecond.of(-70)),
        RUN_HOPPER(RadiansPerSecond.of(50)),
        HIGH_SHOOTER_SPEED(RadiansPerSecond.of(200)),
        MEDIUM_SHOOTER_SPEED(RadiansPerSecond.of(150)),
        LOW_SHOOTER_SPEED(RadiansPerSecond.of(100)),
        REVERSE_HOPPER(RadiansPerSecond.of(-50)),
        STOP(RadiansPerSecond.of(0));

        // Radians / sec
        public final AngularVelocity rollerVelocity;

        private RollerStates(AngularVelocity velocity) {
            rollerVelocity = velocity;
        }
    }

    /**
     * Base class for roller configurations
     */
    public static abstract class RollerConfigBase {
        public final int[] canIds;
        public final double momentOfInertia;
        public final double gearRatio;
        public final double measurementNoise;
        public final boolean inverted;
        public final ControlConfigBuilder CONTROL_CONFIG;
        public final IdleMode idleMode;
        public final AngularVelocity velocityTolerance;

        RollerConfigBase(int[] canIds, double momentOfInertia, double gearRatio, double measurementNoise,
                boolean inverted,
                ControlConfigBuilder CONTROL_CONFIG, IdleMode idleMode, AngularVelocity velocityTolerance) {
            this.canIds = canIds;
            this.momentOfInertia = momentOfInertia;
            this.gearRatio = gearRatio;
            this.measurementNoise = measurementNoise;
            this.inverted = inverted;
            this.velocityTolerance = velocityTolerance;
            this.CONTROL_CONFIG = CONTROL_CONFIG;
            this.idleMode = idleMode;
        }
    }

    /**
     * Configuration for the intake rollers
     */
    public static final class IntakeRollerConfig extends RollerConfigBase {

        public static final ControlConfigBuilder CONTROL_CONFIG = new ControlConfigBuilder()
                .kG(0)
                .kP(0, 0.5)
                .kI(0)
                .kD(0)
                .kS(0.205, 0)
                .kV(0.0594, 12.0 / 201.575)
                .kA(0);

        public IntakeRollerConfig() {
            super(RobotConfig.CAN.INTAKE_ROLLER_MOTOR_IDS, 0.04, 3, 0.0, false, CONTROL_CONFIG, IdleMode.kBrake,
                    RobotConfig.Accuracy.Intake.ROLLER_VELOCITY_TOLERANCE);
        }
    }

    /**
     * Configuration for the hopper rollers
     */
    public static final class HopperRollerConfig extends RollerConfigBase {

        public static final ControlConfigBuilder CONTROL_CONFIG = new ControlConfigBuilder()
                .kG(0)
                .kP(0, 0.5)
                .kI(0)
                .kD(0)
                .kS(0.205, 0)
                .kV(0.0594, 12.0 / 201.575)
                .kA(0.04, 0);

        public HopperRollerConfig() {
            super(RobotConfig.CAN.HOPPER_ROLLER_MOTOR_IDS, 0.04, 3, 0.0, false, CONTROL_CONFIG, IdleMode.kBrake,
                    RobotConfig.Accuracy.Hopper.ROLLER_VELOCITY_TOLERANCE);
        }
    }

    /**
     * Configuration for the shooter rollers
     */
    public static final class ShooterRollerConfig extends RollerConfigBase {

        public static final ControlConfigBuilder CONTROL_CONFIG = new ControlConfigBuilder()
                .kG(0)
                .kP(0, 0.5)
                .kI(0)
                .kD(0)
                .kS(0.205, 0)
                .kV(0.0594, 12.0 / 201.575)
                .kA(0);
                
        public static final AngularVelocity maxVelocity =
            Units.DegreesPerSecond.of(0.1);        

        public ShooterRollerConfig() {
            super(RobotConfig.CAN.SHOOTER_ROLLER_MOTOR_IDS, 0.04, 3, 0.0, false, CONTROL_CONFIG, IdleMode.kCoast,
                    RobotConfig.Accuracy.Shooter.ROLLER_VELOCITY_TOLERANCE);
        }
    }

    /**
     * Enum for different roller implementations
     */
    public enum RollerImplementations {
        INTAKE(new IntakeRollerConfig()),
        HOPPER(new HopperRollerConfig()),
        SHOOTER(new HopperRollerConfig());

        public final RollerConfigBase config;

        private RollerImplementations(RollerConfigBase config) {
            this.config = config;
        }
    }

}
