package frc.robot.config;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.utils.MiscUtils;

public class IntakeArmConfig {
    public static class IntakeArmSpecs {
        public static final double GEARING = 48.0;
        public static final Distance INTAKE_ARM_LENGTH = Inches.of(8);
        public static final Mass INTAKE_ARM_MASS = Pounds.of(10);

        public static final double INTAKE_ARM_MOI = SingleJointedArmSim.estimateMOI(
                IntakeArmSpecs.INTAKE_ARM_LENGTH.in(Meters),
                IntakeArmSpecs.INTAKE_ARM_MASS.in(Kilograms));

        public static final Angle ABSOLUTE_ENCODER_OFFSET = Degrees.of(5);

        // placeholders until we know min and max angles
        public static final Angle MIN_ANGLE = Degrees.of(-99999999);
        public static final Angle MAX_ANGLE = Degrees.of(99999999);

        public static final boolean SIMULATE_GRAVITY = false;
        public static final boolean IS_INVERTED = false;
    }

    public static class IntakeArmControl {

        public static final AngularVelocity MAX_VELOCITY = RadiansPerSecond.of(14.0);
        public static final AngularAcceleration MAX_ACCEL = RadiansPerSecondPerSecond.of(180);

        public static final MiscUtils.ControlConfigBuilder CONTROL_CONFIG = new MiscUtils.ControlConfigBuilder()
                .kG(0)
                .kP(8, 0)
                .kI(0)
                .kD(0)
                .kS(0)
                .kV(12.0 / MAX_VELOCITY.in(RadiansPerSecond))
                .kA(12.0 / MAX_ACCEL.in(RadiansPerSecondPerSecond));
    }

    public enum IntakeArmStates {
        STOW(Degrees.of(90)),
        DEPLOYED(Degrees.of(-10));

        public final Angle angle;

        private IntakeArmStates(Angle s_angle) {
            this.angle = s_angle;
        }
    }
}