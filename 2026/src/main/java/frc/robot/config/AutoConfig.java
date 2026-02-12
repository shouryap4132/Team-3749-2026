package frc.robot.config;

import frc.robot.utils.MiscUtils.ControlConfigBuilder;

/**
 * Autonomous tuning values shared across commands. Separating these from the
 * command logic keeps auto routines declarative and easier to retune.
 */
public final class AutoConfig {
    public static final class Drive {
        public static final ControlConfigBuilder CONTROL_CONFIG = new ControlConfigBuilder()
                .kP(3.0)
                .kI(0.5)
                .kD(0.5);
    }

    public static final class Turn {
        public static final ControlConfigBuilder CONTROL_CONFIG = new ControlConfigBuilder()
                .kP(4.75)
                .kI(0)
                .kD(0);
    }
}
