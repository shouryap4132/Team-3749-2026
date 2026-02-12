package frc.robot.config;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import frc.robot.utils.MiscUtils;

public class ClimbConfig {
    public static class ClimbSpecs {
        public static final double GEARING = 45; // 16:1 stage followed by another 16:1 stage
        public static final Mass CARRIAGE_MASS = Pounds.of(54);
        // 22 TOOTH, 1/4 in pitch, divide by 2pi to go from circumfrence to radius
        public static final Distance DRUM_RADIUS = Inches.of(4);

        public static final Distance MIN_HEIGHT = Meters.of(0);
        public static final Distance MAX_HEIGHT = Inches.of(30);
        public static final Distance STARTING_HEIGHT = Meters.of(0);

        public static final boolean SIMULATE_GRAVITY = true;
        public static final boolean LEFT_MOTOR_INVERTED = true;
        public static final boolean RIGHT_MOTOR_INVERTED = false;

    }

    public static class ClimbControl {
        public static final MiscUtils.ControlConfigBuilder CONTROL_CONFIG = new MiscUtils.ControlConfigBuilder()
                .kG(0.27, 1.25068)
                .kP(0, 10)
                .kI(0)
                .kD(0)
                .kS(0.0, 0.16)
                .kV(9.3, 7.77)
                .kA(0.2, 0.27);

        public static final LinearVelocity MAX_VELOCITY = MetersPerSecond.of(1.415);
        public static final LinearAcceleration MAX_ACCEL = MetersPerSecondPerSecond.of(4.1);
    }

    /***
     * states for the different possible heights the elevator would need to go to
     */
    public enum ClimbStates {
        CLIMB(Inches.of(12)),
        MIDDLE(Inches.of(3)),
        MAX(ClimbSpecs.MAX_HEIGHT),
        MIN(ClimbSpecs.MIN_HEIGHT),
        STOW(ClimbSpecs.MIN_HEIGHT);

        public Distance height;

        private ClimbStates(Distance height) {

            this.height = height;
        }
    }
}
