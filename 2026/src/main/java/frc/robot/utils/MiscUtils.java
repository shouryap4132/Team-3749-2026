package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import java.io.ObjectInputFilter.Config;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.RobotType;

/**
 * @author FRC 3749
 * @author Neel Adem
 */
public class MiscUtils {

    /**
     * Internal storage class for PID and feedforward control constants.
     * Used by {@link ControlConfigBuilder} to store separate configurations for real and
     * simulated robots.
     * 
     * <p>
     * Contains standard control constants:
     * <ul>
     * <li>kP, kI, kD - PID gains</li>
     * <li>kG - Gravity compensation</li>
     * <li>kS - Static friction compensation</li>
     * <li>kV - Velocity feedforward</li>
     * <li>kA - Acceleration feedforward</li>
     * </ul>
     * </p>
     */
    public static class ControlConfig {
        /** Proportional gain for PID control */
        public double kP;
        /** Integral gain for PID control */
        public double kI;
        /** Derivative gain for PID control */
        public double kD;
        /** Gravity compensation constant */
        public double kG;
        /** Static friction compensation constant */
        public double kS;
        /** Velocity feedforward constant */
        public double kV;
        /** Acceleration feedforward constant */
        public double kA;

        /** Sets the proportional gain. Returns this for method chaining. */
        public ControlConfig kP(double val) {
            kP = val;
            return this;
        }

        /** Sets the integral gain. Returns this for method chaining. */
        public ControlConfig kI(double val) {
            kI = val;
            return this;
        }

        /** Sets the derivative gain. Returns this for method chaining. */
        public ControlConfig kD(double val) {
            kD = val;
            return this;
        }

        /** Sets the gravity compensation. Returns this for method chaining. */
        public ControlConfig kG(double val) {
            kG = val;
            return this;
        }

        /** Sets the static friction compensation. Returns this for method chaining. */
        public ControlConfig kS(double val) {
            kS = val;
            return this;
        }

        /** Sets the velocity feedforward. Returns this for method chaining. */
        public ControlConfig kV(double val) {
            kV = val;
            return this;
        }

        /** Sets the acceleration feedforward. Returns this for method chaining. */
        public ControlConfig kA(double val) {
            kA = val;
            return this;
        }
    }

    /**
     * A fluent builder for configuring PID and feedforward constants with separate
     * values for real and simulated robots.
     * 
     * <p>
     * This class simplifies the process of defining control configurations that
     * differ between real hardware and simulation. Use {@link #get()} to
     * automatically retrieve the correct config based on the current robot type.
     * </p>
     * 
     * <h3>Usage Examples:</h3>
     * 
     * <pre>{@code
     * // Different values for real and sim (real, sim)
     * ControlConfig config = new ControlConfig()
     *     .kP(1.0, 0.8)
     *     .kI(0.0, 0.0)
     *     .kD(0.1, 0.05)
     *     .kV(0.5, 0.4);
     * 
     * // Same value for both real and sim
     * ControlConfig config = new ControlConfig()
     *     .kP(1.0)
     *     .kD(0.1);
     * 
     * // Mix and match as needed
     * ControlConfig config = new ControlConfig()
     *     .kP(1.0, 0.8)  // different values
     *     .kI(0.0)       // same value for both
     *     .kD(0.1, 0.05);
     * 
     * // Get the appropriate config for current robot type
     * double kP = config.get().kP;
     * }</pre>
     */
    public static class ControlConfigBuilder {
        private ControlConfig realConfig = new ControlConfig();
        private ControlConfig simConfig = new ControlConfig();

        /**
         * @return the internal config for the real robot (for direct access/modification)
         */
        public ControlConfig real() {
            return realConfig;
        }

        /**
         * @return the internal config for simulation (for direct access/modification)
         */
        public ControlConfig sim() {
            return simConfig;
        }

        /**
         * Automatically returns the appropriate config based on the current robot type.
         * 
         * @return realConfig if running on real robot, simConfig otherwise
         */
        public ControlConfig get() {
            if (MiscUtils.getRobotType() == RobotType.REAL) {
                return realConfig;
            } else {
                return simConfig;
            }
        }

        // ==================== Fluent Builder Methods (real, sim) ====================

        /**
         * Sets the proportional gain for both real and sim configs.
         * 
         * @param realVal kP value for real robot
         * @param simVal  kP value for simulation
         * @return this ControlConfig for method chaining
         */
        public ControlConfigBuilder kP(double realVal, double simVal) {
            realConfig.kP(realVal);
            simConfig.kP(simVal);
            return this;
        }

        /**
         * Sets the integral gain for both real and sim configs.
         * 
         * @param realVal kI value for real robot
         * @param simVal  kI value for simulation
         * @return this ControlConfig for method chaining
         */
        public ControlConfigBuilder kI(double realVal, double simVal) {
            realConfig.kI(realVal);
            simConfig.kI(simVal);
            return this;
        }

        /**
         * Sets the derivative gain for both real and sim configs.
         * 
         * @param realVal kD value for real robot
         * @param simVal  kD value for simulation
         * @return this ControlConfig for method chaining
         */
        public ControlConfigBuilder kD(double realVal, double simVal) {
            realConfig.kD(realVal);
            simConfig.kD(simVal);
            return this;
        }

        /**
         * Sets the gravity compensation for both real and sim configs.
         * 
         * @param realVal kG value for real robot
         * @param simVal  kG value for simulation
         * @return this ControlConfig for method chaining
         */
        public ControlConfigBuilder kG(double realVal, double simVal) {
            realConfig.kG(realVal);
            simConfig.kG(simVal);
            return this;
        }

        /**
         * Sets the static friction compensation for both real and sim configs.
         * 
         * @param realVal kS value for real robot
         * @param simVal  kS value for simulation
         * @return this ControlConfig for method chaining
         */
        public ControlConfigBuilder kS(double realVal, double simVal) {
            realConfig.kS(realVal);
            simConfig.kS(simVal);
            return this;
        }

        /**
         * Sets the velocity feedforward for both real and sim configs.
         * 
         * @param realVal kV value for real robot
         * @param simVal  kV value for simulation
         * @return this ControlConfig for method chaining
         */
        public ControlConfigBuilder kV(double realVal, double simVal) {
            realConfig.kV(realVal);
            simConfig.kV(simVal);
            return this;
        }

        /**
         * Sets the acceleration feedforward for both real and sim configs.
         * 
         * @param realVal kA value for real robot
         * @param simVal  kA value for simulation
         * @return this ControlConfig for method chaining
         */
        public ControlConfigBuilder kA(double realVal, double simVal) {
            realConfig.kA(realVal);
            simConfig.kA(simVal);
            return this;
        }

        // ==================== Single Value Methods (same for both) ====================

        /**
         * Sets the proportional gain to the same value for both real and sim.
         * 
         * @param val kP value for both configs
         * @return this ControlConfig for method chaining
         */
        public ControlConfigBuilder kP(double val) {
            return kP(val, val);
        }

        /**
         * Sets the integral gain to the same value for both real and sim.
         * 
         * @param val kI value for both configs
         * @return this ControlConfig for method chaining
         */
        public ControlConfigBuilder kI(double val) {
            return kI(val, val);
        }

        /**
         * Sets the derivative gain to the same value for both real and sim.
         * 
         * @param val kD value for both configs
         * @return this ControlConfig for method chaining
         */
        public ControlConfigBuilder kD(double val) {
            return kD(val, val);
        }

        /**
         * Sets the gravity compensation to the same value for both real and sim.
         * 
         * @param val kG value for both configs
         * @return this ControlConfig for method chaining
         */
        public ControlConfigBuilder kG(double val) {
            return kG(val, val);
        }

        /**
         * Sets the static friction compensation to the same value for both real and sim.
         * 
         * @param val kS value for both configs
         * @return this ControlConfig for method chaining
         */
        public ControlConfigBuilder kS(double val) {
            return kS(val, val);
        }

        /**
         * Sets the velocity feedforward to the same value for both real and sim.
         * 
         * @param val kV value for both configs
         * @return this ControlConfig for method chaining
         */
        public ControlConfigBuilder kV(double val) {
            return kV(val, val);
        }

        /**
         * Sets the acceleration feedforward to the same value for both real and sim.
         * 
         * @param val kA value for both configs
         * @return this ControlConfig for method chaining
         */
        public ControlConfigBuilder kA(double val) {
            return kA(val, val);
        }
    }

    public static Rotation2d normalize(Rotation2d rotation) {
        double currentRads = rotation.getRadians();
        return Rotation2d.fromRadians(MathUtil.angleModulus(currentRads));
    }

    /***
     * @param <T>   any numeric type (e.g., Double, Integer). Will automatically be
     *              inferred.
     * @param input the value to flip
     * @return the flipped value
     */
    public static <T extends Number> double fieldFlipY(T input) {
        double value = input.doubleValue();
        double height = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getFieldWidth();

        System.out.println("FIELD FLIP Y IS USING 2025 FIELD DIMENSIONS. UPDATE IF NEEDED.");

        return height - value;
    }

    /***
     * Clamps the input voltage to the nominal bus voltage defined in RobotConfig.
     * 
     * @param <T>   any numeric type (e.g., Double, Integer). Will automatically be
     *              inferred.
     * @param input the voltage to clamp
     * @return the clamped voltage
     */
    public static <T extends Number> double voltageClamp(T input) {
        double value = input.doubleValue();

        if (Math.abs(value) > RobotConfig.General.NOMINAL_BUS_VOLTAGE) {
            return Math.copySign(RobotConfig.General.NOMINAL_BUS_VOLTAGE, value);
        }

        return value;
    }

    /***
     * @param <T>    any numeric type (e.g., Double, Integer). Will automatically be
     *               inferred.
     * @param margin how close the values need to be to return true. Use a positive
     *               number
     * @param a      the first number
     * @param b      the second number
     * @return true if it is within the margin, false if not
     */
    public static <T extends Number> boolean withinMargin(T margin, T a, T b) {
        double marginDouble = margin.doubleValue();
        double aDouble = a.doubleValue();
        double bDouble = b.doubleValue();

        return aDouble + marginDouble >= bDouble && aDouble - marginDouble <= bDouble;
    }

    public static boolean isStopped(LinearVelocity velocity) {
        return withinMargin(RobotConfig.Accuracy.DEFAULT_MOVEMENT_TOLERANCE.in(MetersPerSecond),
                velocity.in(MetersPerSecond),
                0.0);
    }

    /**
     * @param <T>      any numeric type (e.g., Double, Integer). Will automatically
     *                 be inferred.
     * @param velocity
     * @return whether or not the velocity is below the default min speed, which we
     *         consider to be
     *         stopped
     */
    public static <T extends Number> boolean isStopped(T velocity) {
        return isStopped(velocity, RobotConfig.Accuracy.DEFAULT_MOVEMENT_TOLERANCE.in(MetersPerSecond));
    }

    /**
     * @param <T>      any numeric type (e.g., Double, Integer). Will automatically
     *                 be inferred.
     * @param velocity
     * @param minSpeed
     * @return whether or not the velocity is below the minimum speed to be
     *         considered stopped
     */
    public static <T extends Number> boolean isStopped(T velocity, T minSpeed) {
        return withinMargin(minSpeed, velocity, 0.0);
    }

    /**
     * @param <T>         any numeric type (e.g., Double, Integer). Will
     *                    automatically be inferred.
     * @param measurement
     * @param deadband
     * @return whether or not the velocity is below the minimum speed to be
     *         considered stopped
     */
    public static <T extends Number> double signedDeadband(T measurement, T deadband) {
        double measurementDouble = measurement.doubleValue();
        double deadbandDouble = deadband.doubleValue();

        if (Math.abs(measurementDouble) < deadbandDouble) {
            return 0.0;
        }
        return measurementDouble;
    }

    public static RobotType getRobotType() {
        if (Robot.isReal()) {
            return RobotType.REAL;
        } else if (Robot.isSimulation()) {
            return RobotType.SIM;
        } else {
            throw new IllegalStateException("[MiscUtils.java] Unknown Robot Type. Something has gone HORRIBLY wrong.");
        }
    }

    public static Alliance getAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Red);
    }

    public static boolean isSimulation() {
        return getRobotType() == RobotType.SIM;
    }

    public static boolean isReal() {
        return getRobotType() == RobotType.REAL;
    }

    public static boolean isReplay() {
        return RobotConfig.REPLAY_MODE && isSimulation();
    }

    public static boolean isBlueAlliance() {
        return getAlliance() == Alliance.Blue;
    }

    public static boolean isRedAlliance() {
        return getAlliance() == Alliance.Red;
    }
}
