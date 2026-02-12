package frc.robot.subsystems.ShooterHood;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.config.HoodedShooterConfig.*;
import frc.robot.config.MiscConfig.FieldSpecs;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.RobotType;
import frc.robot.subsystems.ShooterHood.ShooterHoodIO.ShooterHoodData;
import frc.robot.subsystems.ShooterHood.real.ShooterHoodReal;
import frc.robot.subsystems.ShooterHood.sim.ShooterHoodSim;
import frc.robot.utils.ShooterCalculator;
import frc.robot.utils.ShooterCalculator.ShooterTarget;
import frc.robot.utils.MiscUtils;

/**
 * Subsystem that controls the hood (angle) for the shooter assembly.
 *
 * <p>
 * Responsibilities:
 * <ul>
 * <li>Maintain the current hood state and target angle (latched during
 * AUTOAIM).</li>
 * <li>Compute control commands using a profiled PID controller plus arm
 * feedforward
 * and send voltages to the {@code HoodedShooterIO} implementation.</li>
 * <li>Expose helpers to query state, measured angle, and stability, and provide
 * a
 * static solver to compute firing angle/velocity pairs from a robot pose.</li>
 * </ul>
 *
 * <p>
 * Notes: interacts with the swerve drive pose (via
 * {@code Robot.swerve.getPose()}),
 * logs key outputs to the Logger, and updates an AKIT visual mechanism for
 * debugging.
 * See {@link HoodedShooterStates} and {@link ShooterHoodIO} for related
 * contracts.
 * 
 * @author Weston Gardner, Aaryav
 */
public class ShooterHood extends SubsystemBase {

    private final ShooterHoodIO io;
    private final ShooterHoodDataAutoLogged data = new ShooterHoodDataAutoLogged();

    private final static MiscUtils.ControlConfig config = HoodedShooterControl.CONTROL_CONFIG.get();
    private final ProfiledPIDController profile = new ProfiledPIDController(config.kP, config.kI, config.kD,
            new Constraints(HoodedShooterControl.MAX_VELOCITY.in(RadiansPerSecond),
                    HoodedShooterControl.MAX_ACCEL.in(RadiansPerSecondPerSecond)));
    private final ArmFeedforward feedforward = new ArmFeedforward(config.kS, config.kG, config.kV, config.kA);

    private HoodedShooterStates currentState = HoodedShooterStates.STOPPED;
    private double goalRadians = HoodedShooterSpecs.MIN_ANGLE.getRadians();

    private final LoggedMechanism2d mech2d = new LoggedMechanism2d(3, 5);
    private final LoggedMechanismRoot2d root = mech2d.getRoot("HoodedShooterRoot", 1.5, 0);
    private final LoggedMechanismLigament2d ligament = root.append(new LoggedMechanismLigament2d("Hooded Shooter",
            10, data.angle.getDegrees()));

    // Tunables
    public static LoggedNetworkNumber kG = new LoggedNetworkNumber("/Tuning/ShooterHood/kG", config.kG);
    public static LoggedNetworkNumber kP = new LoggedNetworkNumber("/Tuning/ShooterHood/kP", config.kP);
    public static LoggedNetworkNumber kI = new LoggedNetworkNumber("/Tuning/ShooterHood/kI", config.kI);
    public static LoggedNetworkNumber kD = new LoggedNetworkNumber("/Tuning/ShooterHood/kD", config.kD);
    public static LoggedNetworkNumber kS = new LoggedNetworkNumber("/Tuning/ShooterHood/kS", config.kS);
    public static LoggedNetworkNumber kV = new LoggedNetworkNumber("/Tuning/ShooterHood/kV", config.kV);
    public static LoggedNetworkNumber kA = new LoggedNetworkNumber("/Tuning/ShooterHood/kA", config.kA);
    public static LoggedNetworkNumber maxVelocity = new LoggedNetworkNumber("/Tuning/ShooterHood/Max Velocity",
            HoodedShooterControl.MAX_VELOCITY.in(RadiansPerSecond));
    public static LoggedNetworkNumber maxAcceleration = new LoggedNetworkNumber("/Tuning/ShooterHood/Max Acceleration",
            HoodedShooterControl.MAX_ACCEL.in(RadiansPerSecondPerSecond));

    /**
     * ShooterHood subsystem constructor
     */
    public ShooterHood() {
        if (MiscUtils.getRobotType() == RobotType.REAL) {
            io = new ShooterHoodReal(data);
        } else {
            io = new ShooterHoodSim(data);
        }

        profile.setTolerance(RobotConfig.Accuracy.Shooter.HOOD_ANGLE_TOLERANCE.in(Radians));
    }

    // Getters ///////////////////////////////////////////////////////////

    /**
     * @return the current hooded shooter state.
     */
    public HoodedShooterStates getState() {
        return currentState;
    }

    /**
     * @return the latest IO data snapshot for the hooded shooter.
     */
    public ShooterHoodData getData() {
        return data;
    }

    /**
     * @return the current measured hood angle.
     */
    public Rotation2d getAngle() {
        return data.angle;
    }

    /*
     * SETTERS
     */

    /**
     * Set the desired hooded shooter state. Null or STOPPED is not allowed.
     */
    public void setState(HoodedShooterStates state) {
        if (state == null || state == HoodedShooterStates.STOPPED) {
            Logger.recordOutput("Errors/ShooterHood", "Attempted to set hooded shooter state to 'null' or 'STOPPED'");
            return;
        }

        currentState = state;

        profile.reset(getAngle().getRadians());
        profile.setGoal(currentState.angle.getRadians());
    }

    // Misc //////////////////////////////////////////////////////////////

    /**
     * Immediately stop the hood: set state to STOPPED and command zero voltage.
     */
    public void stop() {
        currentState = HoodedShooterStates.STOPPED;
        io.setVoltage(0);
    }

    /**
     * True when the hood is within the position tolerance and is not moving.
     */
    public boolean isStableState() {
        double goalPos = profile.getGoal().position;
        boolean withinTolerance = MiscUtils.withinMargin(
                RobotConfig.Accuracy.Shooter.HOOD_ANGLE_TOLERANCE.in(Radians),
                goalPos,
                getAngle().getRadians());

        return withinTolerance && isStopped();
    }

    /**
     * True when the hood's angular velocity is near zero.
     */
    public boolean isStopped() {
        return MiscUtils.isStopped(data.velocity.in(RadiansPerSecond),
                RobotConfig.Accuracy.Shooter.HOOD_VELOCITY_TOLERANCE.in(RadiansPerSecond));
    }

    /**
     * Calculate the auto-aim angle for shooting at the hub.
     * 
     * @return the goal angle in radians, or the current stow angle if no solution
     *         found
     */
    public double autoAim() {
        Translation2d hubPosition;
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (alliance == Alliance.Red) {
            hubPosition = FieldSpecs.HUB_POSITION_RED_ALLIANCE;
        } else {
            hubPosition = FieldSpecs.HUB_POSITION_BLUE_ALLIANCE;
        }

        return ShooterCalculator.calculateSolution(hubPosition, FieldSpecs.HUB_HEIGHT_FROM_GROUND_METERS)
                .map(ShooterTarget::angle)
                .map(Rotation2d::getRadians)
                .orElseGet(() -> {
                    setState(HoodedShooterStates.STOW);
                    return currentState.angle.getRadians();
                });
    }

    /**
     * Calculate the angle for passing to a teammate.
     * 
     * @return the goal angle in radians, or the current stow angle if no solution
     *         found
     */
    public double pass() {
        Translation2d target;
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        if (alliance == Alliance.Red) {
            if (Robot.swerve.getPose().getTranslation().getY() < FieldSpecs.FIELD_CENTER_Y) {
                target = FieldSpecs.LOWER_RED_ALLIANCE_CENTER;
            } else {
                target = FieldSpecs.UPPER_RED_ALLIANCE_CENTER;
            }
        } else {
            if (Robot.swerve.getPose().getTranslation().getY() < FieldSpecs.FIELD_CENTER_Y) {
                target = FieldSpecs.LOWER_BLUE_ALLIANCE_CENTER;
            } else {
                target = FieldSpecs.UPPER_BLUE_ALLIANCE_CENTER;
            }
        }

        return ShooterCalculator.calculateSolution(target, FieldSpecs.HUB_HEIGHT_FROM_GROUND_METERS)
                .map(ShooterTarget::angle)
                .map(Rotation2d::getRadians)
                .orElseGet(() -> {
                    setState(HoodedShooterStates.STOW);
                    return currentState.angle.getRadians();
                });
    }

    public void refreshTuneables() {
        feedforward.setKa(kA.get());
        feedforward.setKg(kG.get());
        feedforward.setKs(kS.get());
        feedforward.setKv(kV.get());
        profile.setPID(kP.get(), kI.get(), kD.get());
        profile.setConstraints(new Constraints(maxVelocity.get(), maxAcceleration.get()));
    }

    /**
     * Move the hood toward the current goal angle depending on state
     * (STOPPED/AUTOAIM/other).
     * Uses the profiled PID + arm feedforward to compute voltage and commands it to
     * the IO.
     */
    public void moveToGoal() {
        if (isStableState()) {
            Logger.recordOutput("ShooterHood/DesiredVolts", 0);
            io.setVoltage(0);
            return;
        }

        if (currentState == HoodedShooterStates.AUTOAIM) {
            goalRadians = autoAim();
            profile.setGoal(goalRadians);
        } else if (currentState == HoodedShooterStates.PASS) {
            goalRadians = pass();
            profile.setGoal(goalRadians);
        }

        double pidOutput = profile.calculate(getAngle().getRadians());
        State setpoint = profile.getSetpoint();
        double feedforwardOutput = feedforward.calculate(setpoint.position, setpoint.velocity);

        double voltageOutput = pidOutput + feedforwardOutput;

        Logger.recordOutput("ShooterHood/DesiredVolts", voltageOutput);
        io.setVoltage(voltageOutput);
    }

    public void logData() {
        Logger.processInputs("ShooterHood", data);

        Logger.recordOutput("ShooterHood/State", currentState.toString());
        Logger.recordOutput("ShooterHood/TargetAngle", currentState.angle);
        Logger.recordOutput("ShooterHood/Angle", getAngle());
        Logger.recordOutput("ShooterHood/Velocity", data.velocity);
    }

    /**
     * Update the visualization mechanism to the current measured angle.
     */
    public void updateMechanism() {
        ligament.setAngle(data.angle);
        Logger.recordOutput("ShooterHood/Mechanism", mech2d);
    }

    // I just have to change it to the trech position
    public void autoStow() {
        if (FieldSpecs.isNearAnyTrench(Robot.swerve.getPose())) {
            setState(HoodedShooterStates.STOW);
        }
    }

    /**
     * Periodic update: refresh IO, log data, and drive the hood toward the goal.
     */
    @Override
    public void periodic() {
        refreshTuneables();

        io.updateData();

        logData();
        updateMechanism();

        moveToGoal();
    }
}
