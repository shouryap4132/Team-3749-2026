package frc.robot.subsystems.IntakeArm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.IntakeArmConfig.IntakeArmControl;
import frc.robot.config.IntakeArmConfig.IntakeArmStates;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.RobotType;
import frc.robot.subsystems.IntakeArm.IntakeArmIO.IntakeArmData;
import frc.robot.subsystems.IntakeArm.real.IntakeArmReal;
import frc.robot.subsystems.IntakeArm.sim.IntakeArmSim;
import frc.robot.utils.MiscUtils;
import frc.robot.utils.MiscUtils.ControlConfig;

/**
 * Single jointed arm to intake fuel from the ground. This subsystem does NOT
 * control the roller
 * for the intake, that is handled by the Roller subsystem.
 * 
 * @author Anish Gupta
 * @author Rigved Gaddam
 * @author Seonyoo Pak
 */

public class IntakeArm extends SubsystemBase {

    private final IntakeArmIO io;
    private final IntakeArmDataAutoLogged data = new IntakeArmDataAutoLogged();

    private final static ControlConfig config = IntakeArmControl.CONTROL_CONFIG.get();
    private final ProfiledPIDController profile = new ProfiledPIDController(config.kP, config.kI, config.kD,
            new Constraints(IntakeArmControl.MAX_VELOCITY.in(RadiansPerSecond),
                    IntakeArmControl.MAX_ACCEL.in(RadiansPerSecondPerSecond)));
    private final ArmFeedforward feedforward = new ArmFeedforward(config.kS, config.kG, config.kV, config.kA);

    private IntakeArmStates currentState = IntakeArmStates.STOW;

    private final LoggedMechanism2d mech = new LoggedMechanism2d(4, 4);
    public LoggedMechanismRoot2d root = mech.getRoot("IntakeRoot", 2.0, 1.5);
    private final LoggedMechanismLigament2d intakeLigament = root.append(new LoggedMechanismLigament2d("Intake", 1.5,
            currentState.angle.in(Degrees)));

    // Tunables
    public static LoggedNetworkNumber kG = new LoggedNetworkNumber("/Tuning/IntakeArm/kG", config.kG);
    public static LoggedNetworkNumber kP = new LoggedNetworkNumber("/Tuning/IntakeArm/kP", config.kP);
    public static LoggedNetworkNumber kI = new LoggedNetworkNumber("/Tuning/IntakeArm/kI", config.kI);
    public static LoggedNetworkNumber kD = new LoggedNetworkNumber("/Tuning/IntakeArm/kD", config.kD);
    public static LoggedNetworkNumber kS = new LoggedNetworkNumber("/Tuning/IntakeArm/kS", config.kS);
    public static LoggedNetworkNumber kV = new LoggedNetworkNumber("/Tuning/IntakeArm/kV", config.kV);
    public static LoggedNetworkNumber kA = new LoggedNetworkNumber("/Tuning/IntakeArm/kA", config.kA); // 1.72
    public static LoggedNetworkNumber maxVelocity = new LoggedNetworkNumber("/Tuning/IntakeArm/Max Velocity",
            IntakeArmControl.MAX_VELOCITY.in(RadiansPerSecond));
    public static LoggedNetworkNumber maxAcceleration = new LoggedNetworkNumber("/Tuning/IntakeArm/Max Acceleration",
            IntakeArmControl.MAX_ACCEL.in(RadiansPerSecondPerSecond));

    /**
     * Intake subsystem constructor
     */

    public IntakeArm() {
        if (MiscUtils.getRobotType() == RobotType.REAL) {
            io = new IntakeArmReal(data);
        } else {
            io = new IntakeArmSim(data);
        }
    }

    // Getters ///////////////////////////////////////////////////////////

    /**
     * Get the current state of the intake
     * 
     * @return An IntakeState obj
     */
    public IntakeArmStates getState() {
        return currentState;
    }

    /**
     * Get the current data of the intake
     * 
     * @return An IntakeData object, with angle, velocity, accel, appliedVolts,
     *         currentAmps
     */
    public IntakeArmData getData() {
        return data;
    }

    /**
     * Get the current angle of the intake
     * 
     * @return The angle of the intake
     */
    public Angle getAngle() {
        return data.angle;
    }

    /**
     * Get the current velocity of the intake
     * 
     * @return The velocity of the intake
     */
    public AngularVelocity getVelocity() {
        return data.velocity;
    }

    // Setters ///////////////////////////////////////////////////////////

    /**
     * Set the desired state of the intake.
     * 
     * @param state An IntakeState representing the desired state
     */
    public void setState(IntakeArmStates state) {
        // check for valid state
        if (state == null) {
            Logger.recordOutput("Errors/IntakeArm", "Attempted to set intake state to null");
            return;
        }

        currentState = state;

        profile.reset(
                getAngle().in(Radians),
                getVelocity().in(RadiansPerSecond));

        profile.setGoal(new State(currentState.angle.in(Radians), 0));
    }

    // Misc //////////////////////////////////////////////////////////////

    /**
     * Check if the intake is in a stable state (at setpoint and not moving)
     */
    public boolean isStableState() {
        double goalPos = profile.getGoal().position;
        boolean withinTolerance = MiscUtils.withinMargin(
                RobotConfig.Accuracy.Intake.ARM_ANGLE_TOLERANCE.in(Radians),
                goalPos,
                getAngle().in(Radians));

        return withinTolerance && isStopped();
    }

    /**
     * Check if the intake is stopped (not moving)
     * 
     * @return Whether or not the intake is stopped
     */
    public boolean isStopped() {
        return MiscUtils.isStopped(getVelocity().in(RadiansPerSecond),
                RobotConfig.Accuracy.Intake.ARM_VELOCITY_TOLERANCE.in(RadiansPerSecond));

    }

    /**
     * Calculate and apply voltage to move the intake to the goal angle
     */
    public void moveToGoal() {
        if (isStableState()) {
            Logger.recordOutput("IntakeArm/DesiredVolts", 0);
            io.setVoltage(0);
            return;
        }

        double pidOutput = profile.calculate(getAngle().in(Radians));
        State setpoint = profile.getSetpoint();
        double feedforwardOutput = feedforward.calculate(setpoint.position, setpoint.velocity);

        double voltageOutput = pidOutput + feedforwardOutput;

        Logger.recordOutput("IntakeArm/DesiredVolts", voltageOutput);
        io.setVoltage(voltageOutput);
    }

    public void refreshTuneables() {
        profile.setP(kP.get());
        profile.setI(kI.get());
        profile.setD(kD.get());

        feedforward.setKa(kA.get());
        feedforward.setKg(kG.get());
        feedforward.setKs(kS.get());
        feedforward.setKv(kV.get());

        profile.setConstraints(
                new Constraints(
                        maxVelocity.get(),
                        maxAcceleration.get()));
    }

    /**
     * Log intake arm data to AdvantageKit
     */
    public void logData() {
        Logger.processInputs("IntakeArm", data);

        Logger.recordOutput("IntakeArm/TargetAngle", currentState.angle);
        Logger.recordOutput("IntakeArm/State", currentState.toString());
        Logger.recordOutput("IntakeArm/IsStableState", isStableState());

        Logger.recordOutput("IntakeArm/PID Setpoint", profile.getSetpoint());
    }

    /**
     * Update the logged mech 2d visualization to the current intake angle
     */
    public void updateMechanism() {
        intakeLigament.setAngle(getAngle());
        Logger.recordOutput("IntakeArm/mech", mech);
    }

    /**
     * periodic!!
     */
    @Override
    public void periodic() {
        refreshTuneables();

        // always get latest data first
        io.updateData();

        // log data and update visualizations
        logData();
        updateMechanism();

        // then do control
        moveToGoal();
    }
}
