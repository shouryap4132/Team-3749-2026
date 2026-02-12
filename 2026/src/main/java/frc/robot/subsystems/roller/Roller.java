package frc.robot.subsystems.roller;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.RobotConfig.RobotType;
import frc.robot.config.RollerConfig.RollerImplementations;
import frc.robot.config.RollerConfig.RollerStates;
import frc.robot.subsystems.roller.RollerIO.RollerData;
import frc.robot.subsystems.roller.real.RollerReal;
import frc.robot.subsystems.roller.sim.RollerSim;
import frc.robot.utils.MiscUtils;
import frc.robot.utils.MiscUtils.ControlConfig;

/**
 * Roller subsystem to control rollers for intaking and moving fuel.
 * 
 * @author Anish Gupta
 * @author Rigved Gaddam
 * @author Seonyoo Pak
 */
public class Roller extends SubsystemBase {

    private final RollerIO io;
    private final RollerData data = new RollerData();

    private final RollerImplementations implementation;
    private final SimpleMotorFeedforward feedforward;
    private final PIDController pidController;

    private RollerStates currentState = RollerStates.STOP;

    /**
     * Roller subsystem constructor
     * 
     * @param implementation The implementation of the roller to use
     */
    public Roller(RollerImplementations implementation) {
        this.implementation = implementation;

        if (MiscUtils.getRobotType() == RobotType.REAL) {
            io = new RollerReal(implementation);
        } else {
            io = new RollerSim(implementation);
        }

        io.setMotorIdleMode(implementation.config.idleMode);

        ControlConfig controlConfig = implementation.config.CONTROL_CONFIG.get();

        this.feedforward = new SimpleMotorFeedforward(
                controlConfig.kS,
                controlConfig.kV,
                controlConfig.kA);

        this.pidController = new PIDController(
                controlConfig.kP,
                controlConfig.kI,
                controlConfig.kD);
    }

    // Getters ///////////////////////////////////////////////////////////

    /**
     * Gets the current state of the roller
     * 
     * @return Current roller state
     */
    public RollerStates getState() {
        return currentState;
    }

    /**
     * Get the current data of the roller
     * 
     * @return A RollerData object
     */
    public RollerData getData() {
        return data;
    }

    /**
     * Gets the current velocity of the roller
     * 
     * @return Current roller velocity
     */
    public AngularVelocity getVelocity() {
        if (data.rollerVelocityRadPerSec.isEmpty()) {
            return RadiansPerSecond.of(0);
        }
        return RadiansPerSecond.of(data.rollerVelocityRadPerSec.get(0));
    }

    // Setters ///////////////////////////////////////////////////////////

    /**
     * Sets the current state of the roller
     * 
     * @param rollerState Target RollerStates object
     */
    public void setState(RollerStates rollerState) {
        if (rollerState == null) {
            Logger.recordOutput("Errors/Roller/" + implementation.toString(), "Attempted to set roller state to null");
            return;
        }

        this.currentState = rollerState;
    }

    // Misc //////////////////////////////////////////////////////////////

    public boolean isStopped() {
        return MiscUtils.isStopped(getVelocity().in(RadiansPerSecond),
                implementation.config.velocityTolerance.in(RadiansPerSecond));

    }

    public boolean isStableState() {
        boolean withinTolerance = MiscUtils.withinMargin(
                implementation.config.velocityTolerance.in(RadiansPerSecond),
                currentState.rollerVelocity.in(RadiansPerSecond),
                getVelocity().in(RadiansPerSecond));

        return withinTolerance;
    }

    /**
     * Sets the voltage to the roller motor
     * 
     * @param volts The number of volts to send
     */
    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    /**
     * Sets the target velocity of the roller in radians per second
     * 
     * @param velocityRadPerSec Velocity in radians per second to set
     */
    public void runVelocity(AngularVelocity velocity) {
        double pidOutput;
        double ffOutput;
        double velocityRadPerSec = velocity.in(RadiansPerSecond);

        if (velocityRadPerSec == 0) {
            setVoltage(0);
            pidOutput = 0;
            ffOutput = 0;
        } else {
            pidOutput = pidController.calculate(getVelocity().in(RadiansPerSecond), velocityRadPerSec);
            ffOutput = feedforward.calculate(velocityRadPerSec);
            double volts = MiscUtils.voltageClamp(pidOutput + ffOutput);
            setVoltage(volts);
        }

        Logger.recordOutput("Roller/" + implementation.toString() + "/PID", pidOutput);
        Logger.recordOutput("Roller/" + implementation.toString() + "/FF", ffOutput);
    }

    /**
     * Log the data from each of the motors
     */
    public void logData() {
        String basePrefix = "Roller/" + implementation.toString();

        for (int i = 0; i < data.numMotors; i++) {
            Logger.recordOutput(basePrefix + "/motor" + i + "/CAN ID", implementation.config.canIds[i]);
            Logger.recordOutput(basePrefix + "/motor" + i + "/current", data.currentAmps.get(i));
            Logger.recordOutput(basePrefix + "/motor" + i + "/temperature", data.rollerTempCelsius.get(i));
            Logger.recordOutput(basePrefix + "/motor" + i + "/position", data.rollerPositionRad.get(i));
            Logger.recordOutput(basePrefix + "/motor" + i + "/velocity", data.rollerVelocityRadPerSec.get(i));
            Logger.recordOutput(basePrefix + "/motor" + i + "/appliedVoltage", data.rollerAppliedVolts.get(i));
            Logger.recordOutput(basePrefix + "/motor" + i + "/acceleration", data.acceleration.get(i));
        }

        Logger.recordOutput(basePrefix + "/State", currentState.name());
        Logger.recordOutput(basePrefix + "/SetpointVelocity", currentState.rollerVelocity);
    }

    /**
     * Periodic function to update data, sim, log, and run the rollers
     */
    @Override
    public void periodic() {
        io.updateData(data);

        logData();

        runVelocity(currentState.rollerVelocity);
    }
}
