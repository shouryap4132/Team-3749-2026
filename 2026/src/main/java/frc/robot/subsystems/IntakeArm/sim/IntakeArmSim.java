package frc.robot.subsystems.IntakeArm.sim;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.config.RobotConfig;
import frc.robot.config.IntakeArmConfig.IntakeArmSpecs;
import frc.robot.config.IntakeArmConfig.IntakeArmStates;
import frc.robot.subsystems.IntakeArm.IntakeArmIO;
import frc.robot.subsystems.IntakeArm.IntakeArmDataAutoLogged;
import frc.robot.utils.MiscUtils;

/**
 * Implementation of IntakeArmIO for a simulated intake arm
 * 
 * @author Anish Gupta
 * @author Rigved Gaddam
 * @author Seonyoo Pak
 */

public class IntakeArmSim implements IntakeArmIO {
    private final IntakeArmDataAutoLogged data;
    private final SingleJointedArmSim intakeArmSim = new SingleJointedArmSim(DCMotor.getNEO(1),
            IntakeArmSpecs.GEARING,
            IntakeArmSpecs.INTAKE_ARM_MOI,
            IntakeArmSpecs.INTAKE_ARM_LENGTH.in(Meters),
            IntakeArmSpecs.MIN_ANGLE.in(Radians),
            IntakeArmSpecs.MAX_ANGLE.in(Radians),
            IntakeArmSpecs.SIMULATE_GRAVITY,
            IntakeArmStates.STOW.angle.in(Radians));

    /**
     * Constructor to make a new IntakeArmSim object
     * 
     * @param intakeData The IntakeArmData object to log data to
     */
    public IntakeArmSim(IntakeArmDataAutoLogged intakeData) {
        data = intakeData;
    }

    /**
     * Send voltage to the intake arm sim
     * 
     * @param volts The number of volts to send
     */
    @Override
    public void setVoltage(double volts) {
        double clamped = MiscUtils.voltageClamp(volts);
        intakeArmSim.setInputVoltage(clamped);
        data.appliedVolts = clamped;
    }

    /**
     * Set what the motor does in idle. In sim, this has no effect
     * 
     * @param idleMode kCoast or kBrake
     */
    @Override
    public void setMotorIdleMode(IdleMode idleMode) {
        System.out.println("[IntakeArmSim] setMotorIdleMode() called in sim with no effect");
    }

    /**
     * Update the intake arm data
     */
    @Override
    public void updateData() {
        double deltaT = RobotConfig.General.NOMINAL_LOOP_TIME_S;

        intakeArmSim.update(deltaT);

        data.angle = Radians.of(intakeArmSim.getAngleRads());

        AngularVelocity prevVelocity = data.velocity.copy();
        data.velocity = RadiansPerSecond.of(intakeArmSim.getVelocityRadPerSec());
        data.accel = data.velocity.minus(prevVelocity).div(Seconds.of(deltaT));

        data.currentAmps = intakeArmSim.getCurrentDrawAmps();
        data.appliedVolts = intakeArmSim.getInput(1);
    }
}
