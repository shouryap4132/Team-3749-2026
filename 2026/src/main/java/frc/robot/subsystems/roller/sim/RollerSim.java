package frc.robot.subsystems.roller.sim;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.config.RobotConfig;
import frc.robot.config.RollerConfig.RollerImplementations;
import frc.robot.config.RollerConfig.RollerConfigBase;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.utils.MiscUtils;

/**
 * Implementation of RollerIO for a simulated roller. This program only
 * simulates one roller
 * 
 * @author Anish Gupta
 * @author Rigved Gaddam
 * @author Seonyoo Pak
 */
public class RollerSim implements RollerIO {
    private RollerImplementations implementation;
    private FlywheelSim rollerMotor;

    private double previousVelocity = 0;
    private double velocity = 0;
    private double position = 0;

    private RollerConfigBase config;

    /**
     * Constructor to make a new RollerSim object
     * 
     * @param implementation The implementation of the roller to construct
     */
    public RollerSim(RollerImplementations implementation) {
        this.implementation = implementation;
        config = implementation.config;
        DCMotor motor = DCMotor.getNEO(1);
        LinearSystem<N1, N1, N1> flyWheelSystem = LinearSystemId.createFlywheelSystem(motor, config.momentOfInertia,
                config.gearRatio);
        rollerMotor = new FlywheelSim(flyWheelSystem, motor, config.measurementNoise);
    }

    /**
     * Update the RollerData of the rollers. Since only one roller is simmed,
     * all motors' data are the same
     * 
     * @param data The RollerData object to update
     */
    @Override
    public void updateData(RollerData data) {

        if (!data.isInitialized) {
            data.initArrays(config.canIds.length);
        }

        velocity = rollerMotor.getAngularVelocityRadPerSec();
        position += velocity * RobotConfig.General.NOMINAL_LOOP_TIME_S;


        for (int i = 0; i < config.canIds.length; i++) {
            data.rollerAppliedVolts.set(i, rollerMotor.getInputVoltage());
            data.rollerVelocityRadPerSec.set(i, velocity);
            data.currentAmps.set(i, rollerMotor.getCurrentDrawAmps());
            data.rollerTempCelsius.set(i, 0.0);
            data.rollerPositionRad.set(i, position);
            data.acceleration.set(i, (velocity - previousVelocity) / RobotConfig.General.NOMINAL_LOOP_TIME_S);
        }

        previousVelocity = velocity;

    }

    /**
     * Send voltage to the motors
     * 
     * @param rollerVolts The amount of volts to send
     */
    @Override
    public void setVoltage(double rollerVolts) {
        rollerMotor.setInputVoltage(MiscUtils.voltageClamp(rollerVolts));
    }

    /**
     * Update the FlywheelSim
     * 
     * @param dtseconds Delta time, how much time has passed since the last update
     */
    @Override
    public void updateSimulation(double dtseconds) {
        rollerMotor.update(dtseconds);
    }

    /**
     * Set what the motor does in idle. In sim, this has no effect
     * 
     * @param idleMode kCoast or kBrake
     */
    @Override
    public void setMotorIdleMode(IdleMode idleMode) {
        // No need to implement for sim
        System.out.println(String.format("[RollerSim (%s)] setMotorIdleMode() called in sim with no effect",
                implementation.toString()));

    }
}
