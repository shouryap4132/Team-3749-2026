package frc.robot.subsystems.Climb;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.ClimbConfig.*;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.RobotType;
import frc.robot.subsystems.Climb.real.ClimbReal;
import frc.robot.subsystems.Climb.sim.ClimbSimulation;
import frc.robot.utils.MiscUtils;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Subsystem for controlling the Robot Climb mechanism.
 * Uses AdvantageKit for logging and simulation-ready IO abstraction.
 */
public class Climb extends SubsystemBase {

  private final ClimbIO io;
  private final ClimbDataAutoLogged data = new ClimbDataAutoLogged();

  // Control Constants from Config
  private static final MiscUtils.ControlConfig config = ClimbControl.CONTROL_CONFIG.get();
  /**
   * * Profiled PID Controller: Handles smooth motion by following a trapezoidal
   * velocity profile.
   * This prevents the climber from jerking by limiting max velocity and
   * acceleration.
   * 
   */
  private final ProfiledPIDController profile = new ProfiledPIDController(
      config.kP, config.kI, config.kD,
      new Constraints(
          ClimbControl.MAX_VELOCITY.in(MetersPerSecond),
          ClimbControl.MAX_ACCEL.in(MetersPerSecondPerSecond)));

  /**
   * Feedforward Controller: Calculates the voltage needed to overcome gravity
   * (kG)
   * and friction/inertia (kS, kV, kA) to keep the elevator at a setpoint.
   */
  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(
      config.kS, config.kG, config.kV, config.kA);

  // Subsystem State
  private ClimbStates state = ClimbStates.STOW;

  // AdvantageKit Mechanism2d Visualization
  private final LoggedMechanism2d mech = new LoggedMechanism2d(3, 3);
  private final LoggedMechanismRoot2d root = mech.getRoot("ClimbRoot", 1, 0);
  private final LoggedMechanismLigament2d ligament = root.append(
      new LoggedMechanismLigament2d("Climb", ClimbSpecs.STARTING_HEIGHT.in(Meters), 90));

  // Tunables
  public static LoggedNetworkNumber kG = new LoggedNetworkNumber("/Tuning/Climb/kG", config.kG);
  public static LoggedNetworkNumber kP = new LoggedNetworkNumber("/Tuning/Climb/kP", config.kP);
  public static LoggedNetworkNumber kI = new LoggedNetworkNumber("/Tuning/Climb/kI", config.kI);
  public static LoggedNetworkNumber kD = new LoggedNetworkNumber("/Tuning/Climb/kD", config.kD);
  public static LoggedNetworkNumber kS = new LoggedNetworkNumber("/Tuning/Climb/kS", config.kS);
  public static LoggedNetworkNumber kV = new LoggedNetworkNumber("/Tuning/Climb/kV", config.kV);
  public static LoggedNetworkNumber kA = new LoggedNetworkNumber("/Tuning/Climb/kA", config.kA);
  public static LoggedNetworkNumber maxVelocity = new LoggedNetworkNumber("/Tuning/Climb/Max Velocity",
      ClimbControl.MAX_VELOCITY.in(MetersPerSecond));
  public static LoggedNetworkNumber maxAcceleration = new LoggedNetworkNumber("/Tuning/Climb/Max Acceleration",
      ClimbControl.MAX_ACCEL.in(MetersPerSecondPerSecond));

  /**
   * Climb subsystem constructor
   */
  public Climb() {
    if (MiscUtils.getRobotType() == RobotType.REAL) {
      io = new ClimbReal(data);
    } else {
      io = new ClimbSimulation(data);
    }
  }

  // Getters ///////////////////////////////////////////////////////////

  public Distance getHeight() {
    return data.height;
  }

  public LinearVelocity getVelocity() {
    return data.velocity;
  }

  public ClimbStates getState() {
    return state;
  }

  // Setters ///////////////////////////////////////////////////////////

  public void setState(ClimbStates newState) {
    if (newState == null) {
      Logger.recordOutput("Errors/Climb", "Attempted to set climb state to null");
      return;
    }

    state = newState;

    profile.reset(
        getHeight().in(Meters),
        getVelocity().in(MetersPerSecond));

    profile.setGoal(new State(state.height.in(Meters), 0));
  }

  // Misc //////////////////////////////////////////////////////////////

  public boolean isStopped() {
    return MiscUtils.isStopped(getVelocity().in(MetersPerSecond),
        RobotConfig.Accuracy.Climb.CLIMB_VELOCITY_TOLERANCE.in(MetersPerSecond));
  }

  public boolean isStableState() {
    double goalPos = profile.getGoal().position;
    boolean withinTolerance = MiscUtils.withinMargin(
        RobotConfig.Accuracy.Climb.CLIMB_HEIGHT_TOLERANCE.in(Meters),
        goalPos,
        getHeight().in(Meters));

    return withinTolerance && isStopped();
  }

  public void moveToGoal() {
    if (isStableState()) {
      Logger.recordOutput("Climb/DesiredVolts", 0);
      io.setVoltage(0);
      return;
    }

    double pidOutput = profile.calculate(getHeight().in(Meters));
    State setpoint = profile.getSetpoint();
    double feedforwardOutput = feedforward.calculate(setpoint.velocity);

    double voltageOutput = pidOutput + feedforwardOutput;

    Logger.recordOutput("Climb/DesiredVolts", voltageOutput);
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

  public void logData() {
    Logger.processInputs("Climb", data);

    Logger.recordOutput("Climb/TargetVelocity", profile.getSetpoint().velocity);
    Logger.recordOutput("Climb/HeightGoal", state.height.in(Meters));

    Logger.recordOutput("Climb/State", state.toString());

    Logger.recordOutput("Climb/Position", getHeight());
    Logger.recordOutput("Climb/Velocity", data.velocity);
    Logger.recordOutput("Climb/Acceleration", data.accel);

    Logger.recordOutput("Climb/LeftAppliedVolts", data.leftAppliedVolts);
    Logger.recordOutput("Climb/RightAppliedVolts", data.rightAppliedVolts);
    Logger.recordOutput("Climb/LeftCurrentAmps", data.leftCurrentAmps);
    Logger.recordOutput("Climb/RightCurrentAmps", data.rightCurrentAmps);

    Logger.recordOutput("Climb/currentCommand",
        this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());
  }

  /** Updates the 2D visual representation of the climber. */
  public void updateMechanism() {
    ligament.setLength(getHeight().in(Meters));
    Logger.recordOutput("Climb/Mechanism", mech);
  }

  @Override
  public void periodic() {
    refreshTuneables();

    io.updateData();

    logData();
    updateMechanism();

    moveToGoal();
  }
}