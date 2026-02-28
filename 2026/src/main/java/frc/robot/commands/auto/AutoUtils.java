package frc.robot.commands.auto;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Robot;
import frc.robot.config.RobotConfig.Accuracy;

public class AutoUtils {
    private static AutoFactory factory;
    private static AutoChooser chooser;
    public static ChoreoAllianceFlipUtil.Flipper flipper = ChoreoAllianceFlipUtil.getFlipper();

    private static Map<String, Supplier<Command>> eventMarkerCommands = new HashMap<>();

    static {
        // Initialize default event markers
        eventMarkerCommands.put("shoot", () -> Commands.print("scored!!!"));
        eventMarkerCommands.put("climb", () -> Commands.print("climbed!!!"));
        eventMarkerCommands.put("intake", () -> Commands.print("intake!!!"));

    }

    /**
     * Initializes the auto factory and chooser. Call once during robot init.
     */
    public static void initAutoUtils() {
        setupFactory();
        setupChooser();
    }

    private static void setupFactory() {
        factory = new AutoFactory(
                Robot.swerve::getPose,
                Robot.swerve::setOdometry,
                (SwerveSample sample) -> Robot.swerve.driveToSample(sample),
                true,
                Robot.swerve);
    }

    private static void setupChooser() {
        chooser = new AutoChooser();
        SmartDashboard.putData("Auto Chooser", chooser);

        chooser.addCmd("Sample", () -> trajectory("Sample")
                .bindEvent("shoot", "shoot") // Bind the "score" event from trajectory to the "score" command
                .holdFinalPose(false)
                .build());
        chooser.addCmd("TripleThreatAuto", () -> Autos.tripleThreatAuto());
        chooser.addCmd("getStart1LeftNear", () -> Autos.getStart1LeftNear());
        chooser.addCmd("No Auto", () -> Commands.print("[AutoUtils] No Auto Selected"));
        chooser.select("No Auto");
    }

    /**
     * Registers a global event marker command that can be used across all
     * trajectories.
     */
    public static void registerEventMarker(String name, Supplier<Command> commandSupplier) {
        eventMarkerCommands.put(name, commandSupplier);
    }

    /**
     * Registers a global event marker command that can be used across all
     * trajectories.
     */
    public static void registerEventMarker(String name, Command command) {
        eventMarkerCommands.put(name, () -> command);
    }

    /**
     * Sets up auto to run when autonomous mode is enabled.
     */
    public static void setupAutoTrigger() {
        RobotModeTriggers.autonomous()
                .whileTrue(chooser.selectedCommandScheduler());
    }

    /**
     * Schedules the currently selected auto command.
     */
    public static void runSelectedCommand() {
        Command autoCmd = chooser.selectedCommand();
        if (autoCmd != null) {
            CommandScheduler.getInstance().schedule(autoCmd);
        } else {
            System.out.println("[AutoUtils] No Auto Command Selected");
        }
    }

    /**
     * Registers a routine to the auto chooser.
     */
    public static void registerRoutine(String name, Supplier<AutoRoutine> routineSupplier) {
        chooser.addRoutine(name, routineSupplier);
    }

    /**
     * Registers a command to the auto chooser.
     */
    public static void registerCommand(String name, Supplier<Command> commandSupplier) {
        chooser.addCmd(name, commandSupplier);
    }

    public static void applyEventMarkers(AutoTrajectory traj, String... markers) {
        for (String marker : markers) {
            if (!eventMarkerCommands.containsKey(marker)) {
                System.out.println("[AutoUtils]: No command found for event marker: " + marker);
            }

            traj.atPose(marker, Accuracy.Swerve.TRANSLATE_TOLERANCE.in(Meters),
                    Accuracy.Swerve.ROTATION_TOLERANCE.in(Radians))
                    .onTrue(eventMarkerCommands.get(marker).get());
        }
    }

    public static AutoChooser getChooser() {
        return chooser;
    }

    public static AutoFactory getFactory() {
        return factory;
    }

    /**
     * Creates a new trajectory builder for easy trajectory configuration.
     */
    public static TrajectoryBuilder trajectory(String name) {
        return new TrajectoryBuilder(name);
    }

    /**
     * Builder class for configuring and creating trajectory commands with event
     * markers.
     */
    public static class TrajectoryBuilder {
        private final String trajectoryName;
        private final Map<String, Supplier<Command>> poseMarkers = new HashMap<>();
        private final Map<Double, Supplier<Command>> timeMarkers = new HashMap<>();
        private final Map<String, Supplier<Command>> namedEventBindings = new HashMap<>();
        private boolean resetOdometry = true;
        private boolean holdFinalPose = true;

        private TrajectoryBuilder(String trajectoryName) {
            this.trajectoryName = trajectoryName;
        }

        /**
         * Binds a named event from the Choreo trajectory file to a command.
         * Use this to handle events defined in the trajectory's JSON file.
         * 
         * @param eventName The name of the event as defined in the trajectory file
         * @param command   The command to run when the event is triggered
         * @return This builder for chaining
         */
        public TrajectoryBuilder bindEvent(String eventName, Supplier<Command> command) {
            namedEventBindings.put(eventName, command);
            return this;
        }

        /**
         * Binds a named event from the Choreo trajectory file to a command.
         * 
         * @param eventName The name of the event as defined in the trajectory file
         * @param command   The command to run when the event is triggered
         * @return This builder for chaining
         */
        public TrajectoryBuilder bindEvent(String eventName, Command command) {
            return bindEvent(eventName, () -> command);
        }

        /**
         * Binds a named event to a registered global event marker.
         * 
         * @param eventName       The name of the event as defined in the trajectory
         *                        file
         * @param eventMarkerName The name of the registered global event marker
         * @return This builder for chaining
         */
        public TrajectoryBuilder bindEvent(String eventName, String eventMarkerName) {
            if (!eventMarkerCommands.containsKey(eventMarkerName)) {
                System.out.println("[AutoUtils] Warning: Event marker '" + eventMarkerName + "' not registered");
                return this;
            }
            namedEventBindings.put(eventName, eventMarkerCommands.get(eventMarkerName));
            return this;
        }

        /**
         * Automatically binds all registered global event markers to this trajectory.
         * This will bind any event in the trajectory file that matches a registered
         * event marker name.
         * 
         * @return This builder for chaining
         */
        public TrajectoryBuilder withAllEventMarkers() {
            namedEventBindings.putAll(eventMarkerCommands);
            return this;
        }

        /**
         * Adds a command to trigger when reaching a named pose marker.
         */
        public TrajectoryBuilder atPose(String poseName, Supplier<Command> command) {
            poseMarkers.put(poseName, command);
            return this;
        }

        /**
         * Adds a command to trigger when reaching a named pose marker.
         */
        public TrajectoryBuilder atPose(String poseName, Command command) {
            return atPose(poseName, () -> command);
        }

        /**
         * Adds a runnable to trigger when reaching a named pose marker.
         */
        public TrajectoryBuilder atPose(String poseName, Runnable command, Subsystem... requirements) {
            return atPose(poseName, Commands.runOnce(command, requirements));
        }

        /**
         * Adds a registered global event marker at a named pose.
         */
        public TrajectoryBuilder atPose(String poseName, String eventMarkerName) {
            if (!eventMarkerCommands.containsKey(eventMarkerName)) {
                System.out.println("[AutoUtils] Warning: Event marker '" + eventMarkerName + "' not registered");
                return this;
            }
            poseMarkers.put(poseName, eventMarkerCommands.get(eventMarkerName));
            return this;
        }

        /**
         * Adds a command to trigger at a specific time in the trajectory.
         */
        public TrajectoryBuilder atTime(double timeSeconds, Supplier<Command> command) {
            timeMarkers.put(timeSeconds, command);
            return this;
        }

        /**
         * Adds a command to trigger at a specific time in the trajectory.
         */
        public TrajectoryBuilder atTime(double timeSeconds, Command command) {
            return atTime(timeSeconds, () -> command);
        }

        /**
         * Adds a runnable to trigger at a specific time in the trajectory.
         */
        public TrajectoryBuilder atTime(double timeSeconds, Runnable command, Subsystem... requirements) {
            return atTime(timeSeconds, Commands.runOnce(command, requirements));
        }

        /**
         * Sets whether to reset odometry at the start of this trajectory.
         */
        public TrajectoryBuilder resetOdometry(boolean reset) {
            this.resetOdometry = reset;
            return this;
        }

        /**
         * Sets whether to hold position at the final pose after trajectory completion.
         */
        public TrajectoryBuilder holdFinalPose(boolean hold) {
            this.holdFinalPose = hold;
            return this;
        }

        /**
         * Builds and returns the trajectory command.
         */
        public Command build() {
            // Bind all named events from the trajectory file to their commands
            for (Map.Entry<String, Supplier<Command>> entry : namedEventBindings.entrySet()) {
                factory.bind(entry.getKey(), entry.getValue().get());
            }

            AutoRoutine routine = factory.newRoutine(trajectoryName);
            AutoTrajectory traj = routine.trajectory(trajectoryName);

            // Apply pose markers (manual triggers at specific poses)
            for (Map.Entry<String, Supplier<Command>> entry : poseMarkers.entrySet()) {
                traj.atPose(entry.getKey(), Accuracy.Swerve.TRANSLATE_TOLERANCE.in(Meters),
                        Accuracy.Swerve.ROTATION_TOLERANCE.in(Radians))
                        .onTrue(entry.getValue().get());
            }

            // Apply time markers (manual triggers at specific times)
            for (Map.Entry<Double, Supplier<Command>> entry : timeMarkers.entrySet()) {
                traj.atTime(entry.getKey()).onTrue(entry.getValue().get());
            }

            Pose2d finalPose = traj.getFinalPose().orElse(Pose2d.kZero);

            if (holdFinalPose) {
                routine.active().onTrue(Commands.sequence(
                        resetOdometry ? traj.resetOdometry() : Commands.none(),
                        traj.cmd(),
                        Commands.run(() -> Robot.swerve.driveToPose(finalPose), Robot.swerve)));
            } else {
                routine.active().onTrue(Commands.sequence(
                        resetOdometry ? traj.resetOdometry() : Commands.none(),
                        traj.cmd()));
            }

            Logger.recordOutput("Auto/" + trajectoryName + "/InitialPose", traj.getInitialPose().orElse(Pose2d.kZero));
            Logger.recordOutput("Auto/" + trajectoryName + "/FinalPose", finalPose);

            return Commands.sequence(
                    Commands.print("[AutoUtils] Running - " + trajectoryName),
                    routine.cmd(traj.done()),
                    Commands.print("[AutoUtils] Finished - " + trajectoryName));
        }

        /**
         * Builds the command and registers it to the auto chooser.
         */
        public void register() {
            registerCommand(trajectoryName, () -> this.build());
        }
    }

}