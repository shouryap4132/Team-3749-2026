package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Autos {
    public Autos() {
        AutoUtils.initAutoUtils();
        registerEventMarkers();
        registerAutos();
    }

    private void registerEventMarkers() {
        AutoUtils.registerEventMarker("score", () -> Commands.print("[Auto] Scoring..."));
        AutoUtils.registerEventMarker("intake", () -> Commands.print("[Auto] Intaking..."));
    }

    private void registerAutos() {
        AutoUtils.trajectory("Sample").register();

        AutoUtils.trajectory("Random")
                .atPose("Score", "score")
                .atPose("Intake", "intake")
                .register();

        AutoUtils.trajectory("Complex")
                .atPose("Pickup", () -> Commands.print("[Auto] At pickup"))
                .atTime(1.5, () -> Commands.print("[Auto] 1.5 seconds in"))
                .resetOdometry(true)
                .holdFinalPose(true)
                .register();
    }


    public static Command getStart1LeftNear() {
        Command path = AutoUtils.trajectory("Start1_LeftNearIntake")
                .holdFinalPose(true)
                .resetOdometry(true)
                .build();  
        return path;
    }

    public static Command tripleThreatAuto() {
        Command trajectory1 = AutoUtils.trajectory("Start1_LeftNearIntake")
                .holdFinalPose(false)
                .build();
        Command trajectory2 = AutoUtils.trajectory("LeftNearIntake_LeftNearIntakeThrough")
                .bindEvent("intake", "intake")
                .holdFinalPose(false)
                .resetOdometry(false)
                .build();
        Command trajectory3 = AutoUtils.trajectory("LeftNearIntakeThrough_Shooter")
                .bindEvent("score", "score")
                .holdFinalPose(false)
                .resetOdometry(false)
                .build();    
        
        return Commands.sequence(trajectory1, trajectory2, trajectory3);
    }

    public static Command getOutpostScore() {        
        Command path = AutoUtils.trajectory("Outpost_Shoot")
                .holdFinalPose(true)
                .resetOdometry(true)
                .build();  
        return path;
    }

    public static Command outpostScoreTwice() {        
        Command trajectory1 = Auto
        return path;
    }

    

    public static Command getTaxiAuto() {
        Command path = AutoUtils.trajectory("taxi")
                .holdFinalPose(true)
                .resetOdometry(true)
                .build();  
        return path;
    }
}