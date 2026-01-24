package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.commands.SwerveVisionLogic;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FuelLine;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Rotation2d;

public class CommandFactory {

    private final CommandSwerveDrivetrain cmdSwerveDriveState;
    private final FuelLine intake;
    private final Shooter shooter;
    private final Climber climber;
    private final Limelight cmdLimelight;
    private final Leds leds;

    public Command cmdSwerveVisionLogic() {
        return new SwerveVisionLogic(cmdLimelight, cmdSwerveDriveState);
    }

    public CommandFactory(CommandSwerveDrivetrain SwerveDriveState, FuelLine intake, Shooter shooter, Climber climber,
            Limelight limelight, Leds leds) {
        this.cmdSwerveDriveState = SwerveDriveState;
        this.intake = intake;
        this.shooter = shooter;
        this.climber = climber;
        this.cmdLimelight = limelight;
        this.leds = leds;

    }

    public Command cmdPathfindToRedHubTag() {
        return new SelectCommand<>(
                Map.ofEntries(
                        Map.entry(6, cmdPathfindToPose(HubScoringMap.getReefPoseFromLimelightID(6))),
                        Map.entry(7, cmdPathfindToPose(HubScoringMap.getReefPoseFromLimelightID(7))),
                        Map.entry(8, cmdPathfindToPose(HubScoringMap.getReefPoseFromLimelightID(8))),
                        Map.entry(9, cmdPathfindToPose(HubScoringMap.getReefPoseFromLimelightID(9))),
                        Map.entry(10, cmdPathfindToPose(HubScoringMap.getReefPoseFromLimelightID(10))),
                        Map.entry(11, cmdPathfindToPose(HubScoringMap.getReefPoseFromLimelightID(11)))),
                () -> limelight.getFid()).withName("Drivetain: Pathfind to red reef");
    }

    public Command cmdPathfindToBlueHubTag() {
        return new SelectCommand<>(
                Map.ofEntries(
                        Map.entry(17, cmdPathfindToPose(ReefScoringMap.getReefPoseFromLimelightID(17))),
                        Map.entry(18, cmdPathfindToPose(ReefScoringMap.getReefPoseFromLimelightID(18))),
                        Map.entry(19, cmdPathfindToPose(ReefScoringMap.getReefPoseFromLimelightID(19))),
                        Map.entry(20, cmdPathfindToPose(ReefScoringMap.getReefPoseFromLimelightID(20))),
                        Map.entry(21, cmdPathfindToPose(ReefScoringMap.getReefPoseFromLimelightID(21))),
                        Map.entry(22, cmdPathfindToPose(ReefScoringMap.getReefPoseFromLimelightID(22)))),
                () -> limelight.getFid()).withName("Drivetain: Pathfind to blue reef");
    }
}
