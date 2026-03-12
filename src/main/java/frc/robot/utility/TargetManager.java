package frc.robot.utility;

import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Comparator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utility.FieldLocalization.Landmark;
import frc.robot.utility.FieldLocalization.Zones;

public class TargetManager {

    public record TargetingState(Rotation2d rotation, Translation2d target, double distance, boolean isAligned) {
    }

    private record PrioritizedLandmark(Landmark landmark, int priority) {
    }

    private final CommandSwerveDrivetrain swerveDrivetrain;

    private TargetingState activeState;

    private final Map<Alliance, Map<Zones, PrioritizedLandmark>> strategy = new EnumMap<>(Alliance.class);

    public TargetManager(CommandSwerveDrivetrain swerveDrivetrain) {
        this.swerveDrivetrain = swerveDrivetrain;

        strategy.put(Alliance.Blue, new EnumMap<>(Zones.class));
        strategy.put(Alliance.Red, new EnumMap<>(Zones.class));

        var blue = strategy.get(Alliance.Blue);
        blue.put(Zones.BLUE_ALLIANCE_ZONE, new PrioritizedLandmark(Landmark.BLUE_HUB, 10));
        blue.put(Zones.BLUE_DEPOT_RED_OUTPOST_NEUTRAL_ZONE, new PrioritizedLandmark(Landmark.BLUE_DEPOT, 20));
        blue.put(Zones.BLUE_OUTPOST_RED_DEPOT_NEUTRAL_ZONE, new PrioritizedLandmark(Landmark.BLUE_OUTPOST, 20));

        var red = strategy.get(Alliance.Red);
        red.put(Zones.RED_ALLIANCE_ZONE, new PrioritizedLandmark(Landmark.RED_HUB, 10));
        red.put(Zones.BLUE_DEPOT_RED_OUTPOST_NEUTRAL_ZONE, new PrioritizedLandmark(Landmark.RED_OUTPOST, 20));
        red.put(Zones.BLUE_OUTPOST_RED_DEPOT_NEUTRAL_ZONE, new PrioritizedLandmark(Landmark.RED_DEPOT, 20));
    }

    public void updateTargetState(List<Zones> activeZones, Alliance alliance) {
        var swerveStatePose = swerveDrivetrain.getState().Pose;

        activeState = getTarget(activeZones, alliance)
                .map(target -> {
                    Rotation2d targetAngle = determineRotationToTarget(target, swerveStatePose);
                    Rotation2d fieldRelativeTargetAngle = targetAngle
                            .minus(swerveDrivetrain.getOperatorForwardDirection());

                    double distance = target.getDistance(swerveStatePose.getTranslation());
                    double angleError = Math
                            .abs(swerveStatePose.getRotation().minus(fieldRelativeTargetAngle).getDegrees());
                    boolean aligned = angleError < 2.5;
                    return new TargetingState(targetAngle, target, distance, aligned);
                })
                .orElseGet(() -> new TargetingState(
                        Rotation2d.kZero,
                        Translation2d.kZero,
                        0.0,
                        false));
    }

    public TargetingState getTargetingState() {
        return activeState;
    }

    private Rotation2d determineRotationToTarget(Translation2d targetTranslation, Pose2d swervePose) {
        return Rotation2d.fromRadians(
                Math.atan2(
                        targetTranslation.getY() - swervePose.getY(),
                        targetTranslation.getX() - swervePose.getX()))
                .plus(swerveDrivetrain.getOperatorForwardDirection());
    }

    private Optional<Translation2d> getTarget(List<Zones> activeZones, Alliance alliance) {
        Map<Zones, PrioritizedLandmark> playbook = strategy.get(alliance);

        if (playbook == null) {
            return Optional.empty();
        }

        return activeZones.stream()
                .filter(playbook::containsKey)
                .map(playbook::get)
                .max(Comparator.comparingInt(PrioritizedLandmark::priority))
                .map(target -> target.landmark.location);
    }
}