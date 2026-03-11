package frc.robot.utility;

import java.util.Comparator;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldZoneToTarget {
    private static final Map<FieldZone, Translation2d> blueTargetMap = new HashMap<>();
    private static final Map<FieldZone, Translation2d> redTargetMap = new HashMap<>();

    static {
        blueTargetMap.put(FieldZones.BlueAllianceZone, FieldPositions.blueHub);
        blueTargetMap.put(FieldZones.BlueDepotRedOutpostNeutralZone, FieldPositions.blueDepotSide);
        blueTargetMap.put(FieldZones.BlueOutpostRedDepotNeutralZone, FieldPositions.blueOutpostSide);

        redTargetMap.put(FieldZones.RedAllianceZone, FieldPositions.redHub);
        redTargetMap.put(FieldZones.BlueDepotRedOutpostNeutralZone, FieldPositions.redOutpostSide);
        redTargetMap.put(FieldZones.BlueOutpostRedDepotNeutralZone, FieldPositions.redDepotSide);
    }

    public static Optional<Translation2d> getTarget(Pose2d pose) {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Map<FieldZone, Translation2d> currentMap;

        if (alliance == Alliance.Blue) {
            currentMap = blueTargetMap;
        } else {
            currentMap = redTargetMap;
        }

        return currentMap.keySet().stream()
                .filter(zone -> zone.isRobotInZone(pose))
                .min(Comparator.comparingDouble(FieldZone::getPriority))
                .map(currentMap::get);
    }

}
