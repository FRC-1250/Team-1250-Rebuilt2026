package frc.robot.utility;

public class FieldZones {
    public static FieldZoneRectangle BlueDepotRedOutpostNeutralZone = new FieldZoneRectangle(0, 12.0, 4.0, 8.0, 4.0);
    public static FieldZoneRectangle BlueOutpostRedDepotNeutralZone = new FieldZoneRectangle(0, 12.0, 4.0, 4.0, 0.0);
    public static FieldZoneRectangle BlueAllianceZone = new FieldZoneRectangle(0, 4.0, 0.0, 8.0, 0.0);
    public static FieldZoneRectangle RedAllianceZone = new FieldZoneRectangle(0, 16.0, 12.0, 8.0, 0.0);
    public static FieldZoneCircle AroundBlueHub = new FieldZoneCircle(0, FieldPositions.blueHub, 1.0, 0.0);
    public static FieldZoneCircle AroundRedHub = new FieldZoneCircle(0, FieldPositions.redHub, 1.0, 0.0);
}
