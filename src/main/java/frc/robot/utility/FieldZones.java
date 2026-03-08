package frc.robot.utility;

import frc.robot.FieldPositions;

public class FieldZones {
    public static FieldZoneRectangle centerBlueDepotRedOutpost = new FieldZoneRectangle(12.0, 4.0, 8.0, 4.0);
    public static FieldZoneRectangle centerBlueOutpostRedDepot = new FieldZoneRectangle(12.0, 4.0, 4.0, 0.0);
    public static FieldZoneRectangle blueSide = new FieldZoneRectangle(4.0, 0.0, 8.0, 0.0);
    public static FieldZoneRectangle redSide = new FieldZoneRectangle(16.0, 12.0, 8.0, 0.0);
    public static FieldZoneCircle min = new FieldZoneCircle(FieldPositions.blueHub, 1.0, 0.0);
    public static FieldZoneCircle mid = new FieldZoneCircle(FieldPositions.blueHub, 3.0, 1.0);
    public static FieldZoneCircle semimax = new FieldZoneCircle(FieldPositions.blueHub, 3.0, 2.0);
    public static FieldZoneCircle max = new FieldZoneCircle(FieldPositions.blueHub, 4.0, 3.0);
    public static FieldZoneCircle supermax = new FieldZoneCircle(FieldPositions.blueHub, 5.0, 4.0);

}
