package frc.robot.utility;

import frc.robot.FieldPositions;

public class FieldZones {
    public static FieldZone centerBlueDepotRedOutpost = new FieldZone(12.0, 4.0, 8.0, 4.0);
    public static FieldZone centerBlueOutpostRedDepot = new FieldZone(12.0, 4.0, 4.0, 0.0);
    public static FieldZone blueSide = new FieldZone(4.0, 0.0, 8.0, 0.0);
    public static FieldZone redSide = new FieldZone(16.0, 12.0, 8.0, 0.0);
    public static CircleFeildZones min = new CircleFeildZones(FieldPositions.blueHub, 1.0, 0.0);
    public static CircleFeildZones mid = new CircleFeildZones(FieldPositions.blueHub, 3.0, 1.0);
    public static CircleFeildZones semimax = new CircleFeildZones(FieldPositions.blueHub, 3.0, 2.0);
    public static CircleFeildZones max = new CircleFeildZones(FieldPositions.blueHub, 4.0, 3.0);
    public static CircleFeildZones supermax = new CircleFeildZones(FieldPositions.blueHub, 5.0, 4.0);

}
