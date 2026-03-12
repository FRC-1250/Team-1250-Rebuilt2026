package frc.robot.utility;

import edu.wpi.first.math.geometry.Translation2d;

public class FieldLocalization {
    public enum Landmark {
        BLUE_HUB(new Translation2d(4.605909, 4.027932)),
        BLUE_DEPOT(new Translation2d(1.45, 6.7)),
        BLUE_OUTPOST(new Translation2d(1.45, 1.48)),
        RED_HUB(new Translation2d(11.8745, 4.027932)),
        RED_DEPOT(new Translation2d(14.596, 1.48)),
        RED_OUTPOST(new Translation2d(14.596, 6.801));

        public final Translation2d location;

        Landmark(Translation2d location) {
            this.location = location;
        }
    }

    public enum Zones {
        BLUE_DEPOT_RED_OUTPOST_NEUTRAL_ZONE(
                new FieldZoneRectangle(
                        11.8618,
                        4.0132,
                        8.0,
                        4.0)),

        BLUE_OUTPOST_RED_DEPOT_NEUTRAL_ZONE(
                new FieldZoneRectangle(
                        11.8618,
                        4.0132,
                        4.0,
                        0.0)),

        BLUE_ALLIANCE_ZONE(
                new FieldZoneRectangle(
                        4.0132,
                        0.0,
                        8.2296,
                        0.0)),

        RED_ALLIANCE_ZONE(
                new FieldZoneRectangle(
                        16.4592,
                        11.8618,
                        8.2296,
                        0.0));

        public final FieldZone area;

        Zones(FieldZone area) {
            this.area = area;
        }
    }
}
