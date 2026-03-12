package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldZoneCircle extends FieldZone {
    private final Translation2d target;
    private final double radius;

    public FieldZoneCircle(
            Translation2d target,
            double radius) {
        this.target = target;
        this.radius = radius;
    }

    public FieldZoneCircle(
            double x,
            double y,
            double radius) {
        this(new Translation2d(x, y), radius);
    }

    @Override
    public boolean isRobotInZone(Pose2d pose) {
        return isRobotInZone(pose.getX(), pose.getY());
    }

    @Override
    public boolean isRobotInZone(double x, double y) {
        return distanceToTarget(x, y) <= radius;
    }

    private double distanceToTarget(double x, double y) {
        return Math.sqrt(Math.pow(target.getX() - x, 2) + Math.pow(target.getY() - y, 2));
    }

}
