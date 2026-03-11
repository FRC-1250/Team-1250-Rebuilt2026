package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldZoneCircle extends FieldZone {
    public Translation2d target;
    public double innerRadius;
    public double outerRadius;

    public FieldZoneCircle(
            double priority,
            Translation2d target,
            double outerRadius,
            double innerRadius) {
        this.target = target;
        this.innerRadius = innerRadius;
        this.outerRadius = outerRadius;
        this.priority = priority;
    }

    public FieldZoneCircle(
            double priority,
            double x,
            double y,
            double outerRadius,
            double innerRadius) {
        this(priority, new Translation2d(x, y), outerRadius, innerRadius);
    }

    @Override
    public boolean isRobotInZone(Pose2d pose) {
        return isRobotInZone(pose.getX(), pose.getY());
    }

    @Override
    public boolean isRobotInZone(double x, double y) {
        double distanceToTarget = distanceToTarget(x, y);
        return distanceToTarget >= innerRadius && distanceToTarget < outerRadius;
    }

    private double distanceToTarget(double x, double y) {
        return Math.sqrt(Math.pow(target.getX() - x, 2) + Math.pow(target.getY() - y, 2));
    }

}
