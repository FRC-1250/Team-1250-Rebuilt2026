package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldZoneCircle {
    public Translation2d target;
    public double innerRadius;
    public double outerRadius;

    public FieldZoneCircle(Translation2d target, double outerRadius, double innerRadius) {
        this.target = target;
        this.innerRadius = innerRadius;
        this.outerRadius = outerRadius;
    }

    public boolean isRobotInZone(double x, double y) {
        double distanceToTarget = distanceToTarget(x, y);
        return distanceToTarget >= innerRadius && distanceToTarget < outerRadius;
    }

    public boolean isRobotInZone(Pose2d pose) {
        return isRobotInZone(pose.getX(), pose.getY());
    }

    private double distanceToTarget(double x, double y) {
        return 0;
    }

}
