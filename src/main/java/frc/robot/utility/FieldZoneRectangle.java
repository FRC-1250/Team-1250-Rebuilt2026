package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose2d;

public class FieldZoneRectangle extends FieldZone {
    public double xUpper;
    public double xLower;
    public double yUpper;
    public double yLower;

    public FieldZoneRectangle(
            double priority,
            double xUpper,
            double xLower,
            double yUpper,
            double yLower) {
        this.priority = priority;
        this.xUpper = xUpper;
        this.xLower = xLower;
        this.yUpper = yUpper;
        this.yLower = yLower;
    }

    @Override
    public boolean isRobotInZone(Pose2d pose) {
        return isRobotInZone(pose.getX(), pose.getY());
    }

    @Override
    public boolean isRobotInZone(double x, double y) {
        if (x >= xLower && x < xUpper && y >= yLower && y < yUpper) {
            return true;
        } else {
            return false;
        }
    }

}
