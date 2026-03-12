package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose2d;

public class FieldZoneRectangle extends FieldZone {
    private final double xUpper;
    private final double xLower;
    private final double yUpper;
    private final double yLower;

    public FieldZoneRectangle(
            double xUpper,
            double xLower,
            double yUpper,
            double yLower) {
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
