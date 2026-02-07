package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose2d;

public class FieldZone {
    public Double xUpper;
    public Double xLower;
    public Double yUpper;
    public Double yLower;

    public FieldZone(Double xUpper, Double xLower, Double yUpper, Double yLower) {
        this.xUpper = xUpper;
        this.xLower = xLower;
        this.yUpper = yUpper;
        this.yLower = yLower;
    }

    public Boolean isRobotInZone(Double x, Double y) {
        if (x >= xLower && x < xUpper && y >= yLower && y < yUpper) {
            return true;
        } else {
            return false;
        }
    }

    public Boolean isRobotInZone(Pose2d pose) {
        return isRobotInZone(pose.getX(), pose.getY());
    }
}
