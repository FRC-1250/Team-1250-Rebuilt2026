package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose2d;

public abstract class FieldZone {

    protected double priority = Double.MAX_VALUE;

    public double getPriority() {
        return priority;
    }

    public abstract boolean isRobotInZone(Pose2d pose);

    public abstract boolean isRobotInZone(double x, double y);

}
