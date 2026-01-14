package frc.robot.utility;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;

public class JoystickToHeading {

    private double joystickDeadband;
    private Rotation2d targetHeading;
    private Rotation2d previousTargetHeading;

    public JoystickToHeading(double joystickDeadband) {
        super();
        this.joystickDeadband = joystickDeadband;
    }

    public JoystickToHeading() {
        this.joystickDeadband = 0.5;
        this.targetHeading = Rotation2d.kZero;
        this.previousTargetHeading = Rotation2d.kZero;
    }

    public void setTargetHeading(Rotation2d targetHeading) {
        this.targetHeading = targetHeading;
    }

    public Rotation2d determineHeading(
            double xInput,
            double yInput) {
        previousTargetHeading = targetHeading;

        if (Math.hypot(xInput, yInput) > joystickDeadband) {
            targetHeading = joystickInputToAngle(xInput, yInput);
            return targetHeading;
        } else {
            return previousTargetHeading;
        }
    }

    public Rotation2d determineSnapHeading(
            double xInput,
            double yInput,
            double headingTolerance,
            List<Rotation2d> headings) {
        previousTargetHeading = targetHeading;

        if (Math.hypot(xInput, yInput) > joystickDeadband) {

            Rotation2d stickAngle = joystickInputToAngle(xInput, yInput);
            Rotation2d closestHeading = targetHeading;
            double minAngularDifference = Double.POSITIVE_INFINITY;
            double difference = 0;

            // Find the closest heading
            for (Rotation2d heading : headings) {
                difference = Math.abs(heading.minus(stickAngle).getDegrees());

                if (difference < minAngularDifference) {
                    minAngularDifference = difference;
                    closestHeading = heading;
                }
            }

            if (minAngularDifference < headingTolerance) {
                targetHeading = closestHeading;
            }

            return targetHeading;
        }
        return previousTargetHeading;
    }

    private Rotation2d joystickInputToAngle(double xInput, double yInput) {
        return Rotation2d.fromRadians(Math.atan2(xInput, yInput));
    }
}
