package frc.robot.utility;

public class AgitationStep {
    private double position;
    private double interval;

    public AgitationStep(double position, double interval) {
        this.position = position;
        this.interval = interval;
    }

    public double getPosition() {
        return position;
    }

    public double getInterval() {
        return interval;
    }

}
