package frc.robot.utility;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class RumbleStep {
    private RumbleType rumbleType;
    private double intensity;
    private double interval;

    public RumbleStep(RumbleType rumbleType, double intensity, double interval) {
        this.rumbleType = rumbleType;
        this.intensity = intensity;
        this.interval = interval;
    }

    public double getIntensity() {
        return intensity;
    }

    public double getInterval() {
        return interval;
    }

    public RumbleType getRumbleType() {
        return rumbleType;
    }
}
