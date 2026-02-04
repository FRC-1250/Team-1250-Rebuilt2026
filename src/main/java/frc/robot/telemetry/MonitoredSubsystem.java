package frc.robot.telemetry;

import edu.wpi.first.wpilibj.util.Color;

public interface MonitoredSubsystem {
    void registerWithHealthMonitor(HealthMonitor monitor);

    Color getSubsystemColor();
}