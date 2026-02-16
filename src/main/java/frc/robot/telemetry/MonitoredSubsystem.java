package frc.robot.telemetry;

public interface MonitoredSubsystem {
    void registerWithHealthMonitor(HealthMonitor monitor);
}