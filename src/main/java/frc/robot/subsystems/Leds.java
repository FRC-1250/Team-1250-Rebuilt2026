package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.HealthMonitor;
import frc.robot.telemetry.MonitoredSubsystem;

public class Leds extends SubsystemBase implements MonitoredSubsystem {

    public Leds() {

    }

    @Override
    public void registerWithHealthMonitor(HealthMonitor monitor) {

    }
}
