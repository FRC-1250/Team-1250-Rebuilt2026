package frc.robot.telemetry;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.util.Color;

public class HealthMonitor {
    private static HealthMonitor instance;
    private final Map<String, Color> subsystemColors = new ConcurrentHashMap<>();
    private final Map<String, Map<String, DeviceStatus>> healthStatus = new ConcurrentHashMap<>();
    private final Map<String, Map<String, PhoenixHealthChecker>> subsystems = new ConcurrentHashMap<>();
    private final ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor();
    private final long checkInterval;
    private boolean paused = true;

    private HealthMonitor(long checkInterval) {
        this.checkInterval = checkInterval;
    }

    public static synchronized HealthMonitor getInstance() {
        if (instance == null) {
            instance = new HealthMonitor(3); // Default 3 seconds interval
        }
        return instance;
    }

    public void setSubsystemColor(String subsystemName, Color color) {
        subsystemColors.put(subsystemName, color);
    }

    public HealthMonitor addComponent(String subsystemName, String deviceName, TalonFX talonFX) {
        return addComponent(deviceName, new TalonHealthChecker(talonFX, subsystemName));
    }

    public HealthMonitor addComponent(String subsystemName, String deviceName, CANcoder cancoder) {
        return addComponent(deviceName, new CANcoderHealthChecker(cancoder, subsystemName));
    }

    public HealthMonitor addComponent(String subsystemName, String deviceName, Pigeon2 pigeon2) {
        return addComponent(deviceName, new PigeonHealthChecker(pigeon2, subsystemName));
    }

    private HealthMonitor addComponent(String deviceName, PhoenixHealthChecker e) {
        subsystems.computeIfAbsent(e.subsystemName, k -> new ConcurrentHashMap<>()).put(deviceName, e);
        healthStatus.computeIfAbsent(e.subsystemName, k -> new ConcurrentHashMap<>()).put(deviceName,
                DeviceStatus.OK);
        return this;
    }

    public void start() {
        scheduler.scheduleAtFixedRate(this::checkHealth, 0, checkInterval, TimeUnit.SECONDS);
    }

    public void pause() {
        paused = true;
    }

    public void unpause() {
        paused = false;
    }

    public void stop() {
        scheduler.shutdown();
        try {
            if (!scheduler.awaitTermination(5, TimeUnit.SECONDS)) {
                scheduler.shutdownNow();
            }
        } catch (InterruptedException e) {
            scheduler.shutdownNow();
        }
    }

    private void checkHealth() {
        if (paused)
            return;
        else {
            for (Map.Entry<String, Map<String, PhoenixHealthChecker>> subsystem : subsystems.entrySet()) {
                for (Map.Entry<String, PhoenixHealthChecker> component : subsystem.getValue().entrySet()) {
                    healthStatus.get(subsystem.getKey()).put(component.getKey(), component.getValue().runHealthCheck());
                }
            }
        }
    }

    public DeviceStatus getSubsystemStatus(String subsystemName) {
        if (paused || !healthStatus.containsKey(subsystemName)) {
            return DeviceStatus.OK;
        } else {
            DeviceStatus aggregateStatus = DeviceStatus.OK;
            for (DeviceStatus status : healthStatus.get(subsystemName).values()) {
                if (status == DeviceStatus.ERROR) {
                    return DeviceStatus.ERROR;
                }
                if (status == DeviceStatus.WARN) {
                    aggregateStatus = DeviceStatus.WARN;
                }
            }
            return aggregateStatus;
        }
    }

    public Color getSubsystemColor(String subsystemName) {
        if (!subsystemColors.containsKey(subsystemName)) {
            return Color.kBeige;
        } else {
            return subsystemColors.get(subsystemName);
        }
    }
}