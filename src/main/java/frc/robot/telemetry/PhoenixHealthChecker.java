package frc.robot.telemetry;

import com.ctre.phoenix6.hardware.ParentDevice;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public abstract class PhoenixHealthChecker {
    private final ParentDevice device;
    private final int deviceID;
    public final String subsystemName;
    private final Alert alert;

    private DeviceStatus status = DeviceStatus.OK;
    protected String lastErrorMessage = "";

    public PhoenixHealthChecker(final ParentDevice device, final String subsystemName) {
        this.device = device;
        this.deviceID = device.getDeviceID();
        this.subsystemName = subsystemName;
        this.alert = new Alert("", AlertType.kError);
    }

    public DeviceStatus runHealthCheck() {
        status = checkDeviceHealth();
        if (status == DeviceStatus.OK) {
            alert.set(false);
        } else {
            alert.set(true);
        }
        return status;
    }

    protected abstract DeviceStatus checkSpecificDeviceHealth();

    protected void updateAlertIfChanged(String newError) {
        if (!newError.equals(lastErrorMessage)) {
            lastErrorMessage = newError;
            alert.setText(String.format("[%s] ID %d: %s", subsystemName, deviceID, newError));
        }
    }

    private DeviceStatus checkDeviceHealth() {
        if (!device.isConnected()) {
            updateAlertIfChanged("Disconnected: Check power/CAN connections.");
            return DeviceStatus.ERROR;
        }
        return checkSpecificDeviceHealth();
    }
}