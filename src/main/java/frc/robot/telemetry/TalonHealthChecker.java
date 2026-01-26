package frc.robot.telemetry;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Temperature;

public class TalonHealthChecker extends PhoenixHealthChecker {

    private final StatusSignal<Temperature> deviceTemp;

    private final StatusSignal<Boolean> bootDuringEnable;
    private final StatusSignal<Boolean> remoteSensorDataInvalid;
    private final StatusSignal<Boolean> remoteSensorReset;

    private final StatusSignal<Boolean> oversupplyV;
    private final StatusSignal<Boolean> underVoltage;
    private final StatusSignal<Boolean> unstableSupplyV;

    public TalonHealthChecker(TalonFX talon, String subsystemName) {
        super(talon, subsystemName);
        this.deviceTemp = talon.getDeviceTemp();
        this.bootDuringEnable = talon.getFault_BootDuringEnable();
        this.remoteSensorDataInvalid = talon.getFault_RemoteSensorDataInvalid();
        this.remoteSensorReset = talon.getFault_RemoteSensorReset();
        this.oversupplyV = talon.getFault_OverSupplyV();
        this.underVoltage = talon.getFault_Undervoltage();
        this.unstableSupplyV = talon.getFault_UnstableSupplyV();
    }

    @Override
    protected DeviceStatus checkSpecificDeviceHealth() {
        BaseStatusSignal.refreshAll(
                deviceTemp, bootDuringEnable, oversupplyV,
                remoteSensorDataInvalid, remoteSensorReset, underVoltage, unstableSupplyV);

        if (bootDuringEnable.getValue()) {
            updateAlertIfChanged("Fault: Boot During Enable");
            return DeviceStatus.ERROR;
        }

        if (remoteSensorDataInvalid.getValue()) {
            updateAlertIfChanged("Fault: Remote Sensor Data Invalid");
            return DeviceStatus.ERROR;
        }

        if (remoteSensorReset.getValue()) {
            updateAlertIfChanged("Fault: Remote Sensor Reset");
            return DeviceStatus.ERROR;
        }

        if (unstableSupplyV.getValue()) {
            updateAlertIfChanged("Fault: Unstable Supply Voltage");
            return DeviceStatus.ERROR;
        }

        if (oversupplyV.getValue()) {
            updateAlertIfChanged("Fault: Over Supply Voltage");
            return DeviceStatus.ERROR;
        }

        if (underVoltage.getValue()) {
            updateAlertIfChanged("Fault: Undervoltage");
            return DeviceStatus.ERROR;
        }

        if (deviceTemp.getValueAsDouble() > 75.0) {
            updateAlertIfChanged("Motor Overheating (" + deviceTemp.getValueAsDouble() + "C)");
            return DeviceStatus.WARN;
        }

        lastErrorMessage = "";
        return DeviceStatus.OK;
    }
}
