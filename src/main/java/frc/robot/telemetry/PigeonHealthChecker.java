package frc.robot.telemetry;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

public class PigeonHealthChecker extends PhoenixHealthChecker {

    private final StatusSignal<Boolean> bootDuringEnable;
    private final StatusSignal<Boolean> bootupGyroscope;
    private final StatusSignal<Boolean> dataAcquiredLate;
    private final StatusSignal<Boolean> undervoltage;

    public PigeonHealthChecker(Pigeon2 pigeon, String subsystemName) {
        super(pigeon, subsystemName);
        this.bootDuringEnable = pigeon.getFault_BootDuringEnable();
        this.bootupGyroscope = pigeon.getFault_BootupGyroscope();
        this.dataAcquiredLate = pigeon.getFault_DataAcquiredLate();
        this.undervoltage = pigeon.getFault_Undervoltage();
    }

    @Override
    protected DeviceStatus checkSpecificDeviceHealth() {
        BaseStatusSignal.refreshAll(bootDuringEnable, bootupGyroscope, dataAcquiredLate, undervoltage);

        if (bootDuringEnable.getValue()) {
            updateAlertIfChanged("Fault: Boot During Enable");
            return DeviceStatus.ERROR;
        }
        if (bootupGyroscope.getValue()) {
            updateAlertIfChanged("Fault: Bootup Gyroscope");
            return DeviceStatus.ERROR;
        }
        if (dataAcquiredLate.getValue()) {
            updateAlertIfChanged("Fault: Data Acquired Late");
            return DeviceStatus.ERROR;
        }
        if (undervoltage.getValue()) {
            updateAlertIfChanged("Fault: Undervoltage");
            return DeviceStatus.ERROR;
        }

        lastErrorMessage = "";
        return DeviceStatus.OK;
    }
}