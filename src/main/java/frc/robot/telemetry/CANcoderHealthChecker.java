package frc.robot.telemetry;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;

public class CANcoderHealthChecker extends PhoenixHealthChecker {

    private final StatusSignal<Boolean> bootDuringEnable;
    private final StatusSignal<Boolean> badMagnet;
    private final StatusSignal<Boolean> undervoltage;

    public CANcoderHealthChecker(CANcoder cancoder, String subsystemName) {
        super(cancoder, subsystemName);
        this.bootDuringEnable = cancoder.getFault_BootDuringEnable();
        this.badMagnet = cancoder.getFault_BadMagnet();
        this.undervoltage = cancoder.getFault_Undervoltage();
    }

    @Override
    protected DeviceStatus checkSpecificDeviceHealth() {
        BaseStatusSignal.refreshAll(bootDuringEnable, badMagnet, undervoltage);

        if (bootDuringEnable.getValue()) {
            updateAlertIfChanged("Fault: Boot During Enable");
            return DeviceStatus.ERROR;
        }
        if (badMagnet.getValue()) {
            updateAlertIfChanged("Fault: Bad Magnet");
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