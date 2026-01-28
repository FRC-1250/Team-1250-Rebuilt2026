package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.HealthMonitor;
import frc.robot.telemetry.MonitoredSubsystem;

@Logged
public class FuelLine extends SubsystemBase implements MonitoredSubsystem {

    private final TalonFX conveyorBelt;
    private final VelocityVoltage conveyorBeltVelocityControl;

    public FuelLine() {
        conveyorBelt = new TalonFX(30);
        conveyorBeltVelocityControl = new VelocityVoltage(0);

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0;
        slot0Configs.kV = 0;
        slot0Configs.kP = 0;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;

        conveyorBelt.getConfigurator().apply(talonFXConfiguration);
        conveyorBelt.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(200, Hertz));
    }

    public void setConveyorVelocity(double rotationsPerSecond, Voltage feedForward) {
        conveyorBelt.setControl(
                conveyorBeltVelocityControl
                        .withVelocity(rotationsPerSecond)
                        .withFeedForward(feedForward));
    }

    public double getConveyorBeltVelocity() {
        return conveyorBelt.getVelocity().getValueAsDouble();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void registerWithHealthMonitor(HealthMonitor monitor) {
        monitor.addComponent(getSubsystem(), "Conveyor Belt", conveyorBelt);
    }
}
