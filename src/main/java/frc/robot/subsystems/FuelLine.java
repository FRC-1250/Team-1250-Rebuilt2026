package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.HealthMonitor;
import frc.robot.telemetry.MonitoredSubsystem;

@Logged
public class FuelLine extends SubsystemBase implements MonitoredSubsystem {
    public enum RollerVelocity {
        STOP(0),
        GO(25);

        public double rotationsPerSecond;

        private RollerVelocity(double rotationsPerSecond) {
            this.rotationsPerSecond = rotationsPerSecond;
        }
    }

    public enum LoaderVelocity {
        STALL(-12.5),
        FIRE(15);

        public double rotationsPerSecond;

        private LoaderVelocity(double rotationsPerSecond) {
            this.rotationsPerSecond = rotationsPerSecond;
        }
    }

    public enum LoaderCoderPosition {
        OPEN_LEFT_CENTER(0),
        OPEN_LEFT_RIGHT(0),
        OPEN_RIGHT_CENTER(0);

        public double rotations;

        private LoaderCoderPosition(double rotations) {
            this.rotations = rotations;
        }

    }

    private final TalonFX roller = new TalonFX(31);
    private final VelocityVoltage rollerVelocityControl = new VelocityVoltage(0).withSlot(0);

    private final TalonFX Loader = new TalonFX(30);
    private final VelocityVoltage LoaderVelocityControl = new VelocityVoltage(0).withSlot(0);
    private final PositionVoltage LoaderPositionControl = new PositionVoltage(0).withSlot(1);
    private final double MAGNET_OFFSET = 0;

    private final Color systemColor = new Color(0, 0, 0);

    public FuelLine() {
        configureLoader();
        configureRoller();
    }

    public double getRollerVelocity() {
        return roller.getVelocity().getValueAsDouble();
    }

    public void setRollerVelocity(double rotationsPerSecond) {
        roller.setControl(
                rollerVelocityControl
                        .withVelocity(rotationsPerSecond)
                        .withFeedForward(Volts.of(0)));
    }

    public void resetLoaderPositionToPosition(double rotations) {
        Loader.setPosition(rotations);
    }

    public boolean isNearPosition(double rotations, double tolerance) {
        return Loader.getPosition().isNear(rotations, tolerance);
    }

    public double getLoaderPosition() {
        return Loader.getPosition().getValueAsDouble();
    }

    public void setLoaderPosition(double rotations) {
        Loader.setControl(
                LoaderPositionControl
                        .withPosition(rotations)
                        .withFeedForward(Volts.of(0)));
    }

    public boolean isLoaderNearRotationsPerSecond(double rotationsPerSecond, double tolerance) {
        return Loader.getVelocity().isNear(rotationsPerSecond, tolerance);
    }

    public double getLoaderVelocity() {
        return Loader.getVelocity().getValueAsDouble();
    }

    public void setLoaderVelocity(double rotationsPerSecond) {
        Loader.setControl(
                LoaderVelocityControl
                        .withVelocity(rotationsPerSecond)
                        .withFeedForward(Volts.of(0)));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void registerWithHealthMonitor(HealthMonitor monitor) {
        monitor.addComponent(getSubsystem(), "Roller", roller);
        monitor.addComponent(getSubsystem(), "Loader", Loader);
    }

    private void configureLoader() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        Slot0Configs velocityGains = new Slot0Configs()
                .withKS(0.1)
                .withKV(0.08)
                .withKP(0.1)
                .withKI(0)
                .withKD(0);

        Slot1Configs positionGains = new Slot1Configs()
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withKS(0.1)
                .withKP(0.1)
                .withKI(0)
                .withKD(0);

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.Slot0 = velocityGains;
        talonFXConfiguration.Slot1 = positionGains;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfiguration.MotorOutput = motorOutputConfigs;
        Loader.getConfigurator().apply(talonFXConfiguration);
        Loader.getPosition().setUpdateFrequency(Frequency.ofBaseUnits(200, Hertz));
        Loader.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(200, Hertz));

        CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
        canCoderConfiguration.MagnetSensor.MagnetOffset = MAGNET_OFFSET;
        canCoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        canCoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    }

    private void configureRoller() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

        Slot0Configs velocityGains = new Slot0Configs()
                .withKS(0.1)
                .withKV(0.08)
                .withKP(0.1)
                .withKI(0)
                .withKD(0);

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.Slot0 = velocityGains;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfiguration.MotorOutput = motorOutputConfigs;

        roller.getConfigurator().apply(talonFXConfiguration);
        roller.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(200, Hertz));
    }
}
