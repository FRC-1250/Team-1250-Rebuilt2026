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
import com.ctre.phoenix6.signals.GravityTypeValue;
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

    public enum LoaderCamVelocity {
        THREE_BPS(1),
        SIX_BPS(2),
        NINE_BPS(3),
        TWELVE_BPS(4);

        public double rotationsPerSecond;

        private LoaderCamVelocity(double rotationsPerSecond) {
            this.rotationsPerSecond = rotationsPerSecond;
        }
    }

    public enum CamCoderPosition {
        OPEN_LEFT_CENTER(0),
        OPEN_LEFT_RIGHT(0),
        OPEN_RIGHT_CENTER(0);

        public double rotations;

        private CamCoderPosition(double rotations) {
            this.rotations = rotations;
        }

    }

    private final TalonFX roller = new TalonFX(31);
    private final VelocityVoltage rollerVelocityControl = new VelocityVoltage(0);

    private final TalonFX loaderCam = new TalonFX(30);
    private final CANcoder camCoder = new CANcoder(32);
    private final double MAGNET_OFFSET = 0;
    private final VelocityVoltage loaderCamVelocityControl = new VelocityVoltage(0).withSlot(1);
    private final PositionVoltage loaderCamPositionControl = new PositionVoltage(0).withSlot(0);

    private final Color systemColor = new Color(0, 0, 0);

    public FuelLine() {
        configureLoaderCam();
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

    public void resetLoaderCamPositionToPosition(double rotations) {
        loaderCam.setPosition(rotations);
    }

    public boolean isLoaderCamNearPosition(double rotations, double tolerance) {
        return loaderCam.getPosition().isNear(rotations, tolerance);
    }

    public double getLoaderCamPosition() {
        return loaderCam.getPosition().getValueAsDouble();
    }

    public void setLoaderCamPosition(double rotations) {
        loaderCam.setControl(
                loaderCamPositionControl
                        .withPosition(rotations)
                        .withFeedForward(Volts.of(0)));
    }

    public boolean isLoaderCamNearRotationsPerSecond(double rotationsPerSecond, double tolerance) {
        return loaderCam.getVelocity().isNear(rotationsPerSecond, tolerance);
    }

    public double getLoaderCamVelocity() {
        return loaderCam.getVelocity().getValueAsDouble();
    }

    public void setLoaderCamVelocity(double rotationsPerSecond) {
        loaderCam.setControl(
                loaderCamVelocityControl
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
        monitor.addComponent(getSubsystem(), "Loader cam", loaderCam);
    }

    private void configureLoaderCam() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

        Slot0Configs positionPIDConfigs = new Slot0Configs()
                .withGravityType(GravityTypeValue.Elevator_Static)
                .withKG(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withKS(0.1)
                .withKP(0.1)
                .withKI(0)
                .withKD(0);

        Slot1Configs velocityPIDConfigs = new Slot1Configs()
                .withKS(0.1)
                .withKV(0.08)
                .withKP(0.1)
                .withKI(0)
                .withKD(0);

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.Slot0 = positionPIDConfigs;
        talonFXConfiguration.Slot1 = velocityPIDConfigs;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfiguration.MotorOutput = motorOutputConfigs;
        loaderCam.getConfigurator().apply(talonFXConfiguration);
        loaderCam.getPosition().setUpdateFrequency(Frequency.ofBaseUnits(200, Hertz));
        loaderCam.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(200, Hertz));

        CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
        canCoderConfiguration.MagnetSensor.MagnetOffset = MAGNET_OFFSET;
        canCoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        canCoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        camCoder.getConfigurator().apply(canCoderConfiguration);
        camCoder.getAbsolutePosition().setUpdateFrequency(Frequency.ofBaseUnits(200, Hertz));
    }

    private void configureRoller() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        Slot0Configs slot0Configs = new Slot0Configs()
                .withKS(0.1)
                .withKV(0.08)
                .withKP(0.1)
                .withKI(0)
                .withKD(0);

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.Slot0 = slot0Configs;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfiguration.MotorOutput = motorOutputConfigs;

        roller.getConfigurator().apply(talonFXConfiguration);
        roller.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(200, Hertz));
    }
}
