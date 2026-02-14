package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Volts;

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

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.HealthMonitor;
import frc.robot.telemetry.MonitoredSubsystem;

@Logged
public class FuelLine extends SubsystemBase implements MonitoredSubsystem {
    public enum RollerVelocity {
        STOP(0),
        GO(35);

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

    private final TalonFX roller = new TalonFX(31);
    private final VelocityVoltage rollerVelocityControl = new VelocityVoltage(0);;
    private final TalonFX loaderCam = new TalonFX(30);
    private final CANcoder canCoder = new CANcoder(32);
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
        return MathUtil.isNear(rotations, getLoaderCamPosition(), tolerance);
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
        return MathUtil.isNear(rotationsPerSecond, getLoaderCamVelocity(), tolerance);
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

    @Override
    public Color getSubsystemColor() {
        return systemColor;
    }

    private void configureLoaderCam() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        Slot0Configs positionPIDConfigs = new Slot0Configs()
                .withKP(0.1)
                .withKI(0)
                .withKD(0);

        Slot1Configs velocityPIDConfigs = new Slot1Configs()
                .withKS(0.1)
                .withKV(0)
                .withKP(0)
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
    }

    private void configureRoller() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0.1;
        slot0Configs.kV = 0;
        slot0Configs.kP = 0.1;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.Slot0 = slot0Configs;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfiguration.MotorOutput = motorOutputConfigs;

        roller.getConfigurator().apply(talonFXConfiguration);
        roller.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(200, Hertz));
    }
}
