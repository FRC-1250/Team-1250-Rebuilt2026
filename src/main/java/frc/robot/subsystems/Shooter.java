package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.HealthMonitor;
import frc.robot.telemetry.MonitoredSubsystem;

public class Shooter extends SubsystemBase implements MonitoredSubsystem {

    public enum ShooterVelocity {
        WARM(10),
        HUB(40),
        MAX(50);

        public double shooterRotationsPerSecond;
        public double acceleratorRotationsPerSecond;
        private double feedForwardPercentage = 0.10;

        ShooterVelocity(double rotationsPerSecond) {
            this.shooterRotationsPerSecond = rotationsPerSecond;
            this.acceleratorRotationsPerSecond = rotationsPerSecond + (rotationsPerSecond * feedForwardPercentage);
        }
    }

    private final TalonFX acceleratorLeader = new TalonFX(21);
    private final TalonFX acceleratorFollower = new TalonFX(22);
    private final VelocityVoltage acceleratorVelocityControl = new VelocityVoltage(0).withSlot(0);

    private final TalonFX shooterLeader = new TalonFX(23);
    private final TalonFX shooterFollower = new TalonFX(24);
    private final VelocityVoltage shooterVelocityControl = new VelocityVoltage(0).withSlot(0);

    private final Color systemColor = new Color(0, 0, 0);

    public Shooter() {
        configureAccelerator();
        configureShooter();
    }

    public void setAcceleratorVelocity(double rotationsPerSecond) {
        acceleratorLeader.setControl(
                acceleratorVelocityControl
                        .withVelocity(rotationsPerSecond)
                        .withFeedForward(Volts.of(0)));
    }

    public boolean isAcceleratorNearRotationsPerSecond(double rotationsPerSecond, double tolerance) {
        return acceleratorLeader.getVelocity().isNear(rotationsPerSecond, tolerance);
    }

    public void setShooterVelocity(double rotationsPerSecond) {
        shooterLeader.setControl(
                shooterVelocityControl
                        .withVelocity(rotationsPerSecond)
                        .withFeedForward(Volts.of(0)));
    }

    public boolean isShooterNearRotationsPerSecond(double rotationsPerSecond, double tolerance) {
        return shooterLeader.getVelocity().isNear(rotationsPerSecond, tolerance);
    }

    public void stopShooter() {
        shooterLeader.stopMotor();
    }

    public void stopAccelerator() {
        acceleratorLeader.stopMotor();
    }

    @Logged(name = "Shooter velocity")
    public double getShooterVelocity() {
        return shooterLeader.getVelocity().getValueAsDouble();
    }

    @Logged(name = "Shooter leader stator current")
    public double getShooterLeaderStatorCurrent() {
        return shooterLeader.getStatorCurrent().getValueAsDouble();
    }

    @Logged(name = "Shooter leader supply current")
    public double getShooterLeaderSupplyCurrent() {
        return shooterLeader.getSupplyCurrent().getValueAsDouble();
    }

    @Logged(name = "Shooter follower stator current")
    public double getShooterFollowerStatorCurrent() {
        return shooterFollower.getStatorCurrent().getValueAsDouble();
    }

    @Logged(name = "Shooter follower supply current")
    public double getShooterFollowerSupplyCurrent() {
        return shooterFollower.getSupplyCurrent().getValueAsDouble();
    }

    @Logged(name = "Accelerator velocity")
    public double getAcceleratorVelocity() {
        return acceleratorLeader.getVelocity().getValueAsDouble();
    }

    @Logged(name = "Accelerator leader stator current")
    public double getAcceleratorLeaderStatorCurrent() {
        return acceleratorLeader.getStatorCurrent().getValueAsDouble();
    }

    @Logged(name = "Accelerator leader supply current")
    public double getAcceleratorLeaderSupplyCurrent() {
        return acceleratorLeader.getSupplyCurrent().getValueAsDouble();
    }

    @Logged(name = "Accelerator follower stator current")
    public double getAcceleratorFollowerStatorCurrent() {
        return acceleratorFollower.getStatorCurrent().getValueAsDouble();
    }

    @Logged(name = "Accelerator follower supply current")
    public double getAcceleratorFollowerSupplyCurrent() {
        return acceleratorFollower.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void registerWithHealthMonitor(HealthMonitor monitor) {
        monitor.addComponent(getSubsystem(), "Accelerator leader", acceleratorLeader);
        monitor.addComponent(getSubsystem(), "Accelerator follower", acceleratorFollower);
        monitor.addComponent(getSubsystem(), "Shooter leader", shooterLeader);
        monitor.addComponent(getSubsystem(), "Shooter follower", shooterFollower);
        monitor.setSubsystemColor(getSubsystem(), systemColor);
    }

    private void configureAccelerator() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        acceleratorLeader.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(200, Hertz));

        Slot0Configs velocityGains = new Slot0Configs()
                .withKS(0.09)
                .withKV(0.11)
                .withKP(0.15)
                .withKI(0)
                .withKD(0);

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.Slot0 = velocityGains;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfiguration.MotorOutput = motorOutputConfigs;
        acceleratorLeader.getConfigurator().apply(talonFXConfiguration);
        acceleratorFollower.getConfigurator().apply(talonFXConfiguration);

        acceleratorFollower
                .setControl(new Follower(acceleratorLeader.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    private void configureShooter() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        shooterLeader.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(200, Hertz));

        Slot0Configs velocityGains = new Slot0Configs()
                .withKS(0.09)
                .withKV(0.11)
                .withKP(0.25)
                .withKI(0)
                .withKD(0.01);

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.Slot0 = velocityGains;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfiguration.MotorOutput = motorOutputConfigs;
        shooterLeader.getConfigurator().apply(talonFXConfiguration);
        shooterFollower.getConfigurator().apply(talonFXConfiguration);

        shooterFollower
                .setControl(new Follower(shooterLeader.getDeviceID(), MotorAlignmentValue.Opposed));
    }
}