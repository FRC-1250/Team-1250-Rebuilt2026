package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.HealthMonitor;
import frc.robot.telemetry.MonitoredSubsystem;

@Logged
public class Shooter extends SubsystemBase implements MonitoredSubsystem {

    public enum HoodPosition {
        HOME(0.01),
        MAX(1);

        public double rotations;

        HoodPosition(double rotations) {
            this.rotations = rotations;
        }
    }

    public enum ShooterVelocity {
        STOP(0),
        WARM(10),
        HUB(20),
        MAX(40);

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

    private final TalonFX hood = new TalonFX(25);
    private final PositionVoltage hoodPositionControl = new PositionVoltage(0).withSlot(0);
    private final CANcoder hoodAbsoluteEncoder = new CANcoder(25);
    private final double encoderOffset = 0;

    private final Color systemColor = new Color(0, 0, 0);

    public Shooter() {
        configureAccelerator();
        configureShooter();
        configureHood();
    }

    public double getHoodPosition() {
        return hood.getPosition().getValueAsDouble();
    }

    public void setHoodPosition(double rotations) {
        hood.setControl(
                hoodPositionControl
                        .withPosition(rotations)
                        .withFeedForward(Volts.of(0)));
    }

    public boolean isHoodNearPosition(double position, double tolerance) {
        return MathUtil.isNear(position, getHoodPosition(), tolerance);
    }

    public double getAcceleratorVelocity() {
        return acceleratorLeader.getVelocity().getValueAsDouble();
    }

    public void setAcceleratorVelocity(double rotationsPerSecond) {
        acceleratorLeader.setControl(
                acceleratorVelocityControl
                        .withVelocity(rotationsPerSecond)
                        .withFeedForward(Volts.of(0)));
    }

    public boolean isAcceleratorNearRotationsPerSecond(double rotationsPerSecond, double tolerance) {
        return MathUtil.isNear(rotationsPerSecond, getAcceleratorVelocity(), tolerance);
    }

    public double getShooterVelocity() {
        return shooterLeader.getVelocity().getValueAsDouble();
    }

    public void setShooterVelocity(double rotationsPerSecond) {
        shooterLeader.setControl(
                shooterVelocityControl
                        .withVelocity(rotationsPerSecond)
                        .withFeedForward(Volts.of(0)));
    }

    public boolean isShooterNearRotationsPerSecond(double rotationsPerSecond, double tolerance) {
        return MathUtil.isNear(rotationsPerSecond, getShooterVelocity(), tolerance);
    }

    private void configureAccelerator() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

        acceleratorLeader.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(200, Hertz));

        Slot1Configs velocityPIDConfigs = new Slot1Configs()
                .withKS(0)
                .withKV(0)
                .withKP(0)
                .withKI(0)
                .withKD(0);

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.Slot1 = velocityPIDConfigs;
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
        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

        shooterLeader.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(200, Hertz));

        Slot1Configs velocityPIDConfigs = new Slot1Configs()
                .withKS(0)
                .withKV(0)
                .withKP(0)
                .withKI(0)
                .withKD(0);

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.Slot1 = velocityPIDConfigs;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfiguration.MotorOutput = motorOutputConfigs;
        shooterLeader.getConfigurator().apply(talonFXConfiguration);
        shooterFollower.getConfigurator().apply(talonFXConfiguration);

        shooterFollower
                .setControl(new Follower(shooterLeader.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    private void configureHood() {
        CANcoderConfiguration hoodAbsoluteEncoderConfiguration = new CANcoderConfiguration();
        // Set encoder to provide a value between 0 and 1
        hoodAbsoluteEncoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        hoodAbsoluteEncoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        hoodAbsoluteEncoderConfiguration.MagnetSensor.MagnetOffset = encoderOffset;
        hoodAbsoluteEncoder.getConfigurator().apply(hoodAbsoluteEncoderConfiguration);
        hoodAbsoluteEncoder.getAbsolutePosition().setUpdateFrequency(200);

        Slot0Configs hoodPositionPIDConfigs = new Slot0Configs()
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKG(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withKS(0)
                .withKP(0)
                .withKI(0)
                .withKD(0);

        TalonFXConfiguration hoodConfiguration = new TalonFXConfiguration();
        hoodConfiguration.Slot0 = hoodPositionPIDConfigs;
        hoodConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
        hoodConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        hoodConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hoodConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        hood.getConfigurator().apply(hoodConfiguration);
    }

    @Override
    public void registerWithHealthMonitor(HealthMonitor monitor) {
        monitor.addComponent(getSubsystem(), "Hood", hood);
        monitor.addComponent(getSubsystem(), "Hood encoder", hoodAbsoluteEncoder);
        monitor.addComponent(getSubsystem(), "Accelerator leader", acceleratorLeader);
        monitor.addComponent(getSubsystem(), "Accelerator follower", acceleratorFollower);
        monitor.addComponent(getSubsystem(), "Shooter leader", shooterLeader);
        monitor.addComponent(getSubsystem(), "Shooter follower", shooterFollower);
    }

    @Override
    public Color getSubsystemColor() {
        return systemColor;
    }
}