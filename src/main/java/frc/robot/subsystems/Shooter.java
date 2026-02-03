package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
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
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

@Logged
public class Shooter extends SubsystemBase {

    TalonFX loaderCam = new TalonFX(20);
    VelocityVoltage loaderCamVelocityControl = new VelocityVoltage(0).withSlot(0);
    PositionVoltage loaderCamPositionControl = new PositionVoltage(0).withSlot(1);

    TalonFX precursorFlyWheelLeader = new TalonFX(21);
    VelocityVoltage precursorFlyWheelVelocityControl = new VelocityVoltage(0).withSlot(0);
    TalonFX precursorFlyWheelFollower = new TalonFX(22);

    TalonFX flyWheelLeader = new TalonFX(23);
    VelocityVoltage flyWheelVelocityControl = new VelocityVoltage(0).withSlot(0);
    TalonFX flyWheelFollower = new TalonFX(24);

    TalonFX hood = new TalonFX(25);
    PositionVoltage hoodPositionControl = new PositionVoltage(0).withSlot(0);
    private CANcoder hoodAbsoluteEncoder = new CANcoder(24);
    private final double encoderOffset = 0;

    public void setHoodPosition(double rotations) {
        hood.setControl(
                hoodPositionControl
                        .withPosition(rotations)
                        .withFeedForward(Volts.of(0)));
    }

    public boolean isHoodNearPosition(double position, double tolerance) {
        return MathUtil.isNear(position, getHoodPosition(), tolerance);
    }

    public double getHoodPosition() {
        return loaderCam.getPosition().getValueAsDouble();
    }

    public void setLoaderCamPosition(double rotations) {
        loaderCam.setControl(
                loaderCamPositionControl
                        .withPosition(rotations)
                        .withFeedForward(Volts.of(0)));
    }

    public void setLoaderCamVelocity(double rotationsPerSecond) {
        loaderCam.setControl(
                loaderCamVelocityControl
                        .withVelocity(rotationsPerSecond)
                        .withFeedForward(Volts.of(0)));
    }

    public void resetLoaderCamPositionToPosition(double rotations) {
        loaderCam.setPosition(rotations);
    }

    public double getLoaderCamPosition() {
        return loaderCam.getPosition().getValueAsDouble();
    }

    public double getLoaderCamVelocity() {
        return loaderCam.getVelocity().getValueAsDouble();
    }

    public boolean isLoaderCamNearPosition(double position, double tolerance) {
        return MathUtil.isNear(position, getLoaderCamPosition(), tolerance);
    }

    public void setPrecursorFlywheelVelocity(double rotationsPerSecond) {
        precursorFlyWheelLeader.setControl(
                precursorFlyWheelVelocityControl
                        .withVelocity(rotationsPerSecond)
                        .withFeedForward(Volts.of(0)));
    }

    public void setFlywheelVelocity(double rotationsPerSecond) {
        flyWheelLeader.setControl(
                flyWheelVelocityControl
                        .withVelocity(rotationsPerSecond)
                        .withFeedForward(Volts.of(0)));
    }

    public double getPreFlywheelVelocity() {
        return precursorFlyWheelLeader.getVelocity().getValueAsDouble();
    }

    public boolean isPreFlywheelNearRotationsPerSecond(double rotationsPerSecond, double tolerance) {
        return MathUtil.isNear(rotationsPerSecond, getPreFlywheelVelocity(), tolerance);
    }

    public double getFlywheelVelocity() {
        return flyWheelLeader.getVelocity().getValueAsDouble();
    }

    public boolean isFlywheelNearRotationsPerSecond(double rotationsPerSecond, double tolerance) {
        return MathUtil.isNear(rotationsPerSecond, getFlywheelVelocity(), tolerance);
    }

    public Shooter() {
        configureLoaderCam();
        configurePreFlywheel();
        configureFlywheel();
        configureEncoder();
    }

    private void configureLoaderCam() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        loaderCam.getPosition().setUpdateFrequency(Frequency.ofBaseUnits(200, Hertz));
        loaderCam.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(200, Hertz));

        Slot0Configs positionPIDConfigs = new Slot0Configs()
                .withKP(0)
                .withKI(0)
                .withKD(0);

        Slot1Configs velocityPIDConfigs = new Slot1Configs()
                .withKS(0)
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
    }

    private void configurePreFlywheel() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

        precursorFlyWheelLeader.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(200, Hertz));

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
        precursorFlyWheelLeader.getConfigurator().apply(talonFXConfiguration);
        precursorFlyWheelFollower.getConfigurator().apply(talonFXConfiguration);

        precursorFlyWheelFollower
                .setControl(new Follower(precursorFlyWheelLeader.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    private void configureFlywheel() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

        flyWheelLeader.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(200, Hertz));

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
        flyWheelLeader.getConfigurator().apply(talonFXConfiguration);
        flyWheelFollower.getConfigurator().apply(talonFXConfiguration);

        flyWheelFollower
                .setControl(new Follower(flyWheelLeader.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    private void configureEncoder() {
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
}