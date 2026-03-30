// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.HealthMonitor;
import frc.robot.telemetry.MonitoredSubsystem;
import frc.robot.utility.AgitationProfile;
import frc.robot.utility.AgitationStep;

public class Intake extends SubsystemBase implements MonitoredSubsystem {
    public enum HopperPosition {
        MIN(0),
        HOME(0.1),
        EXTENDED(4),
        MAX(4.05);

        public double rotations;

        private HopperPosition(double rotations) {
            this.rotations = rotations;
        }

    }

    public enum IntakeVelocity {
        UNJAM(-25),
        GO(80);

        public double rotationsPerSecond;

        private IntakeVelocity(double rotationsPerSecond) {
            this.rotationsPerSecond = rotationsPerSecond;
        }
    }

    private final TalonFX intakeUpper = new TalonFX(40);
    private final TalonFX intakeLower = new TalonFX(42);
    private final VelocityVoltage intakeUpperVelocityControl = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage intakeLowerVelocityControl = new VelocityVoltage(0).withSlot(0);
    private final Follower intakeLowerFollowerControl = new Follower(
            intakeUpper.getDeviceID(),
            MotorAlignmentValue.Opposed);

    private final TalonFX hopper = new TalonFX(41);
    private final PositionVoltage hopperPositionVoltage = new PositionVoltage(0).withSlot(1);

    private final Color systemColor = new Color(0, 0, 0);

    private AgitationProfile active;
    private AgitationProfile pulse;
    private AgitationProfile wiggle;
    private AgitationProfile minorWiggle;

    public Intake() {
        configureMotionMagicHopper();
        configureIntake();
        configureHopperAgitation();
        active = minorWiggle;
    }

    public void setIntakeVelocity(double rotationsPerSecond) {
        intakeUpper.setControl(
                intakeUpperVelocityControl
                        .withVelocity(rotationsPerSecond)
                        .withFeedForward(Volts.of(0)));

        intakeLower.setControl(intakeLowerFollowerControl);
    }

    public void setIntakeUpperVelocity(double rotationsPerSecond) {
        intakeUpper.setControl(
                intakeUpperVelocityControl
                        .withVelocity(rotationsPerSecond)
                        .withFeedForward(Volts.of(0)));
    }

    public void setIntakeLowerVelocity(double rotationsPerSecond) {
        intakeLower.setControl(
                intakeLowerVelocityControl
                        .withVelocity(rotationsPerSecond)
                        .withFeedForward(Volts.of(0)));
    }

    public void stopIntake() {
        intakeUpper.stopMotor();
    }

    public void agitate() {
        setHopperPosition(active.shift());
    }

    public void resetAgitation() {
        active.reset();
    }

    public void setHopperPosition(double rotations) {
        hopper.setControl(
                hopperPositionVoltage
                        .withPosition(rotations)
                        .withFeedForward(Volts.of(0)));
    }

    public void resetHopperPosition(double rotations) {
        hopper.setPosition(rotations);
    }

    public boolean isHopperAmpNearLimit() {
        return hopper.getStatorCurrent().isNear(30, 1);
    }

    public void setHopperSpeed(double speed) {
        hopper.set(speed);
    }

    public void stopHopper() {
        hopper.stopMotor();
    }

    public void setHopperNeutralMode(NeutralModeValue neutralModeValue) {
        hopper.setNeutralMode(neutralModeValue);
    }

    public boolean isHopperNearPosition(double rotations, double tolerance) {
        return hopper.getPosition().isNear(rotations, tolerance);
    }

    @Override
    public void registerWithHealthMonitor(HealthMonitor monitor) {
        monitor.addComponent(getSubsystem(), "Intake upper", intakeUpper);
        monitor.addComponent(getSubsystem(), "Intake lower", intakeLower);
        monitor.addComponent(getSubsystem(), "hopper", hopper);
        monitor.setSubsystemColor(getSubsystem(), systemColor);
    }

    private void configureHopperAgitation() {
        pulse = new AgitationProfile();
        pulse.addStep(new AgitationStep(3.7, 3));
        pulse.addStep(new AgitationStep(3.42, 0.5));
        pulse.addStep(new AgitationStep(3.09, 0.5));
        pulse.addStep(new AgitationStep(2.76, 0.5));
        pulse.addStep(new AgitationStep(2.43, 0.5));
        pulse.addStep(new AgitationStep(3.09, 1));
        pulse.addStep(new AgitationStep(2.76, 0.5));
        pulse.addStep(new AgitationStep(2.43, 0.5));
        pulse.addStep(new AgitationStep(2.10, 0.5));
        pulse.addStep(new AgitationStep(1.77, 0.5));
        pulse.addStep(new AgitationStep(2.43, 1));
        pulse.addStep(new AgitationStep(2.10, 0.5));
        pulse.addStep(new AgitationStep(1.77, 0.5));
        pulse.addStep(new AgitationStep(1.44, 0.5));
        pulse.addStep(new AgitationStep(1.25, 0.5));

        minorWiggle = new AgitationProfile();
        minorWiggle.addStep(new AgitationStep(HopperPosition.EXTENDED.rotations, 0.75));

        for (int i = 1; i <= 5; i++) {
            minorWiggle.addStep(new AgitationStep(HopperPosition.EXTENDED.rotations - (i * 0.45), 0.25));
            minorWiggle.addStep(new AgitationStep(HopperPosition.EXTENDED.rotations - (i * 0.225), 0.25));
        }

        wiggle = new AgitationProfile();
        wiggle.addStep(new AgitationStep(3.7, 3));
        wiggle.addStep(new AgitationStep(3.2, 0.5));
        wiggle.addStep(new AgitationStep(3.40, 0.5));
        wiggle.addStep(new AgitationStep(2.9, 1));
        wiggle.addStep(new AgitationStep(3.1, 0.5));
        wiggle.addStep(new AgitationStep(2.6, 0.5));
        wiggle.addStep(new AgitationStep(2.8, 1));
        wiggle.addStep(new AgitationStep(2.3, 0.5));
        wiggle.addStep(new AgitationStep(2.5, 0.5));
        wiggle.addStep(new AgitationStep(2.0, 1));
        wiggle.addStep(new AgitationStep(2.2, 0.5));
        wiggle.addStep(new AgitationStep(1.7, 0.5));
        wiggle.addStep(new AgitationStep(1.90, 0.5));
        wiggle.addStep(new AgitationStep(1.25, 0.5));
    }

    private void configureIntake() {
        MotorOutputConfigs motorOutputConfigsUpper = new MotorOutputConfigs();
        motorOutputConfigsUpper.NeutralMode = NeutralModeValue.Coast;
        motorOutputConfigsUpper.Inverted = InvertedValue.Clockwise_Positive;

        MotorOutputConfigs motorOutputConfigsLower = new MotorOutputConfigs();
        motorOutputConfigsLower.NeutralMode = NeutralModeValue.Coast;
        motorOutputConfigsLower.Inverted = InvertedValue.Clockwise_Positive;

        Slot0Configs velocityGains = new Slot0Configs()
                .withKS(0.1)
                .withKV(0.11)
                .withKP(0.5)
                .withKI(0)
                .withKD(0);

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.Slot0 = velocityGains;
        // Try default limits. Supply = 70 amp, stator = 120 amp, reduce supply to 40
        // amps after 1 second
        // talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 70;
        // talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

        talonFXConfiguration.MotorOutput = motorOutputConfigsUpper;
        intakeUpper.getConfigurator().apply(talonFXConfiguration);
        intakeUpper.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(100, Hertz));
        intakeUpper.optimizeBusUtilization();

        talonFXConfiguration.MotorOutput = motorOutputConfigsLower;
        intakeLower.getConfigurator().apply(talonFXConfiguration);
        intakeLower.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(100, Hertz));
        intakeLower.optimizeBusUtilization();
    }

    private void configureMotionMagicHopper() {
        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.Voltage.PeakReverseVoltage = -12;
        talonFXConfiguration.Voltage.PeakForwardVoltage = 12;

        MotorOutputConfigs motorOutputConfigs = talonFXConfiguration.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

        Slot1Configs positionGains = talonFXConfiguration.Slot1;
        positionGains.GravityType = GravityTypeValue.Elevator_Static;
        positionGains.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        positionGains.kS = 0.4; // output to overcome static friction (output)
        positionGains.kV = 0.15; // output per unit of target velocity (output/rps)
        positionGains.kG = -0.7;
        positionGains.kA = 0; // output per unit of target acceleration (output/(rps/s))
        positionGains.kP = 2; // output per unit of error in position (output/rotation)
        positionGains.kI = 0; // output per unit of integrated error in position (output/(rotation*s))
        positionGains.kD = 0.01; // output per unit of error in velocity (output/rps)

        CurrentLimitsConfigs currentLimitsConfigs = talonFXConfiguration.CurrentLimits;
        currentLimitsConfigs.SupplyCurrentLimit = 20;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;

        SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = talonFXConfiguration.SoftwareLimitSwitch;
        softwareLimitSwitchConfigs.ForwardSoftLimitEnable = true;
        softwareLimitSwitchConfigs.ForwardSoftLimitThreshold = 4;
        softwareLimitSwitchConfigs.ReverseSoftLimitEnable = true;
        softwareLimitSwitchConfigs.ReverseSoftLimitThreshold = 0;

        MotionMagicConfigs motionMagicConfigs = talonFXConfiguration.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 8;
        motionMagicConfigs.MotionMagicAcceleration = 64;
        motionMagicConfigs.MotionMagicAcceleration = 256;

        hopper.getConfigurator().apply(talonFXConfiguration);
        hopper.setPosition(0);
        hopper.getPosition().setUpdateFrequency(Frequency.ofBaseUnits(100, Hertz));
        hopper.optimizeBusUtilization();
    }

}
