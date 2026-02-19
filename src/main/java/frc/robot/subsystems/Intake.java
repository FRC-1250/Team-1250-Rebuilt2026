// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.HealthMonitor;
import frc.robot.telemetry.MonitoredSubsystem;

@Logged
public class Intake extends SubsystemBase implements MonitoredSubsystem {
    public enum HopperPosition {
        RETRACTED(0.1),
        PHASE_1(2),
        PHASE_2(2.5),
        PHASE_3(3),
        EXTENDED(3.7);

        public double rotations;

        private HopperPosition(double rotations) {
            this.rotations = rotations;
        }

    }

    public enum IntakeVelocity {
        STOP(0),
        GO(75);

        public double rotationsPerSecond;

        private IntakeVelocity(double rotationsPerSecond) {
            this.rotationsPerSecond = rotationsPerSecond;
        }
    }

    private class HopperAgitationStep {
        HopperPosition hopperPosition;
        double interval;

        public HopperAgitationStep(HopperPosition hopperPosition, double interval) {
            this.hopperPosition = hopperPosition;
            this.interval = interval;
        }
    }

    private class HopperAgitationProfile {
        private Timer timer;
        private List<HopperAgitationStep> steps;
        private int index;

        public HopperAgitationProfile() {
            this.steps = new ArrayList<>();
            this.timer = new Timer();
            index = 0;
        }

        public void addStep(HopperAgitationStep has) {
            if (has.interval > 0)
                this.steps.add(has);
        }

        public void reset() {
            timer.stop();
            timer.reset();
            index = 0;
        }

        public HopperPosition shift() {
            if (!timer.isRunning()) {
                timer.start();
            }

            var currentStep = steps.get(index % steps.size());

            if (timer.advanceIfElapsed(currentStep.interval)) {
                index++;
                currentStep = steps.get(index % steps.size());
                timer.reset();
            }

            return currentStep.hopperPosition;
        }
    }

    private final TalonFX intake = new TalonFX(40);
    private final VelocityVoltage intakeVelocityControl = new VelocityVoltage(0).withSlot(0);

    private final TalonFX hopper = new TalonFX(41);
    private final PositionVoltage hopperPositionVoltage = new PositionVoltage(0).withSlot(1);

    private final Color systemColor = new Color(0, 0, 0);

    private HopperAgitationProfile activeAgitiationProfile;
    private HopperAgitationProfile wave;

    public Intake() {
        configureHopper();
        configureIntake();
        configureHopperAgitation();
        activeAgitiationProfile = wave;
    }

    public void agitateHopper() {
        activeAgitiationProfile.shift();
    }

    public void resetAgitation() {
        activeAgitiationProfile.reset();
    }

    public void setIntakeVelocity(double rotationsPerSecond) {
        intake.setControl(
                intakeVelocityControl
                        .withVelocity(rotationsPerSecond)
                        .withFeedForward(Volts.of(0)));
    }

    public double getIntakeVelocity() {
        return intake.getVelocity().getValueAsDouble();
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

    public double getHopperPosition() {
        return hopper.getPosition().getValueAsDouble();
    }

    public boolean isHopperNearPosition(double rotations, double tolerance) {
        return hopper.getPosition().isNear(rotations, tolerance);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void registerWithHealthMonitor(HealthMonitor monitor) {
        monitor.addComponent(getSubsystem(), "Intake", intake);
        monitor.addComponent(getSubsystem(), "hopper", hopper);
    }

    private void configureHopperAgitation() {
        wave = new HopperAgitationProfile();
        wave.addStep(new HopperAgitationStep(HopperPosition.PHASE_3, 1));
        wave.addStep(new HopperAgitationStep(HopperPosition.PHASE_2, 1));
        wave.addStep(new HopperAgitationStep(HopperPosition.PHASE_1, 1));
        wave.addStep(new HopperAgitationStep(HopperPosition.PHASE_2, 1));
        wave.addStep(new HopperAgitationStep(HopperPosition.PHASE_3, 1));
    }

    private void configureIntake() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        Slot0Configs velocityGains = new Slot0Configs()
                .withKS(0.1)
                .withKV(0.11)
                .withKP(0.1)
                .withKI(0)
                .withKD(0);

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.Slot0 = velocityGains;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfiguration.MotorOutput = motorOutputConfigs;

        intake.getConfigurator().apply(talonFXConfiguration);
        intake.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(200, Hertz));
    }

    private void configureHopper() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

        Slot1Configs positionGains = new Slot1Configs()
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withKS(0)
                .withKP(8)
                .withKI(0)
                .withKD(0);

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.Slot1 = positionGains;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfiguration.MotorOutput = motorOutputConfigs;
        // talonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        talonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 3.725;
        // talonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        talonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        talonFXConfiguration.Voltage.PeakForwardVoltage = 0.25;
        talonFXConfiguration.Voltage.PeakReverseVoltage = -12;

        hopper.setPosition(0);
        hopper.getConfigurator().apply(talonFXConfiguration);
        hopper.getPosition().setUpdateFrequency(Frequency.ofBaseUnits(200, Hertz));
    }

}
