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
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.HealthMonitor;
import frc.robot.telemetry.MonitoredSubsystem;

public class Intake extends SubsystemBase implements MonitoredSubsystem {
    public enum HopperPosition {
        RETRACTED(0),
        PHASE_1(2),
        PHASE_2(3),
        PHASE_3(4),
        EXTENDED(10);

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

    private final TalonFX hopper = new TalonFX(41);
    private final PositionVoltage hopperPositionVoltage = new PositionVoltage(0);
    private final TalonFX intake = new TalonFX(40);
    private final VelocityVoltage intakeVelocityControl = new VelocityVoltage(0);
    private final Color systemColor = new Color(0, 0, 0);

    private HopperAgitationProfile activeAgitiationProfile;
    private HopperAgitationProfile wave;

    public Intake() {
        configureHopperTalonFX();
        configureIntakeTalonFX();
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

    public double getHopperPosition() {
        return hopper.getPosition().getValueAsDouble();
    }

    public boolean isHopperNearPosition(double rotations, double tolerance) {
        return MathUtil.isNear(rotations, getHopperPosition(), tolerance);
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

    @Override
    public Color getSubsystemColor() {
        return systemColor;
    }

    private void configureHopperAgitation() {
        wave = new HopperAgitationProfile();
        wave.addStep(new HopperAgitationStep(HopperPosition.PHASE_3, 1));
        wave.addStep(new HopperAgitationStep(HopperPosition.PHASE_2, 1));
        wave.addStep(new HopperAgitationStep(HopperPosition.PHASE_1, 1));
        wave.addStep(new HopperAgitationStep(HopperPosition.PHASE_2, 1));
        wave.addStep(new HopperAgitationStep(HopperPosition.PHASE_3, 1));
    }

    private void configureIntakeTalonFX() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        Slot0Configs slot0Configs = new Slot0Configs()
                .withKS(0.1)
                .withKV(0)
                .withKP(0.1)
                .withKI(0)
                .withKD(0);

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.Slot0 = slot0Configs;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfiguration.MotorOutput = motorOutputConfigs;

        intake.getConfigurator().apply(talonFXConfiguration);
        intake.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(200, Hertz));
    }

    private void configureHopperTalonFX() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        Slot0Configs slot0Configs = new Slot0Configs()
                .withKS(0.1)
                .withKV(0)
                .withKP(0.1)
                .withKI(0)
                .withKD(0);

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.Slot0 = slot0Configs;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfiguration.MotorOutput = motorOutputConfigs;

        hopper.getConfigurator().apply(talonFXConfiguration);
        hopper.getPosition().setUpdateFrequency(Frequency.ofBaseUnits(200, Hertz));
    }

}
