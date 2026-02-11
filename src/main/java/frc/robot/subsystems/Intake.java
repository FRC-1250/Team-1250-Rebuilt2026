// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
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
    enum intakePositions {
        RETRACT(0),
        Phase1(2),
        Phase2(3),
        Phase3(4),
        Extend(10);

        public double rotations;

        private intakePositions(double rotations) {
            this.rotations = rotations;
        }

    }

    enum intakeVelocityControl {
        STOP(0),
        GO(75);

        public double rotationsPerSecond;

        private intakeVelocityControl(double rotationsPerSecond) {
            this.rotationsPerSecond = rotationsPerSecond;
        }
    }

    private final TalonFX hopper = new TalonFX(41);

    private final PositionVoltage hopperPositionVoltage = new PositionVoltage(0);
    private final TalonFX intake = new TalonFX(40);

    private final VelocityVoltage intakeVelocityControl = new VelocityVoltage(0);

    private final Color systemColor = new Color(0, 0, 0);

    public Timer timerT = new Timer();

    double interval = 0;

    int index = 0;

    List<intakePositions> hopperAgitationOrder = new ArrayList<>();

    public Intake() {
        configureHopperTalonFX();
        configureIntakeTalonFX();
        configureHopperAgitation();
    }

    private void configureHopperAgitation() {
        hopperAgitationOrder.add(intakePositions.Phase3);
        hopperAgitationOrder.add(intakePositions.Phase2);
        hopperAgitationOrder.add(intakePositions.Phase1);
        hopperAgitationOrder.add(intakePositions.Phase2);
        hopperAgitationOrder.add(intakePositions.Phase3);
    }

    public void shift() {
        if (timerT.advanceIfElapsed(interval)) {
            setHopperPosition(hopperAgitationOrder.get(index % hopperAgitationOrder.size()).rotations);
            index = index + 1;
            timerT.reset();
        }
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

    public void setInterval(double imput) {
        interval = imput;
    }

    private void configureIntakeTalonFX() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0;
        slot0Configs.kV = 0;
        slot0Configs.kP = 0;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;

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

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0;
        slot0Configs.kV = 0;
        slot0Configs.kP = 0;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.Slot0 = slot0Configs;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfiguration.MotorOutput = motorOutputConfigs;

        hopper.getConfigurator().apply(talonFXConfiguration);
        hopper.getPosition().setUpdateFrequency(Frequency.ofBaseUnits(200, Hertz));
    }

}
