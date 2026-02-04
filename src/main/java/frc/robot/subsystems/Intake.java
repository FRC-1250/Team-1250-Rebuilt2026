// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.HealthMonitor;
import frc.robot.telemetry.MonitoredSubsystem;

public class Intake extends SubsystemBase implements MonitoredSubsystem {
    private TalonFX hopper;
    private PositionVoltage hopperPositionVoltage;

    private TalonFX intake;
    private VelocityVoltage intakeVelocityControl;

    public Intake() {
        configureHopperTalonFX();
        configureIntakeTalonFX();
    }

    public void setIntakeVelocity(double rotationsPerSecond) {
        intake.setControl(
                intakeVelocityControl
                        .withVelocity(rotationsPerSecond)
                        .withFeedForward(Voltage.ofBaseUnits(0, Units.Volts)));
    }

    public double getIntakeVelocity() {
        return intake.getVelocity().getValueAsDouble();
    }

    public void setHopperPosition(double rotations) {
        hopper.setControl(
                hopperPositionVoltage
                        .withPosition(rotations)
                        .withFeedForward(Voltage.ofBaseUnits(0, Units.Volts)));
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
    }

    private void configureIntakeTalonFX() {
        intake = new TalonFX(40);
        intakeVelocityControl = new VelocityVoltage(0);

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0;
        slot0Configs.kV = 0;
        slot0Configs.kP = 0;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;

        intake.getConfigurator().apply(talonFXConfiguration);
        intake.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(200, Hertz));
    }

    private void configureHopperTalonFX() {
        hopper = new TalonFX(41);
        hopperPositionVoltage = new PositionVoltage(0);

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0;
        slot0Configs.kV = 0;
        slot0Configs.kP = 0;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;

        hopper.getConfigurator().apply(talonFXConfiguration);
        hopper.getPosition().setUpdateFrequency(Frequency.ofBaseUnits(200, Hertz));

    }

}
