// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final Solenoid hopperExtension;
    private final TalonFX intakeWheels;
    private final VelocityVoltage intakeWheelVelocityControl;

    public Intake() {
        hopperExtension = new Solenoid(PneumaticsModuleType.REVPH, 0);
        intakeWheels = new TalonFX(40);
        intakeWheelVelocityControl = new VelocityVoltage(0);

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

        intakeWheels.getConfigurator().apply(talonFXConfiguration);
        intakeWheels.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(200, Hertz));

    }

    public void setVelocity(double rotationsPerSecond, Voltage feedForward) {
        intakeWheels.setControl(
                intakeWheelVelocityControl
                        .withVelocity(rotationsPerSecond)
                        .withFeedForward(feedForward));
    }

    public double getIntakeVelocity() {
        return intakeWheels.getVelocity().getValueAsDouble();
    }

    public void extend() {
        hopperExtension.set(true);
    }

    public void retract() {
        hopperExtension.set(false);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
