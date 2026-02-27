// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.HealthMonitor;
import frc.robot.telemetry.MonitoredSubsystem;
import frc.robot.utility.AgitationProfile;
import frc.robot.utility.AgitationStep;

public class ReactionBar extends SubsystemBase implements MonitoredSubsystem {
    public enum ReactionBarPosition {
        HOME(0),
        EXTENDED(1);

        public double rotations;

        private ReactionBarPosition(double rotations) {
            this.rotations = rotations;
        }
    }

    private final TalonFX reactionBar = new TalonFX(35);
    private final PositionVoltage reactionBarPositionControl = new PositionVoltage(0);
    private final CANcoder reactionBarEncoder = new CANcoder(36);
    private final double MAGNET_OFFSET = 0;

    private final Color systemColor = new Color(0, 0, 0);

    private AgitationProfile active;
    private AgitationProfile wave;

    public ReactionBar() {
        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

        talonFXConfiguration.Feedback.FeedbackRemoteSensorID = reactionBarEncoder.getDeviceID();
        reactionBar.getConfigurator().apply(talonFXConfiguration);

        CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
        canCoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        canCoderConfiguration.MagnetSensor.MagnetOffset = MAGNET_OFFSET;
        canCoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        reactionBarEncoder.getConfigurator().apply(canCoderConfiguration);

        configureAgitiationProfiles();
        active = wave;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void setReactionBarPosition(double rotations) {
        reactionBar.setControl(
                reactionBarPositionControl
                        .withPosition(rotations)
                        .withFeedForward(Volts.of(0)));

    }

    public boolean isReactionBarNearPosition(double rotations, double tolerance) {
        return reactionBar.getPosition().isNear(rotations, tolerance);

    }

    public boolean isNearPosition(double rotations) {
        return reactionBarEncoder.getPosition().isNear(rotations, 0.01);
    }

    public void agitate() {
        setReactionBarPosition(active.shift());
    }

    public void resetAgitation() {
        active.reset();
    }

    private void configureAgitiationProfiles() {
        wave = new AgitationProfile();
        wave.addStep(new AgitationStep(0.25, 2));
        wave.addStep(new AgitationStep(0.5, 2));
    }

    @Override
    public void registerWithHealthMonitor(HealthMonitor monitor) {
        monitor.addComponent(getSubsystem(), "Reacton bar", reactionBar);
        monitor.addComponent(getSubsystem(), "Reaction bar encoder", reactionBarEncoder);
        monitor.setSubsystemColor(getSubsystem(), systemColor);
    }

}