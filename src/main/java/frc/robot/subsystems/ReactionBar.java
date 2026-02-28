// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.HealthMonitor;
import frc.robot.telemetry.MonitoredSubsystem;
import frc.robot.utility.AgitationProfile;
import frc.robot.utility.AgitationStep;

public class ReactionBar extends SubsystemBase implements MonitoredSubsystem {
    public enum ReactionBarPosition {
        HOME(-0.29),
        EXTENDED(0.29);

        public double rotations;

        private ReactionBarPosition(double rotations) {
            this.rotations = rotations;
        }
    }

    private final TalonFX reactionBar = new TalonFX(35);
    private final PositionVoltage reactionBarPositionControl = new PositionVoltage(0).withSlot(1);
    private final CANcoder reactionBarEncoder = new CANcoder(36);
    private final double MAGNET_OFFSET = -0.2;

    private final Color systemColor = new Color(0, 0, 0);

    private AgitationProfile active;
    private AgitationProfile wave;

    public ReactionBar() {
        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        Slot1Configs positionGains = new Slot1Configs()
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withKS(0)
                .withKP(5)
                .withKI(0)
                .withKD(0.01);
        talonFXConfiguration.Feedback.FeedbackRemoteSensorID = reactionBarEncoder.getDeviceID();
        reactionBar.getConfigurator().apply(talonFXConfiguration);
        talonFXConfiguration.Slot1 = positionGains;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 0;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = false;
        talonFXConfiguration.MotorOutput = motorOutputConfigs;

        CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
        canCoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
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
        return reactionBarEncoder.getPosition().isNear(rotations, tolerance);
    }

    public void agitate() {
        setReactionBarPosition(active.shift());
    }

    public void resetAgitation() {
        active.reset();
    }

    @Logged(name = "Reaction bar abs position")
    public double getReactionBarPosition() {
        return reactionBarEncoder.getAbsolutePosition().getValueAsDouble();
    }

    @Logged(name = "Reaction bar stator current")
    public double getClimberStatorCurrent() {
        return reactionBar.getStatorCurrent().getValueAsDouble();
    }

    @Logged(name = "Reaction bar supply current")
    public double getClimberSupplyCurrent() {
        return reactionBar.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void registerWithHealthMonitor(HealthMonitor monitor) {
        monitor.addComponent(getSubsystem(), "Reacton bar", reactionBar);
        monitor.addComponent(getSubsystem(), "Reaction bar encoder", reactionBarEncoder);
        monitor.setSubsystemColor(getSubsystem(), systemColor);
    }

    private void configureAgitiationProfiles() {
        wave = new AgitationProfile();
        wave.addStep(new AgitationStep(0.25, 2));
        wave.addStep(new AgitationStep(0.5, 2));
    }

}
