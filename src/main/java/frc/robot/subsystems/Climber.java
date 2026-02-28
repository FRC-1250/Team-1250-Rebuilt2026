package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import frc.robot.subsystems.Climber;
import frc.robot.telemetry.HealthMonitor;
import frc.robot.telemetry.MonitoredSubsystem;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase implements MonitoredSubsystem {

    public enum ClimberPosition {
        CLIMB(0),
        PASS(0),
        HOME(0);

        public final double rotations;

        ClimberPosition(double rotations) {
            this.rotations = rotations;
        }
    }

    private final TalonFX climberLeader = new TalonFX(50);
    private final TalonFX climberFollower = new TalonFX(51);
    private final CANcoder climberAbsoluteEncoder = new CANcoder(52);
    private final double encoderOffset = 0.0;
    private final PositionVoltage climberPositionControl = new PositionVoltage(0).withSlot(1);

    private final Color systemColor = new Color(0, 0, 0);

    public Climber() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

        Slot1Configs positionGains = new Slot1Configs()
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withKS(0)
                .withKP(1)
                .withKI(0)
                .withKD(0.01);

        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.FeedbackRemoteSensorID = climberAbsoluteEncoder.getDeviceID();

        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.SupplyCurrentLimit = 20;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.Slot1 = positionGains;
        talonFXConfiguration.CurrentLimits = currentLimitsConfigs;
        talonFXConfiguration.MotorOutput = motorOutputConfigs;
        talonFXConfiguration.Feedback = feedbackConfigs;

        climberLeader.getConfigurator().apply(talonFXConfiguration);
        climberFollower.setControl(new Follower(climberLeader.getDeviceID(), MotorAlignmentValue.Opposed));

        CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
        canCoderConfiguration.MagnetSensor.MagnetOffset = encoderOffset;
        canCoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        canCoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        climberAbsoluteEncoder.getAbsolutePosition().setUpdateFrequency(200);
        climberAbsoluteEncoder.getConfigurator().apply(canCoderConfiguration);
    }

    @Override
    public void registerWithHealthMonitor(HealthMonitor monitor) {
        monitor.addComponent(getSubsystem(), "Climber leader", climberLeader);
        monitor.addComponent(getSubsystem(), "Climber follower", climberFollower);
        monitor.setSubsystemColor(getSubsystem(), systemColor);
    }

    public void setClimberPosition(double rotations) {
        climberLeader.setControl(
                climberPositionControl
                        .withPosition(rotations)
                        .withFeedForward(Volts.of(0)));

    }

    public boolean isNearPosition(double rotations) {
        return climberAbsoluteEncoder.getPosition().isNear(rotations, 0.01);
    }

    @Logged(name = "Abs position")
    public double getClimberPosition() {
        return climberAbsoluteEncoder.getAbsolutePosition().getValueAsDouble();
    }

    @Logged(name = "Leader stator current")
    public double getLeaderStatorCurrent() {
        return climberLeader.getStatorCurrent().getValueAsDouble();
    }

    @Logged(name = "Leader supply current")
    public double getLeaderSupplyCurrent() {
        return climberLeader.getSupplyCurrent().getValueAsDouble();
    }

    @Logged(name = "Follower stator current")
    public double getClimberStatorCurrent() {
        return climberFollower.getStatorCurrent().getValueAsDouble();
    }

    @Logged(name = "Follower supply current")
    public double getClimberSupplyCurrent() {
        return climberFollower.getSupplyCurrent().getValueAsDouble();
    }
}