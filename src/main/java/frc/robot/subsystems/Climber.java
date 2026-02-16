package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import frc.robot.subsystems.Climber;
import frc.robot.telemetry.HealthMonitor;
import frc.robot.telemetry.MonitoredSubsystem;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Climber extends SubsystemBase implements MonitoredSubsystem {

    public enum ClimberPosition {
        HOOKOUT(0),
        TOWERL1(0),
        TOWERL2(0),
        HOME(0);

        public final double rotations;

        ClimberPosition(double rotations) {
            this.rotations = rotations;
        }
    }

    private final PositionVoltage climberPositionControl = new PositionVoltage(0);
    private final Color systemColor = new Color(0, 0, 0);

    private TalonFX climberLeader = new TalonFX(50);
    private TalonFX climberFollower = new TalonFX(51);

    public Climber() {
        climberFollower.setControl(new Follower(climberLeader.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public boolean isClimberNearPosition(double position) {
        return isClimberNearPosition(position);
    }

    @Override
    public void registerWithHealthMonitor(HealthMonitor monitor) {
        monitor.addComponent(getSubsystem(), "Climber leader", climberLeader);
        monitor.addComponent(getSubsystem(), "Climber follower", climberFollower);
    }

    @Override
    public Color getSubsystemColor() {
        return systemColor;
    }

    public void setClimberPosition(double rotations) {
        climberLeader.setControl(
                climberPositionControl
                        .withPosition(rotations)
                        .withFeedForward(Volts.of(0)));
    }

}
