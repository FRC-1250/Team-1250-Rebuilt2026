package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.subsystems.Climber;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Climber {

    public enum LeftClimberPosition {
        HOOKOUT(0),
        TOWERL1(0),
        TOWERL2(0),
        HOME(0);

        public final double rotations;

        LeftClimberPosition(double rotations) {
            this.rotations = rotations;
        }
    }

    private static final PositionVoltage climberPositionControl = new PositionVoltage(0);

    private TalonFX LeftClimber = new TalonFX(50);
    private TalonFX RightClimber = new TalonFX(51);

    public Climber() {
    }

    public boolean isLeftClimberNearPosition(double position) {
        return isLeftClimberNearPosition(position);
    }

    private void setLeftClimberPosition(double rotations) {
        LeftClimber.setControl(
                climberPositionControl
                        .withPosition(rotations)
                        .withFeedForward(Volts.of(0)));
    }

}
