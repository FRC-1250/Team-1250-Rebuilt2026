package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

@Logged
public class Shooter {

    TalonFX loderCamTalonFX;
    VelocityVoltage loderCamVelocityControl = new VelocityVoltage(0).withSlot(0);
    PositionVoltage loderCamPositionVoltage = new PositionVoltage(0);

    TalonFX precursorFlyWheelLeader;
    VelocityVoltage precursorFlyWheelVelocityVoltage = new VelocityVoltage(0).withSlot(0);
    TalonFX precursorFlyWheelFollower;

    TalonFX flyWheelLeader;
    VelocityVoltage flyWheelVelocityVoltage = new VelocityVoltage(0).withSlot(0);
    TalonFX flyWheelFollower;

    Solenoid hoodSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);

    public Shooter() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    }
}