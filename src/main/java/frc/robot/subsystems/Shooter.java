package frc.robot.subsystems;

import java.lang.module.Configuration;
import java.security.PrivateKey;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

@Logged
public class Shooter {

    TalonFX poderCamTalonFX;
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
    }
}