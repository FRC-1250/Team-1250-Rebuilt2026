package frc.robot.subsystems;

import java.lang.module.Configuration;
import java.security.PrivateKey;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;

@Logged
public class Shooter {
    
    TalonFX lodercamTalonFX;
     VelocityVoltage lodercamvelocitycontrol = new VelocityVoltage(0).withSlot(0);
     PositionVoltage lodercamPositionVoltage = new PositionVoltage(0);
public Shooter() {
    }
    TalonFX leader;
     VelocityVoltage leadervelocityVoltage = new VelocityVoltage(0).withSlot(0);
     TalonFX follower;
       VelocityVoltage  followervVelocityVoltage = new VelocityVoltage(0).withSlot(0);
      TalonFX leaderflywheel;
      
}