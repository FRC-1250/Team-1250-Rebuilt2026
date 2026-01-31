package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearVelocity;
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

    void setloderCamPosition(double rotations) {
        loderCamTalonFX.setControl(
                loderCamPositionVoltage
                        .withPosition(rotations)
                        .withFeedForward(Volts.of(0)));
    }

    void setloderCamVelocity(double rotations) {
        loderCamTalonFX.setControl(
                loderCamPositionVoltage
                        .withVelocity(rotations)
                        .withFeedForward(Volts.of(0)));
    }

    @Logged(name = "Position")
    public double getRotations() {
        return loderCamTalonFX.getPosition().getValueAsDouble();
    }

    @Logged(name = "velocity")
    public double loderCamPositionVoltage() {
        return loderCamTalonFX.getVelocity().getValueAsDouble();
    }

    public Shooter() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        loderCamTalonFX.getPosition().setUpdateFrequency(Frequency.ofBaseUnits(200, Hertz));
        loderCamTalonFX.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(200, Hertz));
        Slot0Configs intakePositionPIDConfigs = new Slot0Configs()
                .withKP(0)
                .withKI(0)
                .withKD(0);

        Slot1Configs intakeVelocityPIDConfigs = new Slot1Configs()
                .withKS(0)
                .withKV(0)
                .withKP(0)
                .withKI(0)
                .withKD(0);
        TalonFXConfiguration intakeTalonFXConfiguration = new TalonFXConfiguration();
        intakeTalonFXConfiguration.Slot0 = intakePositionPIDConfigs;
        intakeTalonFXConfiguration.Slot1 = intakeVelocityPIDConfigs;
        intakeTalonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
        intakeTalonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

    }

    private double MaxSpeed = TunerConstants.kSpeedAt12Voltss.in(MetersPerSecond);

}