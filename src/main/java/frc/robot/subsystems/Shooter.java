package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.telemetry.HealthMonitor;
import frc.robot.telemetry.MonitoredSubsystem;

public class Shooter extends SubsystemBase implements MonitoredSubsystem {

    public enum ShooterVelocity {
        UNJAM(-10),
        WARM(10),
        MIN(40),
        TOWER(48),
        TRENCH(55),
        MAX(80); // Do not go any faster than this

        public double rotationsPerSecond;

        ShooterVelocity(double shooterRotationsPerSecond) {
            this.rotationsPerSecond = shooterRotationsPerSecond;
        }
    }

    private final TalonFX acceleratorLeader = new TalonFX(21);
    private final TalonFX acceleratorFollower = new TalonFX(22);
    private final VelocityVoltage acceleratorVelocityControl = new VelocityVoltage(0).withSlot(0);

    private final TalonFX shooterLeader = new TalonFX(23);
    private final TalonFX shooterFollower = new TalonFX(24);
    private final VelocityVoltage shooterVelocityControl = new VelocityVoltage(0).withSlot(0);

    private final Color systemColor = new Color(0, 0, 0);

    private final InterpolatingDoubleTreeMap shooterVelocityLUT = new InterpolatingDoubleTreeMap();

    private final double kGearRatio = 1.0;

    private final DCMotorSim m_motorSimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    DCMotor.getFalcon500(1), 0.001, kGearRatio),
            DCMotor.getFalcon500(1));

    public Shooter() {
        configureAccelerator();
        configureShooter();
        configureVelocityMap();

        if (Robot.isSimulation()) {
            simulationInit();
        }
    }

    public void setAcceleratorVelocity(double rotationsPerSecond) {
        acceleratorLeader.setControl(
                acceleratorVelocityControl
                        .withVelocity(rotationsPerSecond)
                        .withFeedForward(Volts.of(0)));
    }

    public boolean isAcceleratorNearRotationsPerSecond(double rotationsPerSecond, double tolerance) {
        return acceleratorLeader.getVelocity().isNear(rotationsPerSecond, tolerance);
    }

    public void setShooterVelocity(double rotationsPerSecond) {
        shooterLeader.setControl(
                shooterVelocityControl
                        .withVelocity(rotationsPerSecond)
                        .withFeedForward(Volts.of(0)));
    }

    public boolean isShooterNearRotationsPerSecond(double rotationsPerSecond, double tolerance) {
        return shooterLeader.getVelocity().isNear(rotationsPerSecond, tolerance);
    }

    public void stopShooter() {
        shooterLeader.stopMotor();
    }

    public void stopAccelerator() {
        acceleratorLeader.stopMotor();
    }

    @Logged(name = "Shooter velocity")
    public double getShooterVelocity() {
        return shooterLeader.getVelocity().getValueAsDouble();
    }

    @Logged(name = "Shooter leader stator current")
    public double getShooterLeaderStatorCurrent() {
        return shooterLeader.getStatorCurrent().getValueAsDouble();
    }

    @Logged(name = "Shooter leader supply current")
    public double getShooterLeaderSupplyCurrent() {
        return shooterLeader.getSupplyCurrent().getValueAsDouble();
    }

    @Logged(name = "Shooter follower stator current")
    public double getShooterFollowerStatorCurrent() {
        return shooterFollower.getStatorCurrent().getValueAsDouble();
    }

    @Logged(name = "Shooter follower supply current")
    public double getShooterFollowerSupplyCurrent() {
        return shooterFollower.getSupplyCurrent().getValueAsDouble();
    }

    @Logged(name = "Accelerator velocity")
    public double getAcceleratorVelocity() {
        return acceleratorLeader.getVelocity().getValueAsDouble();
    }

    @Logged(name = "Accelerator leader stator current")
    public double getAcceleratorLeaderStatorCurrent() {
        return acceleratorLeader.getStatorCurrent().getValueAsDouble();
    }

    @Logged(name = "Accelerator leader supply current")
    public double getAcceleratorLeaderSupplyCurrent() {
        return acceleratorLeader.getSupplyCurrent().getValueAsDouble();
    }

    @Logged(name = "Accelerator follower stator current")
    public double getAcceleratorFollowerStatorCurrent() {
        return acceleratorFollower.getStatorCurrent().getValueAsDouble();
    }

    @Logged(name = "Accelerator follower supply current")
    public double getAcceleratorFollowerSupplyCurrent() {
        return acceleratorFollower.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void registerWithHealthMonitor(HealthMonitor monitor) {
        monitor.addComponent(getSubsystem(), "Accelerator leader", acceleratorLeader);
        monitor.addComponent(getSubsystem(), "Accelerator follower", acceleratorFollower);
        monitor.addComponent(getSubsystem(), "Shooter leader", shooterLeader);
        monitor.addComponent(getSubsystem(), "Shooter follower", shooterFollower);
        monitor.setSubsystemColor(getSubsystem(), systemColor);
    }

    public double getTargetVelocity(double distance) {
        // (meters, rps)
        // y = 15.36x (0,0) to (3.125, 48)
        // y = 56x - 127 (3.125, 48) to (3.25, 55)
        return Math.max(Math.min(15.36 * distance, ShooterVelocity.MAX.rotationsPerSecond),
                ShooterVelocity.MIN.rotationsPerSecond);
    }

    public double getInterpolatedVelocity(double distance) {
        // get handles interpolation for you here
        return shooterVelocityLUT.get(distance);
    }

    public void simulationInit() {
        var talonFXSim = shooterLeader.getSimState();
        talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
        talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    }

    @Override
    public void simulationPeriodic() {
        var talonFXSim = shooterLeader.getSimState();

        // set the supply voltage of the TalonFX
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // get the motor voltage of the TalonFX
        var motorVoltage = talonFXSim.getMotorVoltageMeasure();

        // use the motor voltage to calculate new position and velocity
        // using WPILib's DCMotorSim class for physics simulation
        m_motorSimModel.setInputVoltage(motorVoltage.in(Volts));
        m_motorSimModel.update(0.020); // assume 20 ms loop time

        // apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratio), but
        // DCMotorSim returns mechanism position/velocity (after gear ratio)
        talonFXSim.setRawRotorPosition(m_motorSimModel.getAngularPosition().times(kGearRatio));
        talonFXSim.setRotorVelocity(m_motorSimModel.getAngularVelocity().times(kGearRatio));
    }

    private void configureVelocityMap() {
        /*
         * In code we assume center to center for distance!
         * 
         * Assuming the below input LUT values are measured from front of hub to front
         * of robot bumper, the offset should be added to each value to account for the
         * missing distance.
         * 
         */
        var hubFrontToCenterOffsetMeters = 0.591;
        var robotFrontToCenterOffsetMeters = 0.33; // with bumpers
        var offset = hubFrontToCenterOffsetMeters + robotFrontToCenterOffsetMeters;

        shooterVelocityLUT.put(Units.inchesToMeters(24) + offset, 40.0);
        shooterVelocityLUT.put(Units.inchesToMeters(24 * 2) + offset, 40.0);
        shooterVelocityLUT.put(Units.inchesToMeters(24 * 3) + offset, 45.0);
        shooterVelocityLUT.put(Units.inchesToMeters(24 * 4) + offset, 48.0);
        shooterVelocityLUT.put(Units.inchesToMeters(24 * 5) + offset, 53.0);
        shooterVelocityLUT.put(Units.inchesToMeters(24 * 10) + offset, 70.0);
    }

    private void configureAccelerator() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        acceleratorLeader.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(200, Hertz));

        Slot0Configs velocityGains = new Slot0Configs()
                .withKS(0.09)
                .withKV(0.11)
                .withKP(0.15)
                .withKI(0)
                .withKD(0);

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.Slot0 = velocityGains;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 50;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfiguration.MotorOutput = motorOutputConfigs;
        acceleratorLeader.getConfigurator().apply(talonFXConfiguration);
        acceleratorFollower.getConfigurator().apply(talonFXConfiguration);

        acceleratorFollower
                .setControl(new Follower(acceleratorLeader.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    private void configureShooter() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

        shooterLeader.getVelocity().setUpdateFrequency(Frequency.ofBaseUnits(200, Hertz));

        Slot0Configs velocityGains = new Slot0Configs()
                .withKS(0.09)
                .withKV(0.11)
                .withKP(0.25)
                .withKI(0)
                .withKD(0.01);

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.Slot0 = velocityGains;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 50;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfiguration.MotorOutput = motorOutputConfigs;
        shooterLeader.getConfigurator().apply(talonFXConfiguration);
        shooterFollower.getConfigurator().apply(talonFXConfiguration);

        shooterFollower
                .setControl(new Follower(shooterLeader.getDeviceID(), MotorAlignmentValue.Opposed));
    }
}