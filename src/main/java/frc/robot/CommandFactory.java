package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.SwerveVisionLogic;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FuelLine;
import frc.robot.subsystems.FuelLine.LoaderVelocity;
import frc.robot.subsystems.FuelLine.RollerVelocity;
import frc.robot.subsystems.Shooter.ShooterVelocity;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.HopperPosition;
import frc.robot.subsystems.Intake.IntakeVelocity;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class CommandFactory {

    private final CommandSwerveDrivetrain cmdSwerveDriveState;
    private final FuelLine fuelLine;
    private final Shooter shooter;
    private final Climber climber;
    private final Limelight cmdLimelight;
    private final Leds leds;
    private final Intake intake;

    public CommandFactory(
            CommandSwerveDrivetrain SwerveDriveState,
            FuelLine fuelLine,
            Intake intake,
            Shooter shooter,
            Climber climber,
            Limelight limelight,
            Leds leds) {
        this.cmdSwerveDriveState = SwerveDriveState;
        this.intake = intake;
        this.fuelLine = fuelLine;
        this.shooter = shooter;
        this.climber = climber;
        this.cmdLimelight = limelight;
        this.leds = leds;
    }

    /*
     * Drive
     */

    public Command cmdSwerveVisionLogic() {
        return new SwerveVisionLogic(cmdLimelight, cmdSwerveDriveState);
    }

    public Rotation2d determineHeading(Translation2d feildHeading) {
        {
            return Rotation2d.fromRadians(
                    Math.atan2(
                            feildHeading.getY() - cmdSwerveDriveState.getState().Pose.getY(),
                            feildHeading.getX() - cmdSwerveDriveState.getState().Pose.getX()))
                    .plus(cmdSwerveDriveState.getOperatorForwardDirection());
        }
    }

    /*
     * Intake
     */
    public Command cmdSetIntakeVelocity(double rotationsPerSecond) {
        return cmdSetIntakeVelocity(() -> rotationsPerSecond);
    }

    public Command cmdSetIntakeVelocity(Supplier<Double> supplier) {
        return Commands.runOnce(
                () -> intake.setIntakeVelocity(supplier.get()),
                intake);
    }

    public Command cmdSetHopperPosition(double rotations) {
        return cmdSetHopperPosition(() -> rotations);
    }

    public Command cmdSetHopperPosition(Supplier<Double> supplier) {
        return Commands.runOnce(
                () -> intake.setHopperPosition(supplier.get()), intake)
                .andThen(
                        Commands.waitUntil((() -> intake.isHopperNearPosition(supplier.get(), 1))));
    }

    public Command cmdResetHopperPositionWithAmps() {
        return Commands.sequence(
                Commands.runOnce(() -> intake.setHopperSpeed(-0.2), intake),
                Commands.waitUntil(() -> intake.isHopperAmpNearLimit()),
                Commands.runOnce(() -> {
                    intake.setHopperPosition(0);
                    intake.resetHopperPosition(0);
                }));
    }

    public Command cmdResetHopperPosition() {
        return Commands.runOnce(() -> {
            intake.resetHopperPosition(0);
        }).ignoringDisable(true);
    }

    /*
     * FuelLine
     */
    public Command cmdSetRollerVelocity(double rotationsPerSecond) {
        return cmdSetRollerVelocity(() -> rotationsPerSecond);
    }

    public Command cmdSetRollerVelocity(Supplier<Double> supplier) {
        return Commands.runOnce(
                () -> fuelLine.setRollerVelocity(supplier.get()),
                fuelLine);
    }

    public Command cmdSetLoaderPosition(double rotations) {
        return cmdSetLoaderPosition(() -> rotations);
    }

    public Command cmdSetLoaderPosition(Supplier<Double> supplier) {
        return Commands.runOnce(
                () -> fuelLine.setLoaderPosition(supplier.get()), fuelLine);
    }

    public Command cmdSetLoaderVelocity(double rotationsPerSecond) {
        return cmdSetLoaderVelocity(() -> rotationsPerSecond);
    }

    public Command cmdSetLoaderVelocity(Supplier<Double> supplier) {
        return Commands.runOnce(
                () -> fuelLine.setLoaderVelocity(supplier.get()), fuelLine);
    }

    /*
     * shooter
     */

    public Command cmdSetFuelAcceleratorVelocity(double rotationsPerSecond) {
        return cmdSetFuelAcceleratorVelocity(() -> rotationsPerSecond);

    }

    public Command cmdSetFuelAcceleratorVelocity(Supplier<Double> supplier) {
        return Commands.runOnce(
                () -> shooter.setAcceleratorVelocity(supplier.get()));

    }

    public Command cmdSetFuelShooterVelocity(double rotationsPerSecond) {
        return cmdSetFuelShooterVelocity(() -> rotationsPerSecond);

    }

    public Command cmdSetFuelShooterVelocity(Supplier<Double> supplier) {
        return Commands.runOnce(
                () -> shooter.setShooterVelocity(supplier.get()));

    }

    public Command cmdSetHoodPosition(Supplier<Double> supplier) {
        return Commands.runOnce(
                () -> shooter.setHoodPosition(supplier.get()), shooter)
                .andThen(Commands.waitUntil(() -> shooter.isHoodNearPosition(supplier.get(), 1)));
    }

    public Command cmdSetHoodPosition(double rotationsPerSecond) {
        return cmdSetHoodPosition(() -> rotationsPerSecond);
    }

    /*
     * Shared
     */
    public Command cmdFireFuel(Supplier<Double> shooterVelocitySupplier, Supplier<Double> acceleratorVelocitySupplier,
            Supplier<Double> loaderVelocitySupplier) {
        return Commands.runEnd(
                () -> {
                    shooter.setAcceleratorVelocity(acceleratorVelocitySupplier.get());
                    shooter.setShooterVelocity(shooterVelocitySupplier.get());
                    if (shooter.isAcceleratorNearRotationsPerSecond(acceleratorVelocitySupplier.get(), 2)
                            && shooter.isShooterNearRotationsPerSecond(shooterVelocitySupplier.get(), 2)) {
                        fuelLine.setLoaderVelocity(LoaderVelocity.FIRE.rotationsPerSecond);
                    } else {
                        fuelLine.setLoaderVelocity(LoaderVelocity.STALL.rotationsPerSecond);
                    }
                },
                () -> {
                    shooter.setAcceleratorVelocity(ShooterVelocity.WARM.acceleratorRotationsPerSecond);
                    shooter.setShooterVelocity(ShooterVelocity.WARM.shooterRotationsPerSecond);
                    fuelLine.setLoaderVelocity(LoaderVelocity.STALL.rotationsPerSecond);

                }, shooter, fuelLine, intake);
    }

    public Command cmdFireFuel(double shooterVelocity, double acceleratorVelocity, double loaderVelocity) {
        return cmdFireFuel(() -> shooterVelocity,
                () -> acceleratorVelocity,
                () -> loaderVelocity);
    }

    public Command cmdPickUpFuel() {
        return cmdSetHopperPosition(HopperPosition.EXTENDED.rotations)
                .andThen(cmdSetLoaderVelocity(LoaderVelocity.STALL.rotationsPerSecond))
                .andThen(cmdSetRollerVelocity(RollerVelocity.GO.rotationsPerSecond))
                .andThen(cmdSetIntakeVelocity(IntakeVelocity.GO.rotationsPerSecond));
    }

    public Command cmdHome() {
        return cmdSetRollerVelocity(RollerVelocity.STOP.rotationsPerSecond)
                .andThen(cmdSetIntakeVelocity(IntakeVelocity.STOP.rotationsPerSecond))
                .andThen(cmdSetHopperPosition(HopperPosition.RETRACTED.rotations));
    }

    public Command cmdShooterPrep() {
        return cmdSetFuelShooterVelocity(ShooterVelocity.WARM.shooterRotationsPerSecond)
                .andThen(cmdSetFuelAcceleratorVelocity(ShooterVelocity.WARM.acceleratorRotationsPerSecond));
    }
}