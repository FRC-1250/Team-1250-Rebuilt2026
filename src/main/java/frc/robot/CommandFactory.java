package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

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

    private final CommandSwerveDrivetrain swerve;
    private final FuelLine fuelLine;
    private final Shooter shooter;
    private final Climber climber;
    private final Limelight limelight;
    private final Leds leds;
    private final Intake intake;

    public CommandFactory(
            CommandSwerveDrivetrain swerve,
            FuelLine fuelLine,
            Intake intake,
            Shooter shooter,
            Climber climber,
            Limelight limelight,
            Leds leds) {
        this.swerve = swerve;
        this.intake = intake;
        this.fuelLine = fuelLine;
        this.shooter = shooter;
        this.climber = climber;
        this.limelight = limelight;
        this.leds = leds;
    }

    /*
     * Drive
     */

    public Command cmdSwerveVisionLogic() {
        return new SwerveVisionLogic(limelight, swerve);
    }

    public Rotation2d determineHeading(Translation2d feildHeading) {
        {
            return Rotation2d.fromRadians(
                    Math.atan2(
                            feildHeading.getY() - swerve.getState().Pose.getY(),
                            feildHeading.getX() - swerve.getState().Pose.getX()))
                    .plus(swerve.getOperatorForwardDirection());
        }
    }

    /*
     * Intake
     */
    public Command cmdSetIntakeVelocity(double rotationsPerSecond) {
        return cmdSetIntakeVelocity(() -> rotationsPerSecond);
    }

    public Command cmdSetIntakeVelocity(DoubleSupplier supplier) {
        return Commands.runOnce(
                () -> intake.setIntakeVelocity(supplier.getAsDouble()),
                intake);
    }

    public Command cmdSetHopperPosition(double rotations) {
        return cmdSetHopperPosition(() -> rotations);
    }

    public Command cmdSetHopperPosition(DoubleSupplier supplier) {
        return Commands.runOnce(
                () -> intake.setHopperPosition(supplier.getAsDouble()), intake)
                .andThen(
                        Commands.waitUntil((() -> intake.isHopperNearPosition(supplier.getAsDouble(), 1))));
    }

    public Command cmdResetHopperPositionRetract() {
        return Commands.sequence(
                Commands.runOnce(() -> intake.setHopperSpeed(-0.2), intake),
                Commands.waitUntil(() -> intake.isHopperAmpNearLimit()),
                Commands.runOnce(() -> intake.resetHopperPosition(HopperPosition.MIN.rotations)),
                Commands.runOnce(() -> intake.setHopperPosition(HopperPosition.RETRACTED.rotations)));
    }

    public Command cmdResetHopperPositionWithExtend() {
        return Commands.sequence(
                Commands.runOnce(() -> intake.setHopperSpeed(0.1), intake),
                Commands.waitUntil(() -> intake.isHopperAmpNearLimit()),
                Commands.runOnce(() -> intake.resetHopperPosition(HopperPosition.MAX.rotations)),
                Commands.runOnce(() -> intake.setHopperPosition(HopperPosition.EXTENDED.rotations)));
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

    public Command cmdSetRollerVelocity(DoubleSupplier supplier) {
        return Commands.runOnce(
                () -> fuelLine.setRollerVelocity(supplier.getAsDouble()),
                fuelLine);
    }

    public Command cmdSetLoaderPosition(double rotations) {
        return cmdSetLoaderPosition(() -> rotations);
    }

    public Command cmdSetLoaderPosition(DoubleSupplier supplier) {
        return Commands.runOnce(
                () -> fuelLine.setLoaderPosition(supplier.getAsDouble()), fuelLine);
    }

    public Command cmdSetLoaderVelocity(double rotationsPerSecond) {
        return cmdSetLoaderVelocity(() -> rotationsPerSecond);
    }

    public Command cmdSetLoaderVelocity(DoubleSupplier supplier) {
        return Commands.runOnce(
                () -> fuelLine.setLoaderVelocity(supplier.getAsDouble()), fuelLine);
    }

    /*
     * shooter
     */

    public Command cmdSetFuelAcceleratorVelocity(double rotationsPerSecond) {
        return cmdSetFuelAcceleratorVelocity(() -> rotationsPerSecond);

    }

    public Command cmdSetFuelAcceleratorVelocity(DoubleSupplier supplier) {
        return Commands.runOnce(
                () -> shooter.setAcceleratorVelocity(supplier.getAsDouble()));

    }

    public Command cmdSetFuelShooterVelocity(double rotationsPerSecond) {
        return cmdSetFuelShooterVelocity(() -> rotationsPerSecond);

    }

    public Command cmdSetFuelShooterVelocity(DoubleSupplier supplier) {
        return Commands.runOnce(
                () -> shooter.setShooterVelocity(supplier.getAsDouble()));

    }

    /*
     * Shared
     */
    public Command cmdFireFuel(DoubleSupplier shooterVelocitySupplier, DoubleSupplier acceleratorVelocitySupplier,
            DoubleSupplier loaderVelocitySupplier) {
        return Commands.runEnd(
                () -> {
                    shooter.setAcceleratorVelocity(acceleratorVelocitySupplier.getAsDouble());
                    shooter.setShooterVelocity(shooterVelocitySupplier.getAsDouble());
                    if (shooter.isAcceleratorNearRotationsPerSecond(acceleratorVelocitySupplier.getAsDouble(), 2)
                            && shooter.isShooterNearRotationsPerSecond(shooterVelocitySupplier.getAsDouble(), 2)) {
                        fuelLine.setLoaderVelocity(LoaderVelocity.FIRE.rotationsPerSecond);
                        intake.agitateHopper();
                    } else {
                        fuelLine.setLoaderVelocity(LoaderVelocity.STALL.rotationsPerSecond);
                    }
                },
                () -> {
                    intake.setHopperPosition(HopperPosition.EXTENDED.rotations);
                    intake.stopHopper();
                    intake.resetAgitation();
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
                .andThen(() -> intake.stopHopper())
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

    /*
     * Prove Out
     */

    public Command shooterTestCommand() {
        return Commands.sequence(
                cmdSetFuelShooterVelocity(ShooterVelocity.HUB.shooterRotationsPerSecond).withTimeout(5),
                cmdSetFuelShooterVelocity(ShooterVelocity.STOP.shooterRotationsPerSecond).withTimeout(5));
    }

    public Command hopperTestCommand() {
        return Commands.sequence(
                cmdSetHopperPosition(HopperPosition.EXTENDED.rotations).withTimeout(5),
                cmdSetHopperPosition(HopperPosition.RETRACTED.rotations).withTimeout(5));
    }

    public Command intakeTestCommand() {
        return Commands.sequence(
                cmdSetIntakeVelocity(IntakeVelocity.GO.rotationsPerSecond).withTimeout(5),
                cmdSetIntakeVelocity(IntakeVelocity.STOP.rotationsPerSecond).withTimeout(5));
    }

    public Command rollerTestCommand() {
        return Commands.sequence(
                cmdSetRollerVelocity(RollerVelocity.GO.rotationsPerSecond).withTimeout(5),
                cmdSetRollerVelocity(RollerVelocity.STOP.rotationsPerSecond).withTimeout(5));
    }

    public Command loaderTestCommand() {
        return Commands.sequence(
                cmdSetLoaderVelocity(LoaderVelocity.FIRE.rotationsPerSecond).withTimeout(5),
                cmdSetLoaderVelocity(LoaderVelocity.STALL.rotationsPerSecond).withTimeout(5));

    }

    public Command acceleratorTestCommand() {
        return Commands.sequence(
                cmdSetFuelAcceleratorVelocity(ShooterVelocity.HUB.acceleratorRotationsPerSecond).withTimeout(5),
                cmdSetFuelAcceleratorVelocity(ShooterVelocity.HUB.acceleratorRotationsPerSecond).withTimeout(5));
    }

    public Command cmdDriveTest(double maxSpeed, double maxAngularRate) {
        SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                .withDeadband(maxSpeed * 0.1).withRotationalDeadband(maxAngularRate * 0.1)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        return Commands.sequence(

                // Drive forward
                swerve.applyRequest(() -> drive
                        .withVelocityX(0.1 * maxSpeed)
                        .withVelocityY(0)
                        .withRotationalRate(0))
                        .withTimeout(5),

                // Drive forward fast
                swerve.applyRequest(() -> drive
                        .withVelocityX(0.8 * maxSpeed)
                        .withVelocityY(0)
                        .withRotationalRate(0))
                        .withTimeout(5),
                // Drive backwards
                swerve.applyRequest(() -> drive
                        .withVelocityX(-0.1 * maxSpeed)
                        .withVelocityY(0)
                        .withRotationalRate(0))
                        .withTimeout(5),

                // Drive backwards fast
                swerve.applyRequest(() -> drive
                        .withVelocityX(-0.8 * maxSpeed)
                        .withVelocityY(0)
                        .withRotationalRate(0))
                        .withTimeout(5),
                // Drive left
                swerve.applyRequest(() -> drive
                        .withVelocityX(0 * maxSpeed)
                        .withVelocityY(-0.1)
                        .withRotationalRate(0))
                        .withTimeout(5),

                // Drive left fast
                swerve.applyRequest(() -> drive
                        .withVelocityX(0 * maxSpeed)
                        .withVelocityY(-0.8)
                        .withRotationalRate(0))
                        .withTimeout(5),
                // Drive right
                swerve.applyRequest(() -> drive
                        .withVelocityX(0 * maxSpeed)
                        .withVelocityY(0.1)
                        .withRotationalRate(0))
                        .withTimeout(5),

                // Drive right fast
                swerve.applyRequest(() -> drive
                        .withVelocityX(0 * maxSpeed)
                        .withVelocityY(0.8)
                        .withRotationalRate(0))
                        .withTimeout(5),
                // Spin
                swerve.applyRequest(() -> drive
                        .withVelocityX(0 * maxSpeed)
                        .withVelocityY(0)
                        .withRotationalRate(0.1 * maxAngularRate))
                        .withTimeout(5),
                // Circle P1
                swerve.applyRequest(() -> drive
                        .withVelocityX(0.7 * maxSpeed)
                        .withVelocityY(0.3 * maxSpeed)
                        .withRotationalRate(0))
                        .withTimeout(0.3),

                // Circle P2
                swerve.applyRequest(() -> drive
                        .withVelocityX(0.5 * maxSpeed)
                        .withVelocityY(0.5 * maxSpeed)
                        .withRotationalRate(0))
                        .withTimeout(0.3),

                // Circle P3
                swerve.applyRequest(() -> drive
                        .withVelocityX(0.3 * maxSpeed)
                        .withVelocityY(0.7 * maxSpeed)
                        .withRotationalRate(0))
                        .withTimeout(0.3),

                // Circle P4
                swerve.applyRequest(() -> drive
                        .withVelocityX(0 * maxSpeed)
                        .withVelocityY(1 * maxSpeed)
                        .withRotationalRate(0))
                        .withTimeout(0.3),

                // Circle P5
                swerve.applyRequest(() -> drive
                        .withVelocityX(-0.3 * maxSpeed)
                        .withVelocityY(0.7)
                        .withRotationalRate(0))
                        .withTimeout(0.3),

                // Circle P6
                swerve.applyRequest(() -> drive
                        .withVelocityX(-0.5 * maxSpeed)
                        .withVelocityY(0.5 * maxSpeed)
                        .withRotationalRate(0))
                        .withTimeout(0.3),

                // Circle P7
                swerve.applyRequest(() -> drive
                        .withVelocityX(-1 * maxSpeed)
                        .withVelocityY(0)
                        .withRotationalRate(0))
                        .withTimeout(0.3),

                // Circle P8
                swerve.applyRequest(() -> drive
                        .withVelocityX(-0.7 * maxSpeed)
                        .withVelocityY(-0.3 * maxSpeed)
                        .withRotationalRate(0))
                        .withTimeout(0.3),
                // Circle P9
                swerve.applyRequest(() -> drive
                        .withVelocityX(-0.5 * maxSpeed)
                        .withVelocityY(-0.5 * maxSpeed)
                        .withRotationalRate(0))
                        .withTimeout(0.3),
                // Circle P10
                swerve.applyRequest(() -> drive
                        .withVelocityX(-0.3 * maxSpeed)
                        .withVelocityY(-0.7 * maxSpeed)
                        .withRotationalRate(0))
                        .withTimeout(0.3),
                // Circle P11
                swerve.applyRequest(() -> drive
                        .withVelocityX(0)
                        .withVelocityY(-1 * maxSpeed)
                        .withRotationalRate(0))
                        .withTimeout(0.3),
                // Circle P12
                swerve.applyRequest(() -> drive
                        .withVelocityX(0.3 * maxSpeed)
                        .withVelocityY(-0.7 * maxSpeed)
                        .withRotationalRate(0))
                        .withTimeout(0.3),
                // Circle P13
                swerve.applyRequest(() -> drive
                        .withVelocityX(0.5 * maxSpeed)
                        .withVelocityY(-0.5 * maxSpeed)
                        .withRotationalRate(0))
                        .withTimeout(0.3),
                // Circle P14
                swerve.applyRequest(() -> drive
                        .withVelocityX(1 * maxSpeed)
                        .withVelocityY(0)
                        .withRotationalRate(0))
                        .withTimeout(0.3));

    }

    public Command proveOut() {
        return Commands.sequence(
                cmdDriveTest(0, 0),
                rollerTestCommand(),
                loaderTestCommand(),
                hopperTestCommand(),
                intakeTestCommand(),
                acceleratorTestCommand(),
                shooterTestCommand());
        // Add Climber Eventually
    }
}
