package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
import frc.robot.subsystems.ReactionBar.ReactionBarPosition;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ReactionBar;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Climber.ClimberPosition;

public class CommandFactory {

    private final CommandSwerveDrivetrain swerve;
    private final FuelLine fuelLine;
    private final Shooter shooter;
    private final Climber climber;
    private final Limelight limelight;
    private final Leds leds;
    private final Intake intake;
    private final ReactionBar reactionBar;

    public CommandFactory(
            CommandSwerveDrivetrain swerve,
            FuelLine fuelLine,
            Intake intake,
            Shooter shooter,
            Climber climber,
            Limelight limelight,
            Leds leds,
            ReactionBar reactionBar) {
        this.swerve = swerve;
        this.intake = intake;
        this.fuelLine = fuelLine;
        this.shooter = shooter;
        this.climber = climber;
        this.limelight = limelight;
        this.leds = leds;
        this.reactionBar = reactionBar;
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
     * ReactionBar
     */

    public Command cmdSetReactionBarPosition(DoubleSupplier supplier) {
        return Commands.runOnce(() -> reactionBar.setReactionBarPosition(supplier.getAsDouble()), reactionBar)
                .andThen(
                        Commands.waitUntil((() -> reactionBar.isReactionBarNearPosition(supplier.getAsDouble(), .01))));
    }

    public Command cmdSetReactionBarPosition(double rotations) {
        return cmdSetReactionBarPosition(() -> rotations);
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

    public Command cmdStopIntake() {
        return Commands.runOnce(() -> intake.stopIntake(), intake);
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

    public Command cmdStopHopper() {
        return Commands.runOnce(() -> intake.stopHopper(), intake);
    }

    public Command cmdResetHopperPositionRetract() {
        return Commands.sequence(
                Commands.runOnce(() -> intake.setHopperSpeed(-0.2), intake),
                Commands.waitUntil(() -> intake.isHopperAmpNearLimit()),
                Commands.runOnce(() -> intake.resetHopperPosition(HopperPosition.MIN.rotations)),
                Commands.runOnce(() -> intake.setHopperPosition(HopperPosition.HOME.rotations)));
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

    public Command cmdStopRoller() {
        return Commands.runOnce(() -> fuelLine.stopRoller());
    }

    public Command cmdSetLoaderVelocity(double rotationsPerSecond) {
        return cmdSetLoaderVelocity(() -> rotationsPerSecond);
    }

    public Command cmdSetLoaderVelocity(DoubleSupplier supplier) {
        return Commands.runOnce(
                () -> fuelLine.setLoaderVelocity(supplier.getAsDouble()), fuelLine);
    }

    public Command cmdStopLoader() {
        return Commands.runOnce(() -> fuelLine.stopLoader());
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

    public Command cmdStopAccelerator() {
        return Commands.runOnce(() -> shooter.stopAccelerator(), shooter);
    }

    public Command cmdSetFuelShooterVelocity(double rotationsPerSecond) {
        return cmdSetFuelShooterVelocity(() -> rotationsPerSecond);

    }

    public Command cmdSetFuelShooterVelocity(DoubleSupplier supplier) {
        return Commands.runOnce(
                () -> shooter.setShooterVelocity(supplier.getAsDouble()));

    }

    public Command cmdStopShooter() {
        return Commands.runOnce(() -> shooter.stopShooter(), shooter);
    }

    /*
     * Climber
     */
    public Command cmdSetClimberPosition(DoubleSupplier supplier) {
        return Commands.runOnce(
                () -> climber.setClimberPosition(supplier.getAsDouble()))
                .andThen(Commands.waitUntil(() -> climber.isNearPosition(supplier.getAsDouble())));
    }

    /*
     * Shared
     */
    public Command cmdFireFuel(DoubleSupplier shooterVelocitySupplier, DoubleSupplier acceleratorVelocitySupplier,
            DoubleSupplier loaderVelocitySupplier) {
        return Commands.runEnd(
                () -> {
                    var accelVelocity = acceleratorVelocitySupplier.getAsDouble();
                    var shooterVelocity = shooterVelocitySupplier.getAsDouble();
                    var loaderVelocity = loaderVelocitySupplier.getAsDouble();

                    shooter.setAcceleratorVelocity(accelVelocity);
                    shooter.setShooterVelocity(shooterVelocity);
                    if (shooter.isAcceleratorNearRotationsPerSecond(accelVelocity, 2)
                            && shooter.isShooterNearRotationsPerSecond(shooterVelocity, 2)) {
                        fuelLine.setLoaderVelocity(loaderVelocity);
                        reactionBar.agitate();
                    } else {
                        fuelLine.setLoaderVelocity(loaderVelocity);
                    }
                },
                () -> {
                    reactionBar.setReactionBarPosition(ReactionBarPosition.EXTENDED.rotations);
                    reactionBar.resetAgitation();
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

    public Command cmdActivateFuelPickUp() {
        return cmdSetHopperPosition(HopperPosition.EXTENDED.rotations)
                .andThen(cmdSetReactionBarPosition(ReactionBarPosition.EXTENDED.rotations))
                .andThen(cmdSetLoaderVelocity(LoaderVelocity.STALL.rotationsPerSecond))
                .andThen(cmdSetRollerVelocity(RollerVelocity.GO.rotationsPerSecond))
                .andThen(cmdSetIntakeVelocity(IntakeVelocity.GO.rotationsPerSecond));
    }

    public Command cmdDeactivateFuelPickUp() {
        return cmdStopIntake()
                .andThen(cmdStopRoller())
                .andThen(cmdStopLoader())
                .andThen(cmdSetReactionBarPosition(ReactionBarPosition.HOME.rotations))
                .andThen(cmdSetHopperPosition(HopperPosition.HOME.rotations));
    }

    public Command cmdWarmUpShooter() {
        return cmdSetFuelShooterVelocity(ShooterVelocity.WARM.shooterRotationsPerSecond)
                .andThen(cmdSetFuelAcceleratorVelocity(ShooterVelocity.WARM.acceleratorRotationsPerSecond));
    }

    /*
     * Prove Out
     */

    private Command shooterTestCommand() {
        return Commands.sequence(
                cmdSetFuelShooterVelocity(ShooterVelocity.HUB.shooterRotationsPerSecond),
                Commands.waitSeconds(5),
                cmdStopShooter());
    }

    private Command hopperTestCommand() {
        return Commands.sequence(
                cmdSetHopperPosition(HopperPosition.EXTENDED.rotations),
                Commands.waitSeconds(5),
                cmdSetHopperPosition(HopperPosition.HOME.rotations),
                Commands.waitSeconds(5),
                cmdSetHopperPosition(HopperPosition.EXTENDED.rotations));
    }

    private Command intakeTestCommand() {
        return Commands.sequence(
                cmdSetIntakeVelocity(IntakeVelocity.GO.rotationsPerSecond),
                Commands.waitSeconds(5),
                cmdStopIntake());
    }

    private Command rollerTestCommand() {
        return Commands.sequence(
                cmdSetRollerVelocity(RollerVelocity.GO.rotationsPerSecond),
                Commands.waitSeconds(5),
                cmdStopRoller());
    }

    private Command loaderTestCommand() {
        return Commands.sequence(
                cmdSetLoaderVelocity(LoaderVelocity.FIRE.rotationsPerSecond),
                Commands.waitSeconds(5),
                cmdSetLoaderVelocity(LoaderVelocity.STALL.rotationsPerSecond),
                Commands.waitSeconds(5),
                cmdStopLoader());

    }

    private Command acceleratorTestCommand() {
        return Commands.sequence(
                cmdSetFuelAcceleratorVelocity(ShooterVelocity.HUB.acceleratorRotationsPerSecond),
                Commands.waitSeconds(5),
                cmdStopAccelerator());
    }

    private Command driveTest(double targetVelocityX, double targetVelocityY, double duration, int steps) {
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        double stepVelocityX = targetVelocityX / steps;
        double stepVelocityY = targetVelocityY / steps;
        double stepDuration = duration / steps;

        for (int i = 1; i <= steps; i++) {
            final double currentX = stepVelocityX * i;
            final double currentY = stepVelocityY * i;

            sequence.addCommands(
                    swerve.applyRequest(() -> drive
                            .withVelocityX(currentX)
                            .withVelocityY(currentY)
                            .withRotationalRate(0))
                            .withTimeout(stepDuration));
        }

        sequence.addCommands(
                swerve.applyRequest(() -> drive
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0))
                        .withTimeout(2));

        return sequence;
    }

    private Command rotateTest(double targetRotationRate, double duration, int steps) {
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        double stepRotationalRate = targetRotationRate / steps;
        double stepDuration = duration / steps;

        for (int i = 1; i <= steps; i++) {
            final double currentRate = stepRotationalRate * i;

            sequence.addCommands(
                    swerve.applyRequest(() -> drive
                            .withVelocityX(0)
                            .withVelocityY(0)
                            .withRotationalRate(currentRate))
                            .withTimeout(stepDuration));
        }

        sequence.addCommands(
                swerve.applyRequest(() -> drive
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0))
                        .withTimeout(2));

        return sequence;
    }

    private Command pointWheelsTest(double targetVelocity, double duration, int steps, boolean clockwise) {
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        double stepAngle = (2 * Math.PI) / steps;
        double stepDuration = duration / steps;

        if (clockwise) {
            stepAngle = -stepAngle;
        }

        for (int index = 0; index < steps; index++) {
            double theta = index * stepAngle;
            double x = Math.cos(theta);
            double y = Math.sin(theta);
            double magnitude = Math.hypot(x, y);

            final double velocityX;
            final double velocityY;

            if (magnitude > 0) {
                velocityX = (x / magnitude) * targetVelocity;
                velocityY = (y / magnitude) * targetVelocity;
            } else {
                velocityX = 0;
                velocityY = 0;
            }

            sequence.addCommands(
                    swerve.applyRequest(() -> drive
                            .withVelocityX(velocityX)
                            .withVelocityY(velocityY)
                            .withRotationalRate(0))
                            .withTimeout(stepDuration));
        }

        sequence.addCommands(
                swerve.applyRequest(() -> drive
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0))
                        .withTimeout(2));

        return sequence;
    }

    public Command driveProveOut() {
        final double targetTestVelocity = 2.5;
        final double targetTestRotationRate = RotationsPerSecond.of(0.375).in(RadiansPerSecond);

        final int driveTestSteps = 3;
        final double driveTestDuration = 5;

        final int rotateTestSteps = 3;
        final double rotateTestDuration = 5;

        final int pointWheelsTestSteps = 10;
        final double pointWheelsTestDuration = 5;

        return Commands.sequence(
                driveTest(targetTestVelocity, 0, driveTestDuration, driveTestSteps),
                driveTest(-targetTestVelocity, 0, driveTestDuration, driveTestSteps),
                driveTest(0, targetTestVelocity, driveTestDuration, driveTestSteps),
                driveTest(0, -targetTestVelocity, driveTestDuration, driveTestSteps),
                rotateTest(targetTestRotationRate, rotateTestDuration, rotateTestSteps),
                rotateTest(-targetTestRotationRate, rotateTestDuration, rotateTestSteps),
                pointWheelsTest(targetTestVelocity, pointWheelsTestDuration, pointWheelsTestSteps, false),
                pointWheelsTest(targetTestVelocity, pointWheelsTestDuration, pointWheelsTestSteps, true));
    }

    public Command proveOut() {
        return Commands.sequence(
                driveProveOut(),
                rollerTestCommand(),
                loaderTestCommand(),
                hopperTestCommand(),
                intakeTestCommand(),
                acceleratorTestCommand(),
                shooterTestCommand());
        // Add Climber Eventually
    }
}
