package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FuelLine;
import frc.robot.subsystems.FuelLine.LoaderVelocity;
import frc.robot.subsystems.FuelLine.RollerVelocity;
import frc.robot.subsystems.Shooter.ShooterVelocity;
import frc.robot.utility.RobotLocalization;
import frc.robot.utility.TargetManager;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.HopperPosition;
import frc.robot.subsystems.Intake.IntakeVelocity;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ReactionBar;
import frc.robot.subsystems.ReactionBar.ReactionBarPosition;
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
    public final RobotLocalization robotLocalization;
    public final TargetManager targetManager;

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
        robotLocalization = new RobotLocalization(limelight, swerve);
        targetManager = new TargetManager(swerve);
    }

    /*
     * Drive
     */

    public void updateTargetState() {
        targetManager.updateTargetState(robotLocalization.getActiveZones(),
                DriverStation.getAlliance().orElse(Alliance.Blue));
        SmartDashboard.putString("Targeting State", targetManager.getTargetingState().toString());
    }

    public Rotation2d getRotationToTargetBasedOnZone() {
        return targetManager.getTargetingState().rotation();
    }

    public double getVelocityBasedOnTargetDistance() {
        return shooter.getInterpolatedVelocity(targetManager.getTargetingState().distance());
    }

    /*
     * ReactionBar
     */

    public Command cmdSetReactionBarPosition(DoubleSupplier supplier) {
        return Commands.runOnce(() -> reactionBar.setReactionBarPosition(supplier.getAsDouble()), reactionBar)
                .andThen(
                        Commands.waitUntil((() -> reactionBar.isReactionBarNearPosition(supplier.getAsDouble(), .08))));
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
        return Commands.sequence(
                Commands.runOnce(() -> intake.setHopperPosition(supplier.getAsDouble()), intake),
                Commands.waitUntil(() -> intake.isHopperNearPosition(supplier.getAsDouble(), 0.2)));
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

    public Command cmdSetHopperNeutralMode(NeutralModeValue neutralModeValue) {
        return Commands.runOnce(() -> intake.setHopperNeutralMode(neutralModeValue), intake);
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

    public Command cmdSetClimberPosition(double rotations) {
        return cmdSetClimberPosition(() -> rotations);
    }

    /*
     * Shared
     */

    public Command cmdFireFuel(DoubleSupplier supplier) {
        return Commands.runEnd(
                () -> {
                    var rotationsPerSecond = supplier.getAsDouble();
                    shooter.setAcceleratorVelocity(rotationsPerSecond);
                    shooter.setShooterVelocity(rotationsPerSecond);
                    if (shooter.isAcceleratorNearRotationsPerSecond(rotationsPerSecond, 2)
                            && shooter.isShooterNearRotationsPerSecond(rotationsPerSecond, 2)) {
                        fuelLine.setLoaderVelocity(LoaderVelocity.FIRE.rotationsPerSecond);
                        fuelLine.setRollerVelocity(RollerVelocity.GO.rotationsPerSecond);
                        intake.agitate();
                    } else {
                        fuelLine.stopLoader();
                        fuelLine.stopRoller();
                    }
                },
                () -> {
                    shooter.setAcceleratorVelocity(ShooterVelocity.WARM.rotationsPerSecond);
                    shooter.setShooterVelocity(ShooterVelocity.WARM.rotationsPerSecond);
                    fuelLine.stopLoader();
                    fuelLine.stopRoller();
                    intake.resetAgitation();
                    intake.setHopperPosition(HopperPosition.EXTENDED.rotations);
                    intake.stopHopper();
                }, shooter, fuelLine);
    }

    public Command cmdFireFuelNoAgitation(DoubleSupplier supplier) {
        return Commands.runEnd(
                () -> {
                    var rotationsPerSecond = supplier.getAsDouble();
                    shooter.setAcceleratorVelocity(rotationsPerSecond);
                    shooter.setShooterVelocity(rotationsPerSecond);
                    if (shooter.isAcceleratorNearRotationsPerSecond(rotationsPerSecond, 2)
                            && shooter.isShooterNearRotationsPerSecond(rotationsPerSecond, 2)) {
                        fuelLine.setLoaderVelocity(LoaderVelocity.FIRE.rotationsPerSecond);
                        fuelLine.setRollerVelocity(RollerVelocity.GO.rotationsPerSecond);
                    } else {
                        fuelLine.stopLoader();
                        fuelLine.stopRoller();
                    }
                },
                () -> {
                    shooter.setAcceleratorVelocity(ShooterVelocity.WARM.rotationsPerSecond);
                    shooter.setShooterVelocity(ShooterVelocity.WARM.rotationsPerSecond);
                    fuelLine.stopLoader();
                    fuelLine.stopRoller();
                }, shooter, fuelLine);
    }

    public Command cmdAgitateFuelWithHopper() {
        return Commands.runEnd(
                () -> {
                    intake.agitate();
                }, () -> {
                    intake.resetAgitation();
                }, intake);
    }

    public Command cmdAgitateFuelWithReactionBar() {
        return Commands.runEnd(
                () -> {
                    reactionBar.agitate();
                }, () -> {
                    reactionBar.resetAgitation();
                }, reactionBar);
    }

    public Command cmdFireFuel(double rotationsPerSecond) {
        return cmdFireFuel(() -> rotationsPerSecond);
    }

    public Command cmdActivateFuelPickUp() {
        return Commands.sequence(
                cmdSetHopperPosition(HopperPosition.EXTENDED.rotations),
                cmdStopHopper(),
                cmdSetReactionBarPosition(ReactionBarPosition.EXTENDED.rotations),
                cmdSetIntakeVelocity(IntakeVelocity.GO.rotationsPerSecond))
                .withName("Activate fuel pick up");
    }

    public Command cmdDeactivateFuelPickUp() {
        return Commands.sequence(
                cmdStopIntake(),
                cmdStopReactionBar(),
                cmdSetHopperPosition(HopperPosition.HOME.rotations))
                .withName("Deactivate fuel pick up");
    }

    public Command cmdStopReactionBar() {
        return Commands.runOnce(() -> reactionBar.stopReactionBar(), reactionBar);
    }

    public Command cmdWarmUpShooter() {
        return cmdSetFuelShooterVelocity(ShooterVelocity.WARM.rotationsPerSecond)
                .andThen(cmdSetFuelAcceleratorVelocity(ShooterVelocity.WARM.rotationsPerSecond));
    }

    public Command cmdResetStartingFuel() {
        return Commands.sequence(
                cmdSetHopperPosition(HopperPosition.EXTENDED.rotations),
                cmdStopHopper(),
                cmdSetReactionBarPosition(ReactionBarPosition.EXTENDED.rotations),
                cmdSetFuelShooterVelocity(ShooterVelocity.UNJAM.rotationsPerSecond),
                cmdSetFuelAcceleratorVelocity(ShooterVelocity.UNJAM.rotationsPerSecond),
                cmdSetLoaderVelocity(LoaderVelocity.UNJAM.rotationsPerSecond),
                cmdSetRollerVelocity(RollerVelocity.UNJAM.rotationsPerSecond));
    }

    /*
     * Prove Out
     */

    private Command shooterTestCommand() {
        return Commands.sequence(
                cmdSetFuelShooterVelocity(ShooterVelocity.TOWER.rotationsPerSecond),
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
                cmdStopLoader());

    }

    private Command acceleratorTestCommand() {
        return Commands.sequence(
                cmdSetFuelAcceleratorVelocity(ShooterVelocity.TOWER.rotationsPerSecond),
                Commands.waitSeconds(5),
                cmdStopAccelerator());
    }

    private Command climberTestCommand() {
        return Commands.sequence(
                cmdSetClimberPosition(ClimberPosition.HOME.rotations),
                Commands.waitSeconds(2),
                cmdSetClimberPosition(ClimberPosition.CLIMB.rotations),
                Commands.waitSeconds(2),
                cmdSetClimberPosition(ClimberPosition.HOME.rotations));
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
    }
}
