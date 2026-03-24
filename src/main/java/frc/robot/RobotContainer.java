// Copyright (c) FIRST and other WPILib contributors
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project
package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FuelLine;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter.ShooterVelocity;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ReactionBar;
import frc.robot.subsystems.Shooter;
import frc.robot.telemetry.HealthMonitor;

public class RobotContainer {

    /* Subsystems */
    private final CommandSwerveDrivetrain swerve = TunerConstants.createDrivetrain();

    @Logged(name = "Fuel line")
    private final FuelLine fuelLine = new FuelLine();

    @Logged(name = "Intake")
    private final Intake intake = new Intake();

    @Logged(name = "Shooter")
    private final Shooter shooter = new Shooter();

    private final Limelight limelight = new Limelight();
    private final Limelight limelightRear = new Limelight("rear");

    @Logged(name = "Reaction bar")
    private final ReactionBar reactionBar = new ReactionBar();

    public final CommandFactory commandFactory = new CommandFactory(
            swerve,
            fuelLine,
            intake,
            shooter,
            List.of(limelight, limelightRear),
            reactionBar);

    private final Telemetry logger = new Telemetry();

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    /* Bindings */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentricFacingAngle driveWithAngle = new FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SendableChooser<EventLoop> robotOperationModeChooser = new SendableChooser<>();
    private final EventLoop singlePlayer = new EventLoop();

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(14, -18, 0);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(14, -18, 0);
    private final CommandXboxController primary = new CommandXboxController(0);

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable commandInputs = inst.getTable("SmartDashboard/Commands/Inputs");
    private final DoubleEntry intakeVelocity = commandInputs.getDoubleTopic("intakeVelocity").getEntry(0.0);
    private final DoubleEntry rollerVelocity = commandInputs.getDoubleTopic("rollerVelocity").getEntry(0.0);
    private final DoubleEntry loaderLoaderVelocity = commandInputs.getDoubleTopic("loaderLoaderVelocity").getEntry(0.0);
    private final DoubleEntry shooterVelocity = commandInputs.getDoubleTopic("shooterVelocity").getEntry(0.0);
    private final DoubleEntry hopperposition = commandInputs.getDoubleTopic("hopperPosition").getEntry(0.0);

    private Trigger robotIsAligned;

    /* Auto */
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        configureRobotOperationMode();
        configureSinglePlayerBindings();
        configureDevBindings();
        changeEventLoop(singlePlayer);
        configureNamedCommands();
        configureAutoCommands();
        configureHealthMonitor();
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void configureHealthMonitor() {
        HealthMonitor hm = HealthMonitor.getInstance();
        fuelLine.registerWithHealthMonitor(hm);
        intake.registerWithHealthMonitor(hm);
        shooter.registerWithHealthMonitor(hm);
        swerve.registerWithHealthMonitor(hm);
        swerve.registerTelemetry(logger::telemeterize);
    }

    private void changeEventLoop(EventLoop loop) {
        CommandScheduler.getInstance().setActiveButtonLoop(loop);
    }

    private void configureRobotOperationMode() {
        robotOperationModeChooser.setDefaultOption("Single player", singlePlayer);
        robotOperationModeChooser.onChange(this::changeEventLoop);
        SmartDashboard.putData("Robot operation mode", robotOperationModeChooser);
    }

    private void configureSinglePlayerBindings() {
        configureCommonBindings(singlePlayer);

        robotIsAligned = new Trigger(singlePlayer,
                () -> commandFactory.targetManager.getTargetingState().isAligned());

        primary.start(singlePlayer)
                .onTrue(swerve.runOnce(() -> swerve.seedFieldCentric()).withName("Reseed swerve"));

        primary.rightTrigger(0.5, singlePlayer).and(primary.leftTrigger(0.5, singlePlayer)).and(robotIsAligned)
                .whileTrue(commandFactory.cmdFireFuel(
                        () -> commandFactory.getVelocityBasedOnTargetDistance())
                        .withName("Fire by distance")); // Shoot

        primary.rightTrigger(0.5, singlePlayer).and(primary.leftTrigger(0.5, singlePlayer).negate())
                .whileTrue(commandFactory.cmdFireFuel(ShooterVelocity.TOWER.rotationsPerSecond)
                        .withName("Fire")); // Shoot

        primary.leftTrigger(0.5, singlePlayer).whileTrue(
                swerve.applyRequest(
                        () -> driveWithAngle
                                .withVelocityX(yLimiter.calculate(-primary.getLeftY() * MaxSpeed))
                                .withVelocityY(xLimiter.calculate(-primary.getLeftX() * MaxSpeed))
                                .withHeadingPID(15, 0, 0)
                                .withTargetDirection(commandFactory.getRotationToTargetBasedOnZone()))
                        .withName("Point centric swerve"));

        primary.b(singlePlayer).whileTrue(
                swerve.applyRequest(
                        () -> driveWithAngle
                                .withVelocityX(yLimiter.calculate(-primary.getLeftY() * (MaxSpeed / 3)))
                                .withVelocityY(xLimiter.calculate(-primary.getLeftX() * (MaxSpeed / 3)))
                                .withHeadingPID(15, 0, 0)
                                .withTargetDirection(Rotation2d.k180deg))
                        .withName("Snap backwards"));

        primary.a(singlePlayer).whileTrue(
                swerve.applyRequest(
                        () -> driveWithAngle
                                .withVelocityX(yLimiter.calculate(-primary.getLeftY() * (MaxSpeed / 3)))
                                .withVelocityY(xLimiter.calculate(-primary.getLeftX() * (MaxSpeed / 3)))
                                .withHeadingPID(15, 0, 0)
                                .withTargetDirection(Rotation2d.kZero))
                        .withName("Snap forward"));

        primary.rightBumper(singlePlayer)
                .onTrue(commandFactory.cmdActivateFuelPickUp().withName("Activate fuel pick up")); // Intake out
        primary.leftBumper(singlePlayer)
                .onTrue(commandFactory.cmdDeactivateFuelPickUp().withName("Deactivate fuel pick up")); // Intake in

        primary.pov(0, 0, singlePlayer).onTrue(Commands.none()); // Climb
        primary.pov(0, 180, singlePlayer).onTrue(Commands.none()); // Unclim
    }

    private void configureCommonBindings(EventLoop loop) {
        swerve.setDefaultCommand(
                swerve.applyRequest(() -> drive
                        .withVelocityX(yLimiter.calculate(-primary.getLeftY() * MaxSpeed))
                        .withVelocityY(xLimiter.calculate(-primary.getLeftX() * MaxSpeed))
                        .withRotationalRate(-primary.getRightX() * MaxAngularRate))
                        .withName("Field centric swerve"));

        configureDevBindings();
    }

    private void addPathAuto(String name, String pathName) {
        try {
            autoChooser.addOption(name, new PathPlannerAuto(pathName));
        } catch (Exception e) {
            // Exceptions are now caught in the PathPlannerAuto constructor and this should
            // never run. Leaving it in place to catch any edge cases.
            DataLogManager.log(String.format("GatorBot: Not able to build auto routines! %s", e.getMessage()));
        }
    }

    private void configureDevBindings() {
        intakeVelocity.set(0);
        rollerVelocity.set(0);
        loaderLoaderVelocity.set(0);
        shooterVelocity.set(0);
        hopperposition.set(0);

        SmartDashboard.putData("Commands/Intake/Set intake velocity",
                commandFactory.cmdSetIntakeVelocity(() -> intakeVelocity.get()));
        SmartDashboard.putData("Commands/Intake/Set hopper position",
                commandFactory.cmdSetHopperPosition(() -> hopperposition.get()));
        SmartDashboard.putData("Commands/Intake/Stop intake velocity",
                commandFactory.cmdSetIntakeVelocity(0));
        SmartDashboard.putData("Commands/Intake/Reset hopper position",
                commandFactory.cmdResetHopperPosition());
        SmartDashboard.putData("Commands/Intake/Reset hopper position (retract)",
                commandFactory.cmdResetHopperPositionRetract());
        SmartDashboard.putData("Commands/Intake/Reset hopper position (extend)",
                commandFactory.cmdResetHopperPositionWithExtend());

        SmartDashboard.putData("Commands/Fuel line/Set roller velocity",
                commandFactory.cmdSetRollerVelocity(() -> rollerVelocity.get()));
        SmartDashboard.putData("Commands/Fuel line/Set Loader velocity",
                commandFactory.cmdSetLoaderVelocity(() -> loaderLoaderVelocity.get()));
        SmartDashboard.putData("Commands/Fuel line/Stop roller velocity",
                commandFactory.cmdSetRollerVelocity(0));
        SmartDashboard.putData("Commands/Fuel line/Stop Loader velocity",
                commandFactory.cmdSetLoaderVelocity(0));

        SmartDashboard.putData("Commands/Shooter/Set shoot velocity",
                commandFactory.cmdSetFuelShooterVelocity(() -> shooterVelocity.get())
                        .andThen(commandFactory.cmdSetFuelAcceleratorVelocity(() -> shooterVelocity.get())));
        SmartDashboard.putData("Commands/Shooter/Stop shooter",
                commandFactory.cmdSetFuelShooterVelocity(0).andThen(commandFactory.cmdSetFuelAcceleratorVelocity(0)));

        SmartDashboard.putData("Commands/Shared/Fire fuel",
                commandFactory.cmdFireFuel(() -> shooterVelocity.get()));

        SmartDashboard.putData("Commands/Shared/Fire fuel no agi",
                commandFactory.cmdFireFuelNoAgitation(() -> shooterVelocity.get()));

        SmartDashboard.putData("Commands/Shared/Prove out", commandFactory.proveOut());
        SmartDashboard.putData("Commands/Drivetrain/Prove out", commandFactory.driveProveOut());
    }

    private void configureAutoCommands() {
        /*
         * Do nothing as default is a human safety condition, this should always be the
         * default
         */
        autoChooser.setDefaultOption("Do nothing", new WaitCommand(15));
        addPathAuto("LeftCenterQuadrant", "LeftCenterQuadrant");
        addPathAuto("RightCenterQuadrant", "RightCenterQuadrant");
        addPathAuto("LeftDepot", "LeftDepot");
        addPathAuto("RightOutpost", "RightOutpost");
        addPathAuto("DepotOutpost", "DepotOutpost");
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureNamedCommands() {
        final double fireTimeout = 5;
        final double aimingTimeout = 2;

        NamedCommands.registerCommand("shooter_prep", commandFactory.cmdWarmUpShooter());
        NamedCommands.registerCommand("fire_fuel",
                commandFactory.cmdFireFuel(ShooterVelocity.TOWER.rotationsPerSecond)
                        .withTimeout(fireTimeout));

        NamedCommands.registerCommand("fire_fuel_by_distance",
                swerve.applyRequest(
                        () -> driveWithAngle
                                .withVelocityX(0)
                                .withVelocityY(0)
                                .withHeadingPID(15, 0, 0)
                                .withTargetDirection(commandFactory.getRotationToTargetBasedOnZone()))
                        .withTimeout(aimingTimeout).andThen(

                                commandFactory.cmdFireFuel(() -> commandFactory.getVelocityBasedOnTargetDistance())
                                        .withTimeout(fireTimeout)));

        NamedCommands.registerCommand("activate_fuel_pick_up", commandFactory.cmdActivateFuelPickUp());
        NamedCommands.registerCommand("reset_starting_fuel", commandFactory.cmdResetStartingFuel());

    }
}
