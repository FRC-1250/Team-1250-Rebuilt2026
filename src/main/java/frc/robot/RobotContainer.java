// Copyright (c) FIRST and other WPILib contributors
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project
package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FuelLine;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.intakePositions;
import frc.robot.subsystems.Shooter.ShooterVelocity;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.FuelLine.LoaderCamVelocityControl;
import frc.robot.telemetry.HealthMonitor;
import frc.robot.utility.FieldZones;

@Logged
public class RobotContainer {

    /* Subsystems */
    private final CommandSwerveDrivetrain swerve = TunerConstants.createDrivetrain();
    private final FuelLine fuelLine = new FuelLine();
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    private final Climber climber = new Climber();
    private final Limelight limelight = new Limelight();
    private final Leds leds = new Leds();
    public final CommandFactory commandFactory = new CommandFactory(
            swerve,
            fuelLine,
            intake,
            shooter,
            climber,
            limelight,
            leds);

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
    private final EventLoop twoPlayer = new EventLoop();

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(14, -18, 0);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(14, -18, 0);
    private final CommandXboxController primary = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable commandInputs = inst.getTable("SmartDashboard/Commands/Inputs");
    private final DoubleEntry intakeVelocity = commandInputs.getDoubleTopic("intakeVelocity").getEntry(0.0);
    private final DoubleEntry rollerVelocity = commandInputs.getDoubleTopic("rollerVelocity").getEntry(0.0);
    private final DoubleEntry loaderCamVelocity = commandInputs.getDoubleTopic("loaderCamVelocity").getEntry(0.0);
    private final DoubleEntry loaderCamPosition = commandInputs.getDoubleTopic("loaderCamPosition").getEntry(0.0);;
    private final DoubleEntry fuelAcceleratorVelocity = commandInputs.getDoubleTopic("fuelAcceleratorVelocity")
            .getEntry(0.0);
    private final DoubleEntry fuelShooterVelocity = commandInputs.getDoubleTopic("fuelShooterVelocity").getEntry(0.0);
    private final DoubleEntry climberPosition = commandInputs.getDoubleTopic("climberPosition").getEntry(0.0);

    @NotLogged
    private final Trigger blueAlliance = new Trigger(() -> DriverStation.getAlliance().get() == Alliance.Blue);

    @NotLogged
    private final Trigger redAlliance = new Trigger(() -> DriverStation.getAlliance().get() == Alliance.Red);
    private final Trigger isInBlueSide = new Trigger(() -> FieldZones.blueSide.isRobotInZone(swerve.getState().Pose));
    private final Trigger isInRedSide = new Trigger(() -> FieldZones.redSide.isRobotInZone(swerve.getState().Pose));
    private final Trigger isInCenterBlueOutpostRedDepotZone = new Trigger(
            () -> FieldZones.centerBlueOutpostRedDepot.isRobotInZone(swerve.getState().Pose));
    private final Trigger isInCenterBlueDepotRedOutpostZone = new Trigger(
            () -> FieldZones.centerBlueDepotRedOutpost.isRobotInZone(swerve.getState().Pose));
    /* Auto */
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        configureRobotOperationMode();
        configureSinglePlayerBindings();
        configureTwoPlayerBindings();
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

        swerve.registerTelemetry(logger::telemeterize);
    }

    private void changeEventLoop(EventLoop loop) {
        CommandScheduler.getInstance().setActiveButtonLoop(loop);
    }

    private void configureRobotOperationMode() {
        robotOperationModeChooser.setDefaultOption("Single player", singlePlayer);
        robotOperationModeChooser.addOption("Two player", twoPlayer);
        robotOperationModeChooser.onChange(this::changeEventLoop);
        SmartDashboard.putData("Robot operation mode", robotOperationModeChooser);
    }

    private void configureSinglePlayerBindings() {
        configureCommonBindings(singlePlayer);

        primary.rightTrigger(0.5, singlePlayer).whileTrue(Commands.none()); // Shoot
        primary.leftTrigger(0.5, singlePlayer).whileTrue(Commands.none()); // Tageting
        primary.rightBumper(singlePlayer).onTrue(Commands.none()); // Intake out
        primary.leftBumper(singlePlayer).onTrue(Commands.none()); // Intake in
        primary.y(singlePlayer).onTrue(Commands.none()); // Climb foeward
        primary.b(singlePlayer).onTrue(Commands.none()); // Climb back
        primary.x(singlePlayer).onTrue(Commands.none()); // drive to tower position
        primary.a(singlePlayer).and(isInBlueSide).and(blueAlliance)
                .whileTrue(swerve.applyRequest(() -> driveWithAngle
                        .withVelocityX(yLimiter.calculate(-primary.getLeftY() * MaxSpeed))
                        .withVelocityY(xLimiter.calculate(-primary.getLeftX() * MaxSpeed))
                        .withHeadingPID(8, 0, 0)
                        .withTargetDirection(commandFactory.determineHeading(FieldPositions.blueHub)))
                        .withName("Point centric swerve")); // Toggle auto heading
        primary.a(singlePlayer).and(isInCenterBlueOutpostRedDepotZone).and(blueAlliance)
                .whileTrue(swerve.applyRequest(() -> driveWithAngle
                        .withVelocityX(yLimiter.calculate(-primary.getLeftY() * MaxSpeed))
                        .withVelocityY(xLimiter.calculate(-primary.getLeftX() * MaxSpeed))
                        .withHeadingPID(8, 0, 0)
                        .withTargetDirection(commandFactory.determineHeading(FieldPositions.blueOutpostSide)))
                        .withName("Point centric swerve")); // Toggle auto heading
        primary.a(singlePlayer).and(isInCenterBlueDepotRedOutpostZone).and(blueAlliance)
                .whileTrue(swerve.applyRequest(() -> driveWithAngle
                        .withVelocityX(yLimiter.calculate(-primary.getLeftY() * MaxSpeed))
                        .withVelocityY(xLimiter.calculate(-primary.getLeftX() * MaxSpeed))
                        .withHeadingPID(8, 0, 0)
                        .withTargetDirection(commandFactory.determineHeading(FieldPositions.blueDepotSide)))
                        .withName("Point centric swerve")); // Toggle auto heading
        primary.a(singlePlayer).and(isInRedSide).and(redAlliance)
                .whileTrue(swerve.applyRequest(() -> driveWithAngle
                        .withVelocityX(yLimiter.calculate(-primary.getLeftY() * MaxSpeed))
                        .withVelocityY(xLimiter.calculate(-primary.getLeftX() * MaxSpeed))
                        .withHeadingPID(8, 0, 0)
                        .withTargetDirection(commandFactory.determineHeading(FieldPositions.redHub)))
                        .withName("Point centric swerve")); // Toggle auto heading
        primary.a(singlePlayer).and(isInCenterBlueOutpostRedDepotZone).and(redAlliance)
                .whileTrue(swerve.applyRequest(() -> driveWithAngle
                        .withVelocityX(yLimiter.calculate(-primary.getLeftY() * MaxSpeed))
                        .withVelocityY(xLimiter.calculate(-primary.getLeftX() * MaxSpeed))
                        .withHeadingPID(8, 0, 0)
                        .withTargetDirection(commandFactory.determineHeading(FieldPositions.redDepotSide)))
                        .withName("Point centric swerve")); // Toggle auto heading
        primary.a(singlePlayer).and(isInCenterBlueDepotRedOutpostZone).and(redAlliance)
                .whileTrue(swerve.applyRequest(() -> driveWithAngle
                        .withVelocityX(yLimiter.calculate(-primary.getLeftY() * MaxSpeed))
                        .withVelocityY(xLimiter.calculate(-primary.getLeftX() * MaxSpeed))
                        .withHeadingPID(8, 0, 0)
                        .withTargetDirection(commandFactory.determineHeading(FieldPositions.redOutpostSide)))
                        .withName("Point centric swerve")); // Toggle auto heading
    }

    private void configureTwoPlayerBindings() {
        configureCommonBindings(twoPlayer);
    }

    private void configureDevBindings() {
        intakeVelocity.set(0);
        rollerVelocity.set(0);
        loaderCamVelocity.set(0);
        loaderCamPosition.set(0);
        fuelAcceleratorVelocity.set(0);
        fuelShooterVelocity.set(0);
        climberPosition.set(0);

        SmartDashboard.putData("Commands/Intake/Set intake velocity",
                commandFactory.cmdSetIntakeVelocity(() -> intakeVelocity.get()));

        SmartDashboard.putData("Commands/Fuel line/Set roller velocity",
                commandFactory.cmdSetRollerVelocity(() -> rollerVelocity.get()));
        SmartDashboard.putData("Commands/Fuel line/Set cam velocity",
                commandFactory.cmdSetLoaderCamVelocity(() -> loaderCamVelocity.get()));
        SmartDashboard.putData("Commands/Fuel line/Set cam position",
                commandFactory.cmdSetLoaderCamPosition(() -> loaderCamPosition.get()));

        SmartDashboard.putData("Commands/Shooter/Set fuel accel velocity",
                commandFactory.cmdSetFuelAcceleratorVelocity(() -> fuelAcceleratorVelocity.get()));
        SmartDashboard.putData("Commands/Shooter/Set full shoot velocity",
                commandFactory.cmdSetFuelShooterVelocity(() -> fuelShooterVelocity.get()));
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

    private void configureAutoCommands() {
        /*
         * Do nothing as default is a human safety condition, this should always be the
         * default
         */
        autoChooser.setDefaultOption("Do nothing", new WaitCommand(15));
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureNamedCommands() {
        final double fireTimeout = 3;

        NamedCommands.registerCommand("extend_hopper",
                commandFactory.cmdSetHopperPosition(intakePositions.Extend.rotations));

        /*
         * Make into one command
         * - Extend hopper
         * - Turn on intake wheels
         * - Turn on fuelline rollers
         */

        NamedCommands.registerCommand("fire_fuel_with_timeout",
                commandFactory.cmdFireFuel(ShooterVelocity.HUB, LoaderCamVelocityControl.three_bps)
                        .withTimeout(fireTimeout));
    }
}
