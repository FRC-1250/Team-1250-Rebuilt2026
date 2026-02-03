// Copyright (c) FIRST and other WPILib contributors
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project
package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FuelLine;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.telemetry.HealthMonitor;

@Logged
public class RobotContainer {

    /* Subsystems */
    private final CommandSwerveDrivetrain SwerveDriveState = TunerConstants.createDrivetrain();
    private final FuelLine fuelLine = new FuelLine();
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    private final Climber climber = new Climber();
    private final Limelight limelight = new Limelight();
    private final Leds leds = new Leds();
    public Translation2d blueHub = new Translation2d(4.791, 3.978);
    public Translation2d redHub = new Translation2d(12, 4.155);
    public Translation2d blueDepotSide = new Translation2d(1.45, 6.7);
    public Translation2d blueOutpostSide = new Translation2d(1.45, 1.48);
    public Translation2d redDepotSide = new Translation2d(14.596, 1.48);
    public Translation2d redOutpostSide = new Translation2d(14.596, 6.801);
    public final CommandFactory commandFactory = new CommandFactory(
            SwerveDriveState,
            fuelLine,
            intake,
            shooter,
            climber,
            limelight,
            leds);

    private final Telemetry logger = new Telemetry();

    // kSpeedAt12Volts desired top speed
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    // 3/4 of a rotation per second, max angular velocity
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentricFacingAngle driveWithAngle = new FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    /* Bindings */
    private final SendableChooser<EventLoop> controllerModeChooser = new SendableChooser<>();
    private final EventLoop singlePlayer = new EventLoop();
    private final EventLoop twoPlayer = new EventLoop();
    private final EventLoop devPlayer = new EventLoop();

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(14, -18, 0);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(14, -18, 0);
    private final CommandXboxController DriverJoystick = new CommandXboxController(0);
    private final CommandXboxController DevJoystick = new CommandXboxController(1);

    private DoubleSubscriber intakeVelocity;
    private DoubleSubscriber conveyorBeltVelocity;
    private DoubleSubscriber camVelocity;
    private DoubleSubscriber preShooterVelocity;
    private DoubleSubscriber shooterVelocity;
    private DoubleSubscriber climbPosition;

    private void initilizeNetworkTableWidgets() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable datatable = inst.getTable("commands");
        intakeVelocity = datatable.getDoubleTopic("intakeVelocity").subscribe(0.0);
        conveyorBeltVelocity = datatable.getDoubleTopic("conveyorBeltVelocity").subscribe(0.0);
        camVelocity = datatable.getDoubleTopic("camVelocity").subscribe(0.0);
        preShooterVelocity = datatable.getDoubleTopic("preShooterVelocity").subscribe(0.0);
        shooterVelocity = datatable.getDoubleTopic("shooterVelocity").subscribe(0.0);
        climbPosition = datatable.getDoubleTopic("climbPosition").subscribe(0.0);
    }

    /* Auto */
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        configureSinglePlayerBindings();
        configureTwoPlayerBindings();
        configureDevPlayerBindings();
        changeEventLoop(singlePlayer);
        configureNamedCommands();
        configureAutoCommands();
        configureControlLoopChooser();
        configureMonitors();
        SwerveDriveState.registerTelemetry(logger::telemeterize);
        initilizeNetworkTableWidgets();
    }

    private void configureMonitors() {
        HealthMonitor hm = HealthMonitor.getInstance();
        fuelLine.registerWithHealthMonitor(hm);
        intake.registerWithHealthMonitor(hm);
    }

    /* Bindings */

    public BooleanSupplier isEventLoopScheduled(EventLoop loop) {
        return () -> CommandScheduler.getInstance().getActiveButtonLoop().equals(loop);
    }

    public void changeEventLoop(EventLoop loop) {
        CommandScheduler.getInstance().setActiveButtonLoop(loop);
    }

    private void configureControlLoopChooser() {
        controllerModeChooser.setDefaultOption("Single player", singlePlayer);
        controllerModeChooser.addOption("Two player", twoPlayer);
        controllerModeChooser.addOption("Dev", twoPlayer);
        controllerModeChooser.onChange(this::changeEventLoop);
        SmartDashboard.putData("Control Chooser", controllerModeChooser);
    }

    private void configureSinglePlayerBindings() {
        SmartDashboard.putData(commandFactory.cmdSetIntakeVelocity(0));

        SwerveDriveState.setDefaultCommand(
                SwerveDriveState.applyRequest(() -> drive
                        .withVelocityX(yLimiter.calculate(-DriverJoystick.getLeftY() * MaxSpeed))
                        .withVelocityY(xLimiter.calculate(-DriverJoystick.getLeftX() * MaxSpeed))
                        .withRotationalRate(-DriverJoystick.getRightX() * MaxAngularRate))
                        .withName("Field centric swerve"));

        DriverJoystick.rightTrigger().whileTrue(Commands.none()); // Shoot
        DriverJoystick.leftTrigger().whileTrue(Commands.none()); // Tageting
        DriverJoystick.rightBumper().onTrue(Commands.none()); // Intake out
        DriverJoystick.leftBumper().onTrue(Commands.none()); // Intake in
        DriverJoystick.y().onTrue(Commands.none()); // Climb foeward
        DriverJoystick.b().onTrue(Commands.none()); // Climb back
        DriverJoystick.x().onTrue(Commands.none()); // drive to tower position
        DriverJoystick.a(singlePlayer).whileTrue(SwerveDriveState.applyRequest(() -> driveWithAngle
                .withVelocityX(yLimiter.calculate(-DriverJoystick.getLeftY() * MaxSpeed))
                .withVelocityY(xLimiter.calculate(-DriverJoystick.getLeftX() * MaxSpeed))
                .withHeadingPID(8, 0, 0)
                .withTargetDirection(commandFactory.determineHeading(new Translation2d(4.791, 3.978))))
                .withName("Point centric swerve")); // Toggle auto heading

    }

    private void configureTwoPlayerBindings() {

    }

    private void configureDevPlayerBindings() {
        DevJoystick.rightTrigger().whileTrue(Commands.none()); // Shoot
        DevJoystick.leftTrigger().whileTrue(Commands.none()); // Tageting
        DevJoystick.rightBumper().onTrue(Commands.none()); // Intake out
        DevJoystick.leftBumper().onTrue(Commands.none()); // Intake in
        DevJoystick.y().onTrue(Commands.none()); // Climb foeward
        DevJoystick.b().onTrue(Commands.none()); // Climb back
        DevJoystick.x().onTrue(Commands.none()); // drive to tower position

    }

    private void configureCommonBindings(EventLoop loop) {

    }

    /* Auto */

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
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

    }
}
