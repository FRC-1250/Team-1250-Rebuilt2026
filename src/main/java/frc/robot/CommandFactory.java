package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandFactory.NetworkTableWidget;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.SwerveVisionLogic;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FuelLine;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TunerConstants;

public class CommandFactory {

    private final CommandSwerveDrivetrain cmdSwerveDriveState;
    private final FuelLine fuelLine;
    private final Shooter shooter;
    private final Climber climber;
    private final Limelight cmdLimelight;
    private final Leds leds;
    private final Intake intake;

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
        initilizeNetworkTableWidgets();
    }

    public enum NetworkTableWidget {
        IntakeVelocity,
        ConveyorBeltVelocity,
        CamVelocity,
        PreShooterVelocity,
        ShooterVelocity,
        ClimbPosition;
    }

    private void initilizeNetworkTableWidgets() {
        for (NetworkTableWidget widget : NetworkTableWidget.values()) {
            SmartDashboard.putNumber(widget.name(), 0);
        }
    }

    /*
     * Intake
     */

    public Command cmdSetIntakeVelocityNT() {
        double velocity = SmartDashboard.getNumber(NetworkTableWidget.IntakeVelocity.name(), 0);
        return cmdSetIntakeVelocity(velocity);
    }

    public Command cmdSetIntakeVelocity(double rotationsPerSecond) {
        return Commands.runOnce(
                () -> intake.setVelocity(rotationsPerSecond, Voltage.ofBaseUnits(0, Units.Volts)),
                intake);
    }

    public Command cmdExtendIntake() {
        return Commands.runOnce(
                () -> intake.extend(),
                intake);
    }

    public Command cmdRetractIntake() {
        return Commands.runOnce(
                () -> intake.retract(),
                intake);
    }

    /*
     * FuelLine
     */
    public Command cmdSetConveyorBeltVelocity() {
        return Commands.runOnce(
                () -> fuelLine.setConveyorVelocity(0, Voltage.ofBaseUnits(0, Units.Volts)),
                fuelLine);
    }

    /*
     * shooter
     */
    public Command cmdSetLoderCamVelocity(double Velocity) {
        return Commands.runOnce(
                () -> shooter.setLoaderCamVelocity(Velocity));
    }

    public Command cmdSetLoderCamPosition(double Rotations) {
        return Commands.runOnce(
                () -> shooter.setLoaderCamPosition(Rotations));
    }

}