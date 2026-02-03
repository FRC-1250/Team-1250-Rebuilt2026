package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.Supplier;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.commands.SwerveVisionLogic;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FuelLine;
import frc.robot.subsystems.Intake;
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
    }

    /*
     * Intake
     */

    public Command cmdSetIntakeVelocity(Supplier<Double> suppler) {
        return Commands.runOnce(
                () -> intake.setIntakeVelocity(suppler.get(), Voltage.ofBaseUnits(0, Units.Volts)),
                intake);
    }

    public Command cmdSetHopperPosition(Supplier<Double> suppler) {
        return Commands.runOnce(
                () -> intake.setIntakeVelocity(suppler.get(), Voltage.ofBaseUnits(0, Units.Volts)),
                intake);
    }

    /*
     * FuelLine
     */
    public Command cmdSetConveyorBeltVelocity(Supplier<Double> suppler) {
        return Commands.runOnce(
                () -> fuelLine.setConveyorVelocity(0, Voltage.ofBaseUnits(0, Units.Volts)),
                fuelLine);
    }

    /*
     * shooter
     */
    public Command cmdSetLoaderCamVelocity(Supplier<Double> suppler) {
        return Commands.runOnce(
                () -> shooter.setLoaderCamPosition(suppler.get()), shooter);
    }

    public Command cmdSetLoaderCamPosition(Supplier<Double> suppler) {
        return Commands.runOnce(
                () -> shooter.setLoaderCamPosition(suppler.get()), shooter);
    }

    public Command cmdSetprecursorFlywheelVelocity(double rotationsPerSecond) {
        return Commands.runOnce(
                () -> shooter.setPrecursorFlywheelVelocity(rotationsPerSecond));

    }

    public Command cmdSetIntakeVeloctiy(Supplier<Double> suppler) {
        return Commands.runOnce(
                () -> intake.setIntakeVelocity(suppler.get(), Voltage.ofBaseUnits(0, Units.Volts)),
                intake);

    }
}