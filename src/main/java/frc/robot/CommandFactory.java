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
    public Command cmdSetIntakeVeloctiy(double rotationsPerSecond) {
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

    public Command cmdSetLoaderCamPosition(double rotations) {
        return cmdSetLoaderCamPosition(() -> rotations);
    }

    public Command cmdSetLoaderCamPosition(Supplier<Double> supplier) {
        return Commands.runOnce(
                () -> fuelLine.setLoaderCamPosition(supplier.get()), fuelLine)
                .andThen(Commands.waitUntil(() -> fuelLine.isLoaderCamNearPosition(supplier.get(), 1)));
    }

    public Command cmdSetLoaderCamVelocity(double rotationsPerSecond) {
        return cmdSetLoaderCamVelocity(() -> rotationsPerSecond);
    }

    public Command cmdSetLoaderCamVelocity(Supplier<Double> supplier) {
        return Commands.runOnce(
                () -> fuelLine.setLoaderCamVelocity(supplier.get()), fuelLine);
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
                () -> shooter.setAcceleratorVelocity(supplier.get()));

    }

}