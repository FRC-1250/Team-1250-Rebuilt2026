package frc.robot;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.FuelLine;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class CommandFactory {

    private final Swerve swerve;
    private final FuelLine intake;
    private final Shooter shooter;
    private final Climber climber;
    private final Limelight limelight;
    private final Leds leds;

    public CommandFactory(Swerve swerve, FuelLine intake, Shooter shooter, Climber climber,
            Limelight limelight, Leds leds) {
        this.swerve = swerve;
        this.intake = intake;
        this.shooter = shooter;
        this.climber = climber;
        this.limelight = limelight;
        this.leds = leds;
    }

}
