package frc.robot;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class CommandFactory {

    private final Swerve swerve;
    private final Intake intake;
    private final Hopper hopper;
    private final Shooter shooter;
    private final Climber climber;
    private final Limelight limelight;
    private final Leds leds;

    public CommandFactory(Swerve swerve, Intake intake, Hopper hopper, Shooter shooter, Climber climber,
            Limelight limelight, Leds leds) {
        this.swerve = swerve;
        this.intake = intake;
        this.hopper = hopper;
        this.shooter = shooter;
        this.climber = climber;
        this.limelight = limelight;
        this.leds = leds;
    }

}
