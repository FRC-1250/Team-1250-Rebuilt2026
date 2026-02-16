// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.telemetry.HealthMonitor;
import frc.robot.utility.LimelightHelpers;

@Logged
public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private final RobotContainer robotContainer;

    private final Timer gcTimer;
    private final Command visionCommand;

    public Robot() {
        robotContainer = new RobotContainer();
        visionCommand = Commands.sequence(
                Commands.waitSeconds(1),
                robotContainer.commandFactory.cmdSwerveVisionLogic());
        ;
        DriverStation.startDataLog(DataLogManager.getLog());
        Epilogue.bind(this);
        DriverStation.silenceJoystickConnectionWarning(true);

        CommandScheduler.getInstance().onCommandInitialize(
                command -> DataLogManager.log(
                        String.format("Command init: %s, with requirements: %s", command.getName(),
                                command.getRequirements())));

        CommandScheduler.getInstance().onCommandFinish(
                command -> DataLogManager.log(String.format("Command finished: %s", command.getName())));

        CommandScheduler.getInstance().onCommandInterrupt(
                command -> DataLogManager.log(String.format("Command interrupted: %s", command.getName())));

        gcTimer = new Timer();
        gcTimer.start();
    }

    @Override
    public void robotInit() {
        HealthMonitor.getInstance().start();
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
        Pathfinding.setPathfinder(new LocalADStar());
        LimelightHelpers.setupPortForwardingUSB(0);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("Match time", DriverStation.getMatchTime());
    }

    @Override
    public void disabledInit() {
        HealthMonitor.getInstance().unpause();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
        HealthMonitor.getInstance().pause();
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }

        if (!visionCommand.isScheduled()) {
            CommandScheduler.getInstance().schedule(visionCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

        if (!visionCommand.isScheduled()) {
            CommandScheduler.getInstance().schedule(visionCommand);
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
