// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandFactory;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.utility.LimelightHelpers.PoseEstimate;
import frc.robot.utility.LimelightHelpers.RawFiducial;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveVisionLogic extends Command {
    public Limelight cmdLimelight;
    public CommandSwerveDrivetrain cmdSwerveDriveState;
    private double tagAmbiguous = 0;
    private double tagTooSmall = 0;
    private double resultOutOfBounds = 0;
    private double resultTeleported = 0;
    private double framesProcessed = 0;
    private double frameRejectionRate = 0;
    private double xTrust, yTrust = 10.0;

    private double maxRadiansPerSecond = 2;
    private double minAllowedViewableTagDecimal = 0.05; // 0 to 100 so 0.05 is 5%
    private double maxTeleportDistance = 4.5;
    private double maxAllowedTagAmbiguity = 0.6;
    private int maxFramesBeforeTeleport = 10;
    private int teleportFrameCounter = 0;
    private double fieldBuffer = Units.feetToMeters(0);
    private double fieldLength = Units.feetToMeters(52);
    private double fieldWidth = Units.feetToMeters(27);

    /** Creates a new SwerveVisionLogic. */
    public SwerveVisionLogic(Limelight Limelight, CommandSwerveDrivetrain SwerveDriveState) {
        // Use addRequirements() here to declare subsystem dependencies.
        cmdLimelight = Limelight;
        cmdSwerveDriveState = SwerveDriveState;
        addRequirements(Limelight);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        SwerveDriveState driveState = cmdSwerveDriveState.getState();
        cmdLimelight.setRobotOrientation(driveState.Pose.getRotation().getDegrees());
        PoseEstimate megaTag = cmdLimelight.getBotPoseEstimate_wpiBlue_MegaTag2();
        framesProcessed++;

        if (Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond) > maxRadiansPerSecond)
            return;

        if (megaTag == null | megaTag.tagCount == 0)
            return;

        if (areAnyTagsAmbiguous(megaTag.rawFiducials)) {
            tagAmbiguous++;
            SmartDashboard.putNumber("Ambiguious Tag", tagAmbiguous);
            return;
        }

        if (isTagTooSmall(megaTag.rawFiducials)) {
            tagTooSmall++;
            SmartDashboard.putNumber("Too Small", tagTooSmall);
            return;
        }

        if (isEstimateOutOfBounds(megaTag.pose)) {
            resultOutOfBounds++;
            SmartDashboard.putNumber("Went Out of Bounds", resultOutOfBounds);
            return;
        }

        if (hasTeleported(megaTag.pose, driveState.Pose, maxTeleportDistance)) {
            resultTeleported++;
            SmartDashboard.putNumber("Teleported", resultTeleported);
            return;

        }

        xTrust = yTrust = calculateTrust(megaTag);

        cmdSwerveDriveState.setVisionMeasurementStdDevs(VecBuilder.fill(xTrust, yTrust, 9999999));
        cmdSwerveDriveState.addVisionMeasurement(megaTag.pose, Utils.fpgaToCurrentTime(megaTag.timestampSeconds));
        frameRejectionRate = (tagAmbiguous + tagTooSmall + resultOutOfBounds + resultTeleported)
                / framesProcessed;
        SmartDashboard.putNumber("Rejection rate", frameRejectionRate);

    }

    private double calculateTrust(PoseEstimate estimate) {
        double averageDistance = 0;
        for (RawFiducial tag : estimate.rawFiducials) {
            averageDistance += tag.distToRobot;
        }
        averageDistance /= estimate.tagCount;

        double trust = 0.1 + (0.2 * averageDistance);

        if (estimate.tagCount > 1) {
            return trust * 0.5;
        }

        return trust;
    }

    private boolean areAnyTagsAmbiguous(RawFiducial[] tags) {
        for (RawFiducial tag : tags) {
            if (tag.ambiguity > maxAllowedTagAmbiguity) {
                return true;
            }
        }
        return false;
    }

    private boolean isTagTooSmall(RawFiducial[] tags) {
        for (RawFiducial tag : tags) {
            if (tag.ta * 100 < minAllowedViewableTagDecimal) {
                SmartDashboard.putNumber("ta", tag.ta * 100);
                return true;
            }
        }
        return false;
    }

    private boolean isEstimateOutOfBounds(Pose2d pose) {
        if (pose.getX() < -fieldBuffer || pose.getX() > fieldLength + fieldBuffer) {
            return true;
        }
        if (pose.getY() < -fieldBuffer || pose.getY() > fieldWidth + fieldBuffer) {
            return true;
        }
        return false;
    }

    private boolean hasTeleported(Pose2d visionPose, Pose2d currentPose, double teleportThreshold) {
        double distance = currentPose.getTranslation().getDistance(visionPose.getTranslation());

        if (distance > teleportThreshold) {
            teleportFrameCounter++;
            // If we see the same "wrong" position for X frames,
            // it's probably real just let it jump.
            if (teleportFrameCounter > maxFramesBeforeTeleport) {
                teleportFrameCounter = 0;
                cmdSwerveDriveState.resetPose(visionPose);
                return false;
            }
            return true;
        }
        teleportFrameCounter = 0;
        return false;
    }
    /*
     * Limelight
     */

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
