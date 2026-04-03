// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.LimelightLocalizationMode;
import frc.robot.utility.LimelightHelpers.PoseEstimate;
import frc.robot.utility.LimelightHelpers.RawFiducial;

public class RobotLocalization {
    private final List<Limelight> limelights;
    private final CommandSwerveDrivetrain swerveDrivetrain;

    private double tagAmbiguous = 0;
    private double tagTooSmall = 0;
    private double resultOutOfBounds = 0;
    private double resultTeleported = 0;
    private double framesProcessed = 0;
    private double frameRejectionRate = 0;
    private int teleportFrameCounter = 0;
    private double xTrust, yTrust = 10.0;

    private final double maxRadiansPerSecond = 2;
    private final double minAllowedViewableTagDecimal = 0.05; // 0 to 100 so 0.05 is 5%
    private final double maxTeleportDistance = 4.5;
    private final double maxAllowedTagAmbiguity = 0.6;
    private final int maxFramesBeforeTeleport = 10;
    private final double fieldBuffer = Units.feetToMeters(0);
    private final double fieldLength = Units.feetToMeters(52);
    private final double fieldWidth = Units.feetToMeters(27);

    private final Timer reportTimer = new Timer();
    private List<FieldLocalization.Zones> activeZones = new ArrayList<>();

    /** Creates a new SwerveVisionLogic. */
    public RobotLocalization(List<Limelight> limelights, CommandSwerveDrivetrain swerveDrivetrain) {
        this.limelights = limelights;
        this.swerveDrivetrain = swerveDrivetrain;
        reportTimer.start();
    }

    public void processMegaTag2Measurement() {
        SwerveDriveState driveState = swerveDrivetrain.getState();

        for (Limelight limelight : limelights) {
            limelight.setRobotOrientation(driveState.Pose.getRotation().getDegrees());
            PoseEstimate megaTag = limelight.getBotPoseEstimate_wpiBlue_MegaTag2();
            framesProcessed++;

            if (reportTimer.advanceIfElapsed(5)) {
                reportToNT();
                reportTimer.reset();
            }

            if (Math.abs(Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond)) > maxRadiansPerSecond)
                continue;

            if (megaTag == null)
                continue;

            if (megaTag.tagCount == 0)
                continue;

            if (areAnyTagsAmbiguous(megaTag.rawFiducials)) {
                tagAmbiguous++;
                continue;
            }

            if (isTagTooSmall(megaTag.rawFiducials)) {
                tagTooSmall++;
                continue;
            }

            if (isEstimateOutOfBounds(megaTag.pose)) {
                resultOutOfBounds++;
                continue;
            }

            xTrust = yTrust = calculateTrust(megaTag);
            limelight.getPosePublisher().set(megaTag.pose);

            if (limelight.getMode() == LimelightLocalizationMode.ENABLED) {

                if (hasTeleported(megaTag.pose, driveState.Pose, maxTeleportDistance)) {
                    resultTeleported++;
                    continue;
                }

                swerveDrivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(xTrust, yTrust, 9999999));
                swerveDrivetrain.addVisionMeasurement(megaTag.pose, megaTag.timestampSeconds);
            }
        }

    }

    private void reportToNT() {
        SmartDashboard.putNumber("Ambiguious Tag", tagAmbiguous);
        SmartDashboard.putNumber("Too Small", tagTooSmall);
        SmartDashboard.putNumber("Went Out of Bounds", resultOutOfBounds);
        SmartDashboard.putNumber("Teleported", resultTeleported);

        frameRejectionRate = (tagAmbiguous + tagTooSmall + resultOutOfBounds + resultTeleported)
                / framesProcessed;
        SmartDashboard.putNumber("Rejection rate", frameRejectionRate);
    }

    private double calculateTrust(PoseEstimate estimate) {
        double trust = 0.1 + (0.2 * estimate.avgTagDist);

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
                swerveDrivetrain.resetPose(visionPose);
                return false;
            }
            return true;
        }
        teleportFrameCounter = 0;
        return false;
    }

    public void processActiveZone() {
        Pose2d robotPose = swerveDrivetrain.getState().Pose;

        activeZones = Arrays.stream(FieldLocalization.Zones.values())
                .filter(z -> z.area.isRobotInZone(robotPose))
                .toList();

        SmartDashboard.putString("Zones", activeZones.toString());
    }

    public List<FieldLocalization.Zones> getActiveZones() {
        return activeZones;
    }
}
