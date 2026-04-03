// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.LimelightHelpers;
import frc.robot.utility.LimelightHelpers.PoseEstimate;

public class Limelight extends SubsystemBase {

    public enum LimeLightPipeline {
        DEFAULT(0),
        BRIGHTER(1),
        DARKER(2);

        public final int pipelineId;

        LimeLightPipeline(int pipelineId) {
            this.pipelineId = pipelineId;
        }
    }

    public enum LimelightLocalizationMode {
        ENABLED,
        DISABED
    }

    private final String name;
    private final LimelightLocalizationMode mode;
    private final StructPublisher<Pose2d> llPosePublisher;
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable llTable = inst.getTable("Robot/RobotContainer/Limelight");

    public Limelight(String name, LimelightLocalizationMode mode) {
        this.name = name;
        this.mode = mode;
        this.llPosePublisher = llTable.getStructTopic(name + " pose2d", Pose2d.struct).publish();
    }

    public Limelight() {
        this("limelight", LimelightLocalizationMode.ENABLED);
    }

    public double getFid() {
        return LimelightHelpers.getFiducialID(name);
    }

    public void setRobotOrientation(double headingDeg) {
        LimelightHelpers.SetRobotOrientation(name, headingDeg, 0, 0, 0, 0, 0);
    }

    public PoseEstimate getBotPoseEstimate_wpiBlue_MegaTag1() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    }

    public PoseEstimate getBotPoseEstimate_wpiBlue_MegaTag2() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    }

    public double getDistanceFromTarget() {
        return LimelightHelpers.getTargetPose3d_RobotSpace(name).getMeasureZ().magnitude();
    }

    public Command switchPipeline(LimeLightPipeline limeLightPipeline) {
        return Commands.runOnce(() -> LimelightHelpers.setPipelineIndex(name, limeLightPipeline.pipelineId))
                .withName(String.format("%s set pipeline %s", name, limeLightPipeline.toString()))
                .ignoringDisable(true);
    }

    public LimelightLocalizationMode getMode() {
        return mode;
    }

    public StructPublisher<Pose2d> getPosePublisher() {
        return llPosePublisher;
    }

}
