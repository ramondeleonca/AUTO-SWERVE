package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.LimelightHelpers;

import org.littletonrobotics.junction.Logger;

public class LimelightIO implements VisionIO {
    String name;

    public LimelightIO(String name) {
        this.name = name;
    }

    public Pose2d getEstimatedPose() {
        return LimelightHelpers.getBotPose2d_wpiBlue(name);
    }

    public double getTimestampSeconds() {
        return LimelightHelpers.getLatency_Pipeline(name) + LimelightHelpers.getLatency_Capture(name);
    }

    public void updateInputs(VisionIOInputs inputs) {
        inputs.targetX = LimelightHelpers.getTX(name);
        inputs.targetY = LimelightHelpers.getTY(name);
        inputs.targetArea = LimelightHelpers.getTA(name);
        inputs.targetID = LimelightHelpers.getFiducialID(name);
    }

    public void periodic() {
        Logger.recordOutput("Vision/" + name + "/EstimatedPose", getEstimatedPose());
    }
}
