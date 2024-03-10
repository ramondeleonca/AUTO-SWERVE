package frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {
    @AutoLog
    class VisionIOInputs {
        public double targetX;
        public double targetY;
        public double targetArea;
        public double targetID;
    }

    Pose2d getEstimatedPose();
    double getTimestampSeconds();

    void updateInputs(VisionIOInputs inputs);

    public default void periodic() { }
}
