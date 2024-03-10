package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionIOSim implements VisionIO {
    public Pose2d getEstimatedPose() {return new Pose2d();}
    public double getTimestampSeconds() {return 0;}

    public void updateInputs(VisionIOInputs inputs) {};

    public void periodic() { }
}
