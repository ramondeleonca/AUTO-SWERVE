package frc.robot.subsystems.IntakeLifter;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;

public class IntakeLifterIOSim implements IntakeLifterIO {
    public void setLifterSpeed(double speed) {}
    public void setLifterAngle(Measure<Angle> angleDeg) {}
    public double getLifterAngleRadians() { return 0; }
    public void stopLifter() {}

    public void updateInputs(IntakeLifterIOInputs inputs) {}
    public void periodic() {}
}
