package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Centimeters;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

public class ClimberIOSim implements ClimberIO {
    public Measure<Distance> getExtension() { return Centimeters.of(0); }
    public double getCurrent() { return 0; }
    public void resetEncoder() {}

    public void set(double speed) {};

    public void setClimberUp() {}
    public void setClimberDown() {}

    public void setClimberHold() {}

    public void stop() {}

    public void updateInputs(ClimberIOInputs inputs) {}
    public void periodic() {}
}
