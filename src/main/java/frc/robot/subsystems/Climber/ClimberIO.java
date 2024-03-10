package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

public interface ClimberIO {
    @AutoLog
    class ClimberIOInputs {
        double speed;
        double current;
    }

    Measure<Distance> getExtension();
    double getCurrent();
    void resetEncoder();

    void set(double speed);

    void setClimberUp();
    void setClimberDown();

    void setClimberHold();

    void stop();

    void updateInputs(ClimberIOInputs inputs);
    public default void periodic() {};
}
