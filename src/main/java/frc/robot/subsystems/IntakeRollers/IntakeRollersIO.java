package frc.robot.subsystems.IntakeRollers;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public interface IntakeRollersIO {
    @AutoLog
    class IntakeRollersIOInputs {

    }

    public void setRollersOut();
    public void setRollersIn();
    public void giveToShooter();

    public void setRollersSpeed(double speed);
    public void setRollersSpeedRpm(Measure<Velocity<Angle>> rpm);

    public void setRollersHold();
    public void stopRollers();

    public void setRollersCoast();
    public void setRollersBrake();

    public double getRollersSpeed();

    public boolean hasPiece();

    public void updateInputs(IntakeRollersIOInputs inputs);
    default public void periodic() {};
}
