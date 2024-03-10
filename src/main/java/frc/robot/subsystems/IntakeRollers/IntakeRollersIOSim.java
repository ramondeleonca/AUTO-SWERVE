package frc.robot.subsystems.IntakeRollers;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public class IntakeRollersIOSim implements IntakeRollersIO {
    public void setRollersOut() {}
    public void setRollersIn() {}
    public void giveToShooter() {}

    public void setRollersSpeed(double speed) {}
    public void setRollersSpeedRpm(Measure<Velocity<Angle>> rpm) {}

    public void setRollersHold() {}
    public void stopRollers() {}

    public void setRollersCoast() {}
    public void setRollersBrake() {}

    public double getRollersSpeed() { return 0; }

    public boolean hasPiece() { return false; }

    public void updateInputs(IntakeRollersIOInputs inputs) {}
    public void periodic() {}
}
