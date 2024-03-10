package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public interface ShooterIO {
    @AutoLog
    class ShooterIOInputs {
        double leftRpm;
        double leftPercentage;
        double leftTargetRpm;
        
        double rightRpm;
        double rightPercentage;
        double rightTargetRpm;
    }

    public void setLeftMotor(double speed);
    public void setRightMotor(double speed);
    public void set(double leftSpeed, double rightSpeed);
    public void set(double speed);

    public void shootSpeaker();

    public void setLeftMotorRpm(Measure<Velocity<Angle>> rpm);
    public void setRightMotorRpm(Measure<Velocity<Angle>> rpm);
    public void setRpm(Measure<Velocity<Angle>> leftRpm, Measure<Velocity<Angle>> rightRpm);
    public void setRpm(Measure<Velocity<Angle>> rpm);

    public void stop();

    public double getLeftMotorRpm();
    public double getRightMotorRpm();

    public double getLeftMotorPercentage();
    public double getRightMotorPercentage();

    public void updateInputs(ShooterIOInputs inputs);
    public default void periodic() {};
}
