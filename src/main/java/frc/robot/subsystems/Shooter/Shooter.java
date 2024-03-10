package frc.robot.subsystems.Shooter;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  ShooterIO io;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  public void setLeftMotor(double speed) {
    io.setLeftMotor(speed);
  }

  public void setRightMotor(double speed) {
    io.setRightMotor(speed);
  }

  public void set(double leftSpeed, double rightSpeed) {
    io.set(leftSpeed, rightSpeed);
  }

  public void set(double speed) {
    io.set(speed);
  }

  public void setLeftMotorRpm(Measure<Velocity<Angle>> rpm) {
    io.setLeftMotorRpm(rpm);
  }

  public void setRightMotorRpm(Measure<Velocity<Angle>> rpm) {
    io.setRightMotorRpm(rpm);
  }

  public void setRpm(Measure<Velocity<Angle>> leftRpm, Measure<Velocity<Angle>> rightRpm) {
    io.setRpm(leftRpm, rightRpm);
  }

  public void setRpm(Measure<Velocity<Angle>> rpm) {
    io.setRpm(rpm);
  }

  public void shootSpeaker() {
    io.shootSpeaker();
  }

  public double getLeftMotorRpm() {
    return io.getLeftMotorRpm();
  }

  public double getRightMotorRpm() {
    return io.getRightMotorRpm();
  }

  public double getLeftMotorPercentage() {
    return io.getLeftMotorPercentage();
  }

  public double getRightMotorPercentage() {
    return io.getRightMotorPercentage();
  }

  public void stop() {
    io.stop();
  }

  @Override
  public void periodic() {
    io.periodic();
    io.updateInputs(inputs);
  }
}
