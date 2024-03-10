package frc.robot.subsystems.Climber;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  ClimberIO io;
  ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public Climber(ClimberIO io) {
    this.io = io;
  }

  /**
   * Set the speed of the climber motor
   * @param speed
   */
  public void set(double speed) {
    io.set(speed);
  }

  /**
   * Set the climber motor to go up
   */
  public void setClimberUp() {
    io.setClimberUp();
  }

  /**
   * Set the climber motor to go down
   */
  public void setClimberDown() {
    io.setClimberDown();
  }

  /**
   * Stop the climber motor
   */
  public void stop() {
    io.stop();
  }

  /**
   * Get the extension of the climber
   * @return
   */
  public Measure<Distance> getExtension() {
    return io.getExtension();
  }

  /**
   * Get the current of the climber motor
   * @return
   */
  public double getCurrent() {
    return io.getCurrent();
  }

  /**
   * Reset the encoder of the climber motor
   * @return
   */
  public void resetEncoder() {
    io.resetEncoder();
  }

  /**
   * Set the climber motor to hold in the climb position
   */
  public void setClimberHold() {
    io.setClimberHold();
  }

  @Override
  public void periodic() {
    io.periodic();
    io.updateInputs(inputs);
  }
}
