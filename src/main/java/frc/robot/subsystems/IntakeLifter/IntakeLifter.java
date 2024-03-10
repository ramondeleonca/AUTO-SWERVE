package frc.robot.subsystems.IntakeLifter;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeLifter extends SubsystemBase {
  private final IntakeLifterIO io;
  private final IntakeLifterIOInputsAutoLogged inputs = new IntakeLifterIOInputsAutoLogged();

  public IntakeLifter(IntakeLifterIO io) {
    this.io = io;
  }

  /**
   * Sets the angle of the lifter
   * @param angleDeg the angle to set the lifter to
   */
  public void setLifterAngle(Measure<Angle> angleDeg) {
    io.setLifterAngle(angleDeg);
  }

  /**
   * Gets the angle of the lifter
   * @return the angle of the lifter
   */
  public double getLifterAngleRadians() {
    return io.getLifterAngleRadians();
  }

  /**
   * Stops the lifter
   */
  public void stopLifter() {
    io.stopLifter();
  }

  /**
   * Set the speed of the intake motor
   */
  public void setLifterSpeed(double speed) {
    io.setLifterSpeed(speed);
  }

  @Override
  public void periodic() {
    io.periodic();
    io.updateInputs(inputs);
  }
}
