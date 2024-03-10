// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakeRollers;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollers extends SubsystemBase {
  public final IntakeRollersIO io;
  public final IntakeRollersIOInputsAutoLogged inputs = new IntakeRollersIOInputsAutoLogged();

  public IntakeRollers(IntakeRollersIO io) {
    this.io = io;
  }

  public void setRollersOut() {
    io.setRollersOut();
  }

  public void setRollersIn() {
    io.setRollersIn();
  }

  public void giveToShooter() {
    io.giveToShooter();
  }

  public void setRollersSpeed(double speed) {
    io.setRollersSpeed(speed);
  }

  public void setRollersSpeedRpm(Measure<Velocity<Angle>> rpm) {
    io.setRollersSpeedRpm(rpm);
  }

  public void setRollersHold() {
    io.setRollersHold();
  }

  public void stopRollers() {
    io.stopRollers();
  }

  public void setRollersCoast() {
    io.setRollersCoast();
  }

  public void setRollersBrake() {
    io.setRollersBrake();
  }

  public double getRollersSpeed() {
    return io.getRollersSpeed();
  }

  public boolean hasPiece() {
    return io.hasPiece();
  }

  @Override
  public void periodic() {
    io.periodic();
    io.updateInputs(inputs);
  }
}
