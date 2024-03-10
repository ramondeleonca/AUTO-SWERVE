// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeLifter.IntakeLifter;

public class LifterDown extends Command {
  IntakeLifter intake;

  public LifterDown(IntakeLifter intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setLifterSpeed(Constants.Intake.kLifterDownSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setLifterSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.toDegrees(intake.getLifterAngleRadians())<0) {
      return true;
    } else {
      return false;
    }
  }
}
