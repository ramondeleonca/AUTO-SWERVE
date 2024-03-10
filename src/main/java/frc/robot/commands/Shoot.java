// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeLifter.IntakeLifter;
import frc.robot.subsystems.IntakeRollers.IntakeRollers;
import frc.robot.subsystems.Shooter.Shooter;

public class Shoot extends Command {
  Shooter shooter;
  IntakeRollers rollers;
  private Timer timer = new Timer();

  /** Creates a new Shoot. */
  public Shoot(Shooter shooter, IntakeRollers intake) {
    this.shooter = shooter;
    this.rollers = intake;
    addRequirements(shooter, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.shooter.set(-1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      while (timer.get() < 1) {
        this.rollers.stopRollers();
        this.shooter.set(-0.7);
      }
      
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > Constants.Shooter.kMaxShootTime;
  }
}
