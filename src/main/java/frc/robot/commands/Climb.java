// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Centimeters;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber.Climber;

public class Climb extends Command {
  Climber climber;
  Supplier<Boolean> upSupplier;
  boolean up;

  public Climb(Climber climber, Supplier<Boolean> up) {
    this.climber = climber;
    this.upSupplier = up;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.up = upSupplier.get();
    if (this.up) climber.setClimberDown();
    else climber.setClimberUp();
    
    double extension = climber.getExtension().in(Centimeters);
    if (extension < Constants.Climber.kMinExtension) RobotContainer.m_climberLeds.resumeDefaultAnimation();
    else {
      Constants.LED.climbingAnimation.setProgress(extension / Constants.Climber.kMaxExtension);
      RobotContainer.m_climberLeds.setAnimation(Constants.LED.climbingAnimation::provider);
      RobotContainer.m_climberLeds.resumeAnimation();
    };
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
