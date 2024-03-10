package frc.robot.commands.SwerveDrive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive.SwerveDrive;

public class ZeroHeading extends Command {
  SwerveDrive swerveDrive;

  public ZeroHeading(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    swerveDrive.zeroHeading();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
