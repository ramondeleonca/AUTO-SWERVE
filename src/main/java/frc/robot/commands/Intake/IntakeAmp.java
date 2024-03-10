package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRollers.IntakeRollers;

public class IntakeAmp extends Command {
  private Timer timer = new Timer();
  private IntakeRollers rollers;

  public IntakeAmp(IntakeRollers rollers) {
    this.rollers = rollers;
    addRequirements(rollers);
  }
  
  @Override
  public void initialize() {
    timer.reset();
    timer.start();

  }
  
  @Override
  public void execute() {
    rollers.setRollersOut();
  }
  
  @Override
  public void end(boolean interrupted) {
    rollers.stopRollers();
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
}
