package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRollers.IntakeRollers;

public class IntakeIn extends Command {
  private IntakeRollers rollers;

  public IntakeIn(IntakeRollers rollers) {
    this.rollers = rollers;
    addRequirements(rollers);
  }
  
  @Override
  public void initialize() {}
  
  @Override
  public void execute() {
    rollers.setRollersIn();
  }
  
  @Override
  public void end(boolean interrupted) {
    rollers.stopRollers();
  }
  
  @Override
  public boolean isFinished() {
    return rollers.hasPiece();
  }
}
