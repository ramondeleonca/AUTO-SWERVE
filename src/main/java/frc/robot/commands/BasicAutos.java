package frc.robot.commands;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.IntakeOut;
import frc.robot.commands.Shooter.BasicShoot;
import frc.robot.commands.SwerveDrive.DriveFor;
import frc.robot.commands.SwerveDrive.DriveSwerve;
import frc.robot.subsystems.IntakeRollers.IntakeRollers;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.SwerveDrive.SwerveDrive;
import lib.team3526.commands.RunForCommand;

public class BasicAutos {
    public static final Command leaveCommunity(SwerveDrive swerveDrive) {
        return new DriveFor(
            swerveDrive,
            () -> 0.2,
            () -> 0.,
            () -> 0.,
            () -> true,
            3
        );
    }

    public static final Command shoot(Shooter shooter, IntakeRollers rollers) {
        return new ParallelCommandGroup(
            new RunForCommand(new BasicShoot(shooter), 2),
            new SequentialCommandGroup(
                new WaitCommand(1),
                new RunForCommand(new IntakeOut(rollers), 0.5)
            )
        );
    }

    public static final Command shootAndLeave(SwerveDrive swerveDrive, Shooter shooter, IntakeRollers rollers) {
        return new SequentialCommandGroup(
            shoot(shooter, rollers),
            leaveCommunity(swerveDrive)
        );
    }
}
