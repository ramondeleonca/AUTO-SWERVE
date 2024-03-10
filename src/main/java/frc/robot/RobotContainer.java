package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.BasicAutos;
import frc.robot.commands.Climb;
import frc.robot.commands.Intake.IntakeIn;
import frc.robot.commands.Intake.IntakeOut;
import frc.robot.commands.Intake.LifterDown;
import frc.robot.commands.Intake.LifterUp;
import frc.robot.commands.Shooter.BasicShoot;
import frc.robot.commands.SwerveDrive.DriveFor;
import frc.robot.commands.SwerveDrive.DriveSwerve;
import frc.robot.commands.SwerveDrive.ResetPose;
import frc.robot.commands.SwerveDrive.ZeroHeading;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.ClimberIOReal;
import frc.robot.subsystems.Climber.ClimberIOSim;
import frc.robot.subsystems.Gyro.Gyro;
import frc.robot.subsystems.Gyro.GyroIOPigeon;
import frc.robot.subsystems.Gyro.GyroIOSim;
import frc.robot.subsystems.IntakeLifter.IntakeLifter;
import frc.robot.subsystems.IntakeLifter.IntakeLifterIOReal;
import frc.robot.subsystems.IntakeLifter.IntakeLifterIOSim;
import frc.robot.subsystems.IntakeRollers.IntakeRollers;
import frc.robot.subsystems.IntakeRollers.IntakeRollersIOReal;
import frc.robot.subsystems.IntakeRollers.IntakeRollersIOSim;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterIOReal;
import frc.robot.subsystems.Shooter.ShooterIOSim;
import frc.robot.subsystems.SwerveDrive.SwerveDrive;
import frc.robot.subsystems.SwerveDrive.SwerveDriveIOReal;
import frc.robot.subsystems.SwerveDrive.SwerveDriveIOSim;
import frc.robot.subsystems.SwerveModule.SwerveModule;
import frc.robot.subsystems.SwerveModule.SwerveModuleIOSim;
import lib.team3526.commands.RunForCommand;
import lib.team3526.driveControl.CustomController;
import lib.team3526.led.animations.BreatheAnimation;
import lib.team3526.led.animations.ShootingStarAnimation;
import lib.team3526.led.framework.HyperAddressableLEDSegment;
import lib.team3526.led.framework.HyperAddressableLEDStrip;
import frc.robot.subsystems.Vision.LimelightIO;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.SwerveModule.SwerveModuleIOReal;

import java.util.HashMap;

public class RobotContainer {


  private final CustomController m_driverControllerCustom;

  private final SwerveModule m_frontLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;
  
  private final SwerveDrive m_swerveDrive;
  private final Gyro m_gyro;
  private final IntakeLifter m_intake;
  private final IntakeRollers m_Rollers;
  private final Shooter m_shooter;
  private final Climber m_leftClimber;
  private final Climber m_rightClimber;
  // private final PoseEstimatorSubsystem m_poseEstimator;
  // private final LED m_led = new LED(new LEDOptions(0, 20));

  public static final HyperAddressableLEDSegment m_climberLeds = new HyperAddressableLEDSegment(10);
  public static final HyperAddressableLEDSegment m_leftShooterLeds = new HyperAddressableLEDSegment(5);
  public static final HyperAddressableLEDSegment m_rightShooterLeds = new HyperAddressableLEDSegment(5);
  public static final HyperAddressableLEDStrip m_leds = new HyperAddressableLEDStrip(0, m_climberLeds, m_leftShooterLeds, m_rightShooterLeds);

  SendableChooser<Command> autonomousChooser;
  SendableChooser<Command> basicAutonomousChooser;

  private Command autonomous;

  public RobotContainer() {
    if (Robot.isReal()) {
      // Create all real swerve modules and initialize
      this.m_frontLeft = new SwerveModule(new SwerveModuleIOReal(Constants.SwerveDrive.SwerveModules.kFrontLeftOptions));
      this.m_frontRight = new SwerveModule(new SwerveModuleIOReal(Constants.SwerveDrive.SwerveModules.kFrontRightOptions));
      this.m_backLeft = new SwerveModule(new SwerveModuleIOReal(Constants.SwerveDrive.SwerveModules.kBackLeftOptions));
      this.m_backRight = new SwerveModule(new SwerveModuleIOReal(Constants.SwerveDrive.SwerveModules.kBackRightOptions));

      // Create the real swerve drive and initialize
      this.m_gyro = new Gyro(new GyroIOPigeon(Constants.SwerveDrive.kGyroDevice));
      this.m_swerveDrive = new SwerveDrive(new SwerveDriveIOReal(m_frontLeft, m_frontRight, m_backLeft, m_backRight, m_gyro, new Vision[] 
      {
        new Vision(new LimelightIO("limelight"))
      }));
      this.m_intake =  new IntakeLifter(new IntakeLifterIOReal());
      this.m_Rollers = new IntakeRollers(new IntakeRollersIOReal());
      this.m_shooter = new Shooter(new ShooterIOReal());
      this.m_leftClimber = new Climber(new ClimberIOReal(Constants.Climber.kLeftClimberMotorID, "LeftClimber"));
      this.m_rightClimber = new Climber(new ClimberIOReal(Constants.Climber.kRightClimberMotorID, "RightClimber"));
      // this.m_poseEstimator = new PoseEstimatorSubsystem(new Vision[] {
      //   new Vision(new LimelightIO("limelight"))
      // }, m_swerveDrive, m_gyro);

      Logger.recordMetadata("Robot", "Real");
    } else {
      // Create all simulated swerve modules and initialize
      this.m_frontLeft = new SwerveModule(new SwerveModuleIOSim(Constants.SwerveDrive.SwerveModules.kFrontLeftOptions));
      this.m_frontRight = new SwerveModule(new SwerveModuleIOSim(Constants.SwerveDrive.SwerveModules.kFrontRightOptions));
      this.m_backLeft = new SwerveModule(new SwerveModuleIOSim(Constants.SwerveDrive.SwerveModules.kBackLeftOptions));
      this.m_backRight = new SwerveModule(new SwerveModuleIOSim(Constants.SwerveDrive.SwerveModules.kBackRightOptions));

      // Create the simulated swerve drive and initialize
      this.m_gyro = new Gyro(new GyroIOSim());
      this.m_swerveDrive = new SwerveDrive(new SwerveDriveIOSim(m_frontLeft, m_frontRight, m_backLeft, m_backRight));
      this.m_intake = new IntakeLifter(new IntakeLifterIOSim());
      this.m_Rollers = new IntakeRollers(new IntakeRollersIOSim());
      this.m_shooter = new Shooter(new ShooterIOSim());
      this.m_leftClimber = new Climber(new ClimberIOSim());
      this.m_rightClimber = new Climber(new ClimberIOSim());
      // this.m_poseEstimator = null;

      Logger.recordMetadata("Robot", "Sim");
    }

    // boolean isXbox = DriverStation.getJoystickIsXbox(0);
    boolean isXbox = true;
    SmartDashboard.putBoolean("Controller/IsXbox", isXbox);
    this.m_driverControllerCustom = new CustomController(0, CustomController.CustomControllerType.XBOX, CustomController.CustomJoystickCurve.LINEAR);

    // Register the named commands for autonomous
    NamedCommands.registerCommands(new HashMap<String, Command>() {{
      put("IntakeIn", new RunForCommand(new IntakeIn(m_Rollers), 1));
      put("IntakeOut", new RunForCommand(new IntakeOut(m_Rollers), 0.25));

      put("BasicShoot", new RunForCommand(new BasicShoot(m_shooter), 2));

      put("LifterUp", new RunForCommand(new LifterUp(m_intake), 1));
      put("LifterDown", new RunForCommand(new LifterDown(m_intake), 1));

      put("ClimbUp", new RunForCommand(new Climb(m_rightClimber, () -> true), 1));
      put("ClimbDown", new RunForCommand(new Climb(m_leftClimber, () -> false), 1));

      put("Wait", new WaitCommand(1));
    }});
 
    SmartDashboard.putData(new ZeroHeading(m_swerveDrive));
    SmartDashboard.putData(new ResetPose(m_swerveDrive));

    SendableChooser<Command> autonomousChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autonomous", autonomousChooser);
    this.autonomousChooser = autonomousChooser;

    SendableChooser<Command> basicAutonomousChooser = new SendableChooser<Command>();
    basicAutonomousChooser.setDefaultOption("Do Nothing", new WaitCommand(1));
    basicAutonomousChooser.addOption("Shoot", BasicAutos.shoot(m_shooter, m_Rollers));
    basicAutonomousChooser.addOption("Leave Community", BasicAutos.leaveCommunity(m_swerveDrive));
    basicAutonomousChooser.addOption("Shoot and Leave", BasicAutos.shootAndLeave(m_swerveDrive, m_shooter, m_Rollers));
    SmartDashboard.putData("Basic Autonomous", basicAutonomousChooser);
    this.basicAutonomousChooser = basicAutonomousChooser;

    m_climberLeds.setDefaultAnimation(Constants.LED.breatheAnimation::provider);
    m_leftShooterLeds.setDefaultAnimation(Constants.LED.breatheAnimation::provider);
    m_rightShooterLeds.setDefaultAnimation(Constants.LED.breatheAnimation::provider);

    configureBindings();
  }

  private void configureBindings() {
    // Set the default command for the swerve drive
    this.m_swerveDrive.setDefaultCommand(new DriveSwerve(
        m_swerveDrive,
        () -> -this.m_driverControllerCustom.getLeftY(),
        () -> -this.m_driverControllerCustom.getLeftX(),
        () -> -this.m_driverControllerCustom.getRightX(),
        () -> !this.m_driverControllerCustom.bottomButton().getAsBoolean()
      )
    );

    /* this.m_driverControllerCustom.leftTrigger().whileTrue(new DriveSwerve(
        m_swerveDrive,
        () -> 0.7,
        () -> 0.0,
        () -> 0.0,
        () -> false
    )); */

    
    this.m_driverControllerCustom.leftButton().toggleOnTrue(new IntakeIn(m_Rollers));
    this.m_driverControllerCustom.topButton().whileTrue(new IntakeOut(m_Rollers));

    this.m_driverControllerCustom.rightTrigger().whileTrue(new BasicShoot(m_shooter));
    this.m_driverControllerCustom.leftTrigger().whileTrue(new IntakeOut(m_Rollers));

    this.m_driverControllerCustom.rightBumper().whileTrue(new LifterUp(m_intake));

    this.m_driverControllerCustom.leftBumper().whileTrue(new LifterDown(m_intake));

    this.m_driverControllerCustom.povUp().whileTrue(new Climb(m_rightClimber, () -> true));
    this.m_driverControllerCustom.povUp().whileTrue(new Climb(m_leftClimber, () -> true));


    this.m_driverControllerCustom.povDown().whileTrue(new Climb(m_rightClimber, () -> false));
    this.m_driverControllerCustom.povDown().whileTrue(new Climb(m_leftClimber, () -> false));
  }

  public Command getAutonomousCommand() {
    return AutoBuilder.buildAuto("AllNotes");
  };
}
