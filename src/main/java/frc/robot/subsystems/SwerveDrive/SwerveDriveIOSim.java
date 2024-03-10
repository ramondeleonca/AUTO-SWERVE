package frc.robot.subsystems.SwerveDrive;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveModule.SwerveModule;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class SwerveDriveIOSim implements SwerveDriveIO {
    // Create all swerve modules
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    // Create a swerve drive odometry instance to calculate robot position
    private final SwerveDriveOdometry odometry;

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    private double heading = 0;
    private double speedsUpdated = Timer.getFPGATimestamp();
    private boolean drivingRobotRelative = false;

    public SwerveDriveIOSim(SwerveModule m_frontLeft, SwerveModule m_frontRight, SwerveModule m_backLeft, SwerveModule m_backRight) {
        this.frontLeft = m_frontLeft;
        this.frontRight = m_frontRight;
        this.backLeft = m_backLeft;
        this.backRight = m_backRight;

        this.odometry = new SwerveDriveOdometry(Constants.SwerveDrive.PhysicalModel.kDriveKinematics, getHeading(), new SwerveModulePosition[]{
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        });
    }

    public void zeroHeading() {
        this.heading = 0;
    }

    /**
     * Get the current ROBOT RELATIVE ChassisSpeeds
     * @return ChassisSpeeds
     */
    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        if (this.drivingRobotRelative) {
            return this.chassisSpeeds;
        } else {
            return ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getHeading());
        }
    }

    /**
     * Configure the auto builder
     * @param swerveDrive The swerve drive subsystem
     */
    public void configureAutoBuilder(SwerveDrive swerveDrive) {
        AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetPosition,
            this::getRobotRelativeChassisSpeeds,
            this::driveRobotRelative,
            new HolonomicPathFollowerConfig(
                Constants.SwerveDrive.Autonomous.kTranslatePIDConstants,
                Constants.SwerveDrive.Autonomous.kRotatePIDConstants,
                Constants.SwerveDrive.Autonomous.kMaxSpeedMetersPerSecond.in(MetersPerSecond),
                Constants.SwerveDrive.PhysicalModel.kWheelBase.in(Meters) / 2,
                new ReplanningConfig()
            ),
            () -> {
                if (DriverStation.getAlliance().isPresent()) return DriverStation.getAlliance().get() == Alliance.Red;
                return false;
            },
            swerveDrive
        );
    }



    /**
     * Tries to update the odometry (if it fails, it will print the error and still return the odometry)
     * @return SwerveDriveOdometry
     */
    public SwerveDriveOdometry getOdometry() {
        try {
            odometry.update(
                getHeading(),
                new SwerveModulePosition[]{
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    backLeft.getPosition(),
                    backRight.getPosition()
                }
            );
        } catch (Exception e) {
            System.out.println("Error updating odometry: " + e);
        }
        return odometry;
    }

    /**
     * Get the current pose of the robot
     * @return Pose2d
     */
    public Pose2d getPose() {
        return getOdometry().getPoseMeters();
    }

    /**
     * Reset the pose of the robot
     * @param Pose2d
     */
    public void resetPosition(Pose2d pose) {
        odometry.resetPosition(this.getHeading(), new SwerveModulePosition[]{
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        }, pose);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveDrive.PhysicalModel.kMaxSpeed);
        frontLeft.setState(desiredStates[0]);
        frontRight.setState(desiredStates[1]);
        backLeft.setState(desiredStates[2]);
        backRight.setState(desiredStates[3]);
    }

    public SwerveModuleState[] getRealModuleStates() {
        return new SwerveModuleState[]{
            frontLeft.getRealState(),
            frontRight.getRealState(),
            backLeft.getRealState(),
            backRight.getRealState()
        };
    }

    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void updateInputs(SwerveDriveIOInputs inputs) {
        inputs.heading = getHeading().getDegrees();

        inputs.xSpeed = chassisSpeeds.vxMetersPerSecond;
        inputs.ySpeed = chassisSpeeds.vyMetersPerSecond;
        inputs.rotSpeed = chassisSpeeds.omegaRadiansPerSecond;

        this.heading += chassisSpeeds.omegaRadiansPerSecond * (Timer.getFPGATimestamp() - this.speedsUpdated);
    }

    public Rotation2d getHeading() {
        // Calculate the new robot heading angle using the angle theta provided 
        return new Rotation2d(this.heading);
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
    }

    /**
     * Drive the robot using the given speeds (calculate the states for each swerve module and apply it)
     * @param speeds The speeds to drive at (Check `ChassisSpeeds` for more info)
     */
    public void drive(ChassisSpeeds speeds) {
        this.speedsUpdated = Timer.getFPGATimestamp();
        this.chassisSpeeds = speeds;
        SwerveModuleState[] m_moduleStates = Constants.SwerveDrive.PhysicalModel.kDriveKinematics.toSwerveModuleStates(speeds);
        this.setModuleStates(m_moduleStates);
    }

    /**
     * Drive the robot relative to the field
     * @param xSpeed X speed in meters per second
     * @param ySpeed Y speed in meters per second
     * @param rotSpeed Rotation speed in radians per second
     */
    public void driveFieldRelative(double xSpeed, double ySpeed, double rotSpeed) {
        this.drivingRobotRelative = false;
        this.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, this.getHeading()));
    }

    /**
     * Drive the robot relative to the field
     * @param speeds The speeds to drive at (Check `ChassisSpeeds` for more info)
     */
    public void driveFieldRelative(ChassisSpeeds speeds) {
        this.drivingRobotRelative = false;
        this.drive(speeds);
    }

    /**
     * Drive the robot relative to the robot's current position
     * @param xSpeed X speed in meters per second
     * @param ySpeed Y speed in meters per second
     * @param rotSpeed Rotation speed in radians per second
     */
    public void driveRobotRelative(double xSpeed, double ySpeed, double rotSpeed) {
        this.drivingRobotRelative = true;
        this.drive(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));
    }

    /**
     * Drive the robot relative to the field
     * @param speeds The speeds to drive at (Check `ChassisSpeeds` for more info)
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        this.drivingRobotRelative = true;
        this.drive(speeds);
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    public void xFormation() {
        this.frontLeft.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), true);
        this.frontRight.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
        this.backLeft.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
        this.backRight.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), true);
    }

    public void periodic() {
        Logger.recordOutput("SwerveDrive/RobotHeadingRad", this.getHeading().getRadians());
        Logger.recordOutput("SwerveDrive/RobotHeadingDeg", this.getHeading().getDegrees());
        
        Logger.recordOutput("SwerveDrive/RobotPose", this.getPose());

        Logger.recordOutput("SwerveDrive/RobotRelative", this.drivingRobotRelative);
        Logger.recordOutput("SwerveDrive/RobotSpeeds", this.getRobotRelativeChassisSpeeds());
        
        Logger.recordOutput("SwerveDrive/SwerveModuleStates", this.getModuleStates());
    }

    @Override
    public void resetPose() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetPose'");
    }
}
