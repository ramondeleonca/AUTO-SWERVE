package frc.robot.subsystems.SwerveDrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.Gyro.Gyro;
import frc.robot.subsystems.SwerveModule.SwerveModule;
import frc.robot.subsystems.Vision.Vision;
import lib.team3526.math.RotationalInertiaAccumulator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.littletonrobotics.junction.Logger;

public class SwerveDriveIOReal implements SwerveDriveIO {
    SwerveModule frontLeft;
    SwerveModule frontRight;
    SwerveModule backLeft;
    SwerveModule backRight;

    Gyro gyro;

    SwerveDrivePoseEstimator poseEstimator;
    Vision[] cams;
    SwerveDriveOdometry odometry;

    boolean drivingRobotRelative = false;
    ChassisSpeeds speeds = new ChassisSpeeds();
    double headingTarget = 0;

    // PIDController headingController = Constants.SwerveDrive.PhysicalModel.kHeadingControllerPIDConstants.toPIDController();

    RotationalInertiaAccumulator rotationalInertiaAccumulator = new RotationalInertiaAccumulator(Constants.SwerveDrive.PhysicalModel.kRobotMassKg);

    public SwerveDriveIOReal(SwerveModule frontLeft, SwerveModule frontRight, SwerveModule backLeft, SwerveModule backRight, Gyro gyro, Vision[] cams) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.gyro = gyro;
        this.cams = cams;

        poseEstimator = new SwerveDrivePoseEstimator(Constants.SwerveDrive.PhysicalModel.kDriveKinematics, gyro.getRotation2d(), getModulePositions(), Constants.Field.kInitialPoseMeters);

        this.odometry = new SwerveDriveOdometry(
            Constants.SwerveDrive.PhysicalModel.kDriveKinematics,
            this.getHeading(),
            new SwerveModulePosition[]{
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            }
        );

        this.gyro.reset();
    }

    public void configureAutoBuilder(SwerveDrive swerveDrive) {
        AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetOdometry,
            this::getRobotRelativeChassisSpeeds,
            this::driveRobotRelative,
            new HolonomicPathFollowerConfig(
                Constants.SwerveDrive.Autonomous.kTranslatePIDConstants,
                Constants.SwerveDrive.Autonomous.kRotatePIDConstants,
                Constants.SwerveDrive.Autonomous.kMaxSpeedMetersPerSecond.in(MetersPerSecond),
                Constants.SwerveDrive.PhysicalModel.kWheelBase.in(Meters) / 2,
                new ReplanningConfig(true, true)
            ),
            () -> {
                if (DriverStation.getAlliance().isPresent()) return DriverStation.getAlliance().get() == Alliance.Red;
                return false;
            },
            swerveDrive
        );
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void zeroHeading() {
        this.gyro.reset();
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
        // return getEstimatedPose();
    }

    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        if (this.drivingRobotRelative) {
            return this.speeds;
        } else {
            return ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeading());
        }
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }


    //* https://github.com/STMARobotics/frc-7028-2023/blob/5916bb426b97f10e17d9dfd5ec6c3b6fda49a7ce/src/main/java/frc/robot/subsystems/PoseEstimatorSubsystem.java
    /**
     * Resets the current pose to the specified pose. This should ONLY be called
     * when the robot's position on the field is known, like at the beginning of
     * a match.
     * @param newPose new pose
     */
    public void resetVisionOdometry(Pose2d newPose) {
        poseEstimator.resetPosition(
        gyro.getRotation2d(),
        getModulePositions(),
        newPose);
    }

    public void resetPose(){
        this.odometry.update(getHeading(), getModulePositions());
        // resetVisionOdometry(Constants.Field.kInitialPoseMeters);
    }

    public SwerveDriveOdometry getOdometry() {
        // try {
        //     odometry.update(
        //         Rotation2d.fromRadians(getHeading().getRadians() + Math.PI),
        //         new SwerveModulePosition[]{
        //             frontLeft.getPosition(),
        //             frontRight.getPosition(),
        //             backLeft.getPosition(),
        //             backRight.getPosition()
        //         }
        //     );
        // } catch (Exception e) {
        //     System.out.println("Error updating odometry: " + e);
        // }
        return odometry;
    }

    public void resetOdometry() {
        resetOdometry(new Pose2d());
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(this.getHeading(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[]{
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
    }

    public SwerveModuleState[] getRealModuleStates() {
        return new SwerveModuleState[]{
            frontLeft.getRealState(),
            frontRight.getRealState(),
            backLeft.getRealState(),
            backRight.getRealState()
        };
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{
            frontLeft.getPosition(),
            frontRight.getPosition(),

            backLeft.getPosition(),
            backRight.getPosition(),
        };
    }

    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.SwerveDrive.PhysicalModel.kMaxSpeed.in(MetersPerSecond));
        frontLeft.setState(states[0]);
        frontRight.setState(states[1]);
        
        backLeft.setState(states[2]);
        backRight.setState(states[3]);
    }

    public void drive(ChassisSpeeds speeds) {
        this.speeds = speeds;
        SwerveModuleState[] m_moduleStates = Constants.SwerveDrive.PhysicalModel.kDriveKinematics.toSwerveModuleStates(speeds);
        this.setModuleStates(m_moduleStates);
    }

    public void driveFieldRelative(double xSpeed, double ySpeed, double rotSpeed) {
        this.drivingRobotRelative = false;
        this.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, this.getHeading()));
    }

    public void driveFieldRelative(ChassisSpeeds speeds) {
        this.drivingRobotRelative = false;
        this.drive(speeds);
    }

    public void driveRobotRelative(double xSpeed, double ySpeed, double rotSpeed) {
        this.drivingRobotRelative = true;
        this.drive(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        this.drivingRobotRelative = true;
        this.drive(speeds);
    }

    public void stop() {
        this.frontLeft.stop();
        this.frontRight.stop();
        this.backLeft.stop();
        this.backRight.stop();
    }

    public void xFormation() {
        this.frontLeft.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), true);
        this.frontRight.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
        this.backLeft.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
        this.backRight.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), true);
    }

    public void resetTurningEncoders() {
        this.frontLeft.resetTurningEncoder();
        this.frontRight.resetTurningEncoder();
        this.backLeft.resetTurningEncoder();
        this.backRight.resetTurningEncoder();
    }

    public void resetDriveEncoders() {
        this.frontLeft.resetDriveEncoder();
        this.frontRight.resetDriveEncoder();
        this.backLeft.resetDriveEncoder();
        this.backRight.resetDriveEncoder();
    }

    public void resetEncoders() {
        this.resetTurningEncoders();
        this.resetDriveEncoders();
    }

    public void updateInputs(SwerveDriveIOInputs inputs) {
        inputs.heading = this.getHeading().getDegrees();
        inputs.rotSpeed = speeds.omegaRadiansPerSecond;
        inputs.xSpeed = speeds.vxMetersPerSecond;
        inputs.ySpeed = speeds.vyMetersPerSecond;
    }

    public void periodic() {
        // Update inertia acculumator
        rotationalInertiaAccumulator.update(this.getHeading().getRadians());

        this.odometry.update(getHeading(), getModulePositions());
        poseEstimator.update(gyro.getRotation2d(), getModulePositions());
        
        for (Vision cam : cams) {
            poseEstimator.addVisionMeasurement(cam.getEstimatedPose(), cam.getTimestampSeconds());
        }

        try {
            LimelightResults results = LimelightHelpers.getLatestResults(cams[0].getName());
            if (results.targetingResults.targets_Fiducials.length >= 2) poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 99999999));
        } catch (Exception e) {}

        Logger.recordOutput("PoseEstimator/EstimatedPose", poseEstimator.getEstimatedPosition());

        // Record outputs
        Logger.recordOutput("SwerveDrive/RobotHeadingRad", this.getHeading().getRadians());
        Logger.recordOutput("SwerveDrive/RobotHeadingDeg", this.getHeading().getDegrees());

        Logger.recordOutput("SwerveDrive/RobotRotationalInertia", rotationalInertiaAccumulator.getTotalRotationalInertia());
        
        Logger.recordOutput("SwerveDrive/RobotPose", this.getPose());

        Logger.recordOutput("SwerveDrive/RobotRelative", this.drivingRobotRelative);
        Logger.recordOutput("SwerveDrive/RobotSpeeds", this.getRobotRelativeChassisSpeeds());
        
        Logger.recordOutput("SwerveDrive/SwerveModuleStates", this.getRealModuleStates());
    }
}
