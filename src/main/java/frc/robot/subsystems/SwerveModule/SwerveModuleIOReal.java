package frc.robot.subsystems.SwerveModule;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import lib.team3526.constants.PIDFConstants;
import lib.team3526.control.LazyCANSparkMax;
import lib.team3526.utils.SwerveModuleOptions;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

public class SwerveModuleIOReal implements SwerveModuleIO {
    private SwerveModuleOptions options;

    private final LazyCANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final SparkPIDController turningPID;

    private final CANcoder absoluteEncoder;

    private SwerveModuleState state = new SwerveModuleState();

    public SwerveModuleIOReal(SwerveModuleOptions options) {
        // Store the options
        this.options = options;

        // Create the motors
        this.driveMotor = new LazyCANSparkMax(options.driveMotorID, MotorType.kBrushless);
        this.turningMotor = new CANSparkMax(options.turningMotorID, MotorType.kBrushless);

        this.turningMotor.setInverted(options.turningMotorInverted);

        // Get and configure the encoders
        this.driveEncoder = this.driveMotor.getEncoder();
        this.driveEncoder.setPositionConversionFactor(Constants.SwerveDrive.PhysicalModel.kDriveEncoder_RotationToMeter); 
        this.driveEncoder.setVelocityConversionFactor(Constants.SwerveDrive.PhysicalModel.kDriveEncoder_RPMToMeterPerSecond);
        this.driveMotor.setInverted(options.driveMotorInverted);

        this.turningEncoder = this.turningMotor.getEncoder();
        this.turningEncoder.setPositionConversionFactor(Constants.SwerveDrive.PhysicalModel.kTurningEncoder_RotationToRadian); 
        this.turningEncoder.setVelocityConversionFactor(Constants.SwerveDrive.PhysicalModel.kTurningEncoder_RPMToRadianPerSecond);

        this.turningPID = this.turningMotor.getPIDController();
        PIDFConstants.applyToSparkPIDController(this.turningPID, Constants.SwerveDrive.SwerveModules.kTurningPIDConstants);
        this.turningPID.setPositionPIDWrappingMinInput(0);
        this.turningPID.setPositionPIDWrappingMaxInput(2 * Math.PI);
        this.turningPID.setPositionPIDWrappingEnabled(true);

        // Configure the absolute encoder
        this.absoluteEncoder = new CANcoder(options.getAbsoluteEncoderCANDevice().getDeviceID(), options.getAbsoluteEncoderCANDevice().getCanbus());
        // this.absoluteEncoder.getConfigurator().apply(
        //     new MagnetSensorConfigs()
        //         //! VAN EN PHOENIX
        //         //.withMagnetOffset((this.options.getOffsetRad() + Constants.SwerveDrive.SwerveModules.kGlobalOffset.in(Radians))  / (2 * Math.PI))
        //         .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
        // );

        // Reset the encoders
        resetEncoders();
        
    }

    public Measure<Angle> getAbsoluteEncoderPosition() {
        return Radians.of((absoluteEncoder.getAbsolutePosition().refresh().getValue() * 2 * Math.PI) * (this.options.absoluteEncoderInverted ? -1.0 : 1.0));
    }

    public void resetDriveEncoder() {
        this.driveEncoder.setPosition(0);
    }

    public void resetTurningEncoder() {
        this.turningEncoder.setPosition(getAbsoluteEncoderPosition().in(Radians));
    }
    
    public void resetEncoders() {
        resetDriveEncoder();
        resetTurningEncoder();
    }

    public Measure<Angle> getAngle() {
        return Radians.of(this.turningEncoder.getPosition() % (2 * Math.PI));
    }

    public void setState(SwerveModuleState state) {
        setState(state, false);
    }

    public void setState(SwerveModuleState state, boolean force) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001 || force) {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, Rotation2d.fromRadians(getAngle().in(Radians)));

        this.state = state;

        driveMotor.set(state.speedMetersPerSecond / Constants.SwerveDrive.PhysicalModel.kMaxSpeed.in(MetersPerSecond));
        turningPID.setReference(state.angle.getRadians(), ControlType.kPosition);
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    public SwerveModuleState getState() {
        return this.state;
    }

    public SwerveModuleState getRealState() {
        return new SwerveModuleState(
            this.driveEncoder.getVelocity(),
            Rotation2d.fromRadians(this.getAngle().in(Radians))
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            this.driveEncoder.getPosition(),
            Rotation2d.fromRadians(this.getAngle().in(Radians))
        );
    }

    public String getName() {
        return this.options.name;
    }

    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.angle = this.getAngle().in(Radians);
        inputs.speed = this.driveEncoder.getVelocity();

        inputs.targetAngle = this.state.angle.getRadians();
        inputs.targetSpeed = this.state.speedMetersPerSecond;

        inputs.distance = this.driveEncoder.getPosition();
    }

    double lastResetTime = Timer.getFPGATimestamp();
    public void periodic() {
        if (Timer.getFPGATimestamp() - lastResetTime > 1.0) {
            // resetTurningEncoder();
            lastResetTime = Timer.getFPGATimestamp();
        }

        Logger.recordOutput("SwerveDrive/" + this.getName() + "/MotEncoderDeg", this.getAngle().in(Degrees));
        Logger.recordOutput("SwerveDrive/" + this.getName() + "/AbsEncoderDeg", this.getAbsoluteEncoderPosition().in(Degrees));
        Logger.recordOutput("SwerveDrive/" + this.getName() + "/RealState", this.getRealState());
        Logger.recordOutput("SwerveDrive/" + this.getName() + "/TargetState", this.getState());
    }
}
