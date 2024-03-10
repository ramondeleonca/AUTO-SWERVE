package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import lib.team3526.constants.PIDFConstants;
import lib.team3526.control.LazyCANSparkMax;

public class ShooterIOReal implements ShooterIO {
    LazyCANSparkMax leftMotor;
    LazyCANSparkMax rightMotor;

    SparkPIDController leftPID;
    SparkPIDController rightPID;

    RelativeEncoder leftEncoder;
    RelativeEncoder rightEncoder;

    Measure<Velocity<Angle>> leftTargetRpm = RPM.zero();
    Measure<Velocity<Angle>> rightTargetRpm = RPM.zero();

    public ShooterIOReal() {
        this.leftMotor = new LazyCANSparkMax(Constants.Shooter.kLeftShooterMotorID, MotorType.kBrushless);
        this.leftMotor.setInverted(true);
        
        this.rightMotor = new LazyCANSparkMax(Constants.Shooter.kRightShooterMotorID, MotorType.kBrushless);

        this.leftPID = leftMotor.getPIDController();
        PIDFConstants.applyToSparkPIDController(leftPID, Constants.Shooter.kShooterPIDConstants);

        this.rightPID = rightMotor.getPIDController();
        PIDFConstants.applyToSparkPIDController(rightPID, Constants.Shooter.kShooterPIDConstants);

        this.leftEncoder = leftMotor.getEncoder();
        this.leftEncoder.setVelocityConversionFactor(Constants.Shooter.kShooterGearRatio);

        this.rightEncoder = rightMotor.getEncoder();
        this.rightEncoder.setVelocityConversionFactor(Constants.Shooter.kShooterGearRatio);
    }

    public void setLeftMotor(double speed) {
        this.leftTargetRpm = RPM.of(0);
        leftMotor.set(speed);
    }

    public void setRightMotor(double speed) {
        this.rightTargetRpm = RPM.of(0);
        rightMotor.set(speed);
    }

    public void set(double leftSpeed, double rightSpeed) {
        setLeftMotor(leftSpeed);
        setRightMotor(rightSpeed);
    }

    public void set(double speed) {
        set(speed, speed);
    }

    public void shootSpeaker() {
        setRpm(Constants.Shooter.kShooterSpeakerSpeed);
    }

    public void setLeftMotorRpm(Measure<Velocity<Angle>> rpm) {
        this.leftTargetRpm = rpm;
        leftPID.setReference(rpm.in(RPM), ControlType.kVelocity);
    }

    public void setRightMotorRpm(Measure<Velocity<Angle>> rpm) {
        this.rightTargetRpm = rpm;
        rightPID.setReference(rpm.in(RPM), ControlType.kVelocity);
    }

    public void setRpm(Measure<Velocity<Angle>> leftRpm, Measure<Velocity<Angle>> rightRpm) {
        setLeftMotorRpm(leftRpm);
        setRightMotorRpm(rightRpm);
    }

    public void setRpm(Measure<Velocity<Angle>> rpm) {
        setRpm(rpm, rpm);
    }

    public double getLeftMotorRpm() {
        return leftEncoder.getVelocity();
    }

    public double getRightMotorRpm() {
        return rightEncoder.getVelocity();
    }

    public double getLeftMotorPercentage() {
        return leftMotor.getAppliedOutput();
    }

    public double getRightMotorPercentage() {
        return rightMotor.getAppliedOutput();
    }

    public void stop() {
        set(0);
    }

    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leftPercentage = getLeftMotorPercentage();
        inputs.leftRpm = getLeftMotorRpm();
        inputs.leftTargetRpm = leftTargetRpm.magnitude();
        inputs.rightPercentage = getRightMotorPercentage();
        inputs.rightRpm = getRightMotorRpm();
        inputs.rightTargetRpm = rightTargetRpm.magnitude();
    }
}
