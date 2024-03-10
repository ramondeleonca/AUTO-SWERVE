package frc.robot.subsystems.IntakeLifter;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import lib.team3526.control.LazyCANSparkMax;

public class IntakeLifterIOReal implements IntakeLifterIO {
    private final LazyCANSparkMax lifterMotor;
    private final ProfiledPIDController lifterMotorPID;
    private final DutyCycleEncoder lifterEncoder;

    private Measure<Angle> desiredAngle = Degrees.of(0.0);

    public IntakeLifterIOReal() {
        this.lifterMotor = new LazyCANSparkMax(Constants.Intake.kLifterMotorID, MotorType.kBrushless);
        this.lifterMotor.setInverted(true);
        this.lifterMotorPID = Constants.Intake.kLifterPIDController;
        this.lifterEncoder = new DutyCycleEncoder(Constants.Intake.kLifterEncoderPort);
    }

    /**
     * @param angleDeg The angle to set the lifter to
     * @return true if the lifter is within 5 degrees of the target angle
     */
    public void setLifterAngle(Measure<Angle> angleDeg) {
        this.desiredAngle = angleDeg;
    }

    public double getLifterAngleRadians() {
        double angleRadians = (this.lifterEncoder.getAbsolutePosition()-Constants.Intake.kLifterEncoderOffset)*2*Math.PI;
        return ((Math.floor(angleRadians * 1000)) / 1000);
    }

    public void setLifterSpeed(double speed) {
        this.lifterMotor.set(MathUtil.clamp(speed, -Constants.Intake.kMaxLifterSpeed, Constants.Intake.kMaxLifterSpeed));
    }

    public void stopLifter() {
        this.lifterMotor.set(0);
    }

    public void periodic() {
        Logger.recordOutput("Intake/LifterAngle", Math.toDegrees(this.getLifterAngleRadians()));
        Logger.recordOutput("Intake/Lifter", this.lifterMotor.getAppliedOutput());
        Logger.recordOutput("Intake/SetAngle", this.desiredAngle.in(Degrees));


        SmartDashboard.putData(this.lifterMotorPID);
    }

    public void updateInputs(IntakeLifterIOInputs inputs) {
        inputs.lifterAngle = this.getLifterAngleRadians();
    }
}
