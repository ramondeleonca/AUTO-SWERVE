package frc.robot.subsystems.SwerveModule;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {
    @AutoLog
    class SwerveModuleIOInputs {
        public double speed;
        public double angle;

        public double targetSpeed;
        public double targetAngle;

        public double distance;
    }
    
    void setState(SwerveModuleState state, boolean force);
    void setState(SwerveModuleState state);
    void stop();

    SwerveModulePosition getPosition();
    SwerveModuleState getRealState();
    SwerveModuleState getState();
    String getName();

    default void resetTurningEncoder() {};
    default void resetDriveEncoder() {};
    default void resetEncoders() {};

    default void updateInputs(SwerveModuleIOInputs inputs) {};
    default void periodic() {};
}
