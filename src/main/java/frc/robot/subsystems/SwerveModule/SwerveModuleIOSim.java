package frc.robot.subsystems.SwerveModule;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import lib.team3526.utils.SwerveModuleOptions;

public class SwerveModuleIOSim implements SwerveModuleIO {
    // Swerve module name
    private final String m_name;

    // Current state
    private SwerveModuleState state = new SwerveModuleState();
    private double stateUpdated = Timer.getFPGATimestamp();
    private double distance = 0;

    public SwerveModuleIOSim(SwerveModuleOptions options) {
        this.m_name = options.getName();
    }

    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.angle = state.angle.getRadians();
        inputs.speed = state.speedMetersPerSecond;
        inputs.distance = this.distance;
        this.distance += this.state.speedMetersPerSecond * (Timer.getFPGATimestamp() - this.stateUpdated);
    }
    
    public void stop() {
        state = new SwerveModuleState(0, state.angle);
    }

    public void setState(SwerveModuleState state) {
        setState(state, true);
    }

    public void setState(SwerveModuleState state, boolean force) {
        this.state = state;
        this.stateUpdated = Timer.getFPGATimestamp();
    }

    public String getName() {
        return this.m_name;
    }

    // Returns the current state
    public SwerveModuleState getState() {
        return state;
    }

    // Returns the current state
    public SwerveModuleState getRealState() {
        return state;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(this.distance, state.angle);
    }

    public void periodic() {
        Logger.recordOutput("SwerveDrive/" + this.getName() + "/RealState", this.getRealState());
        Logger.recordOutput("SwerveDrive/" + this.getName() + "/TargetState", this.getState());
    }
}
