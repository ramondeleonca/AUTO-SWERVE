package lib.team3526.control;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

public class LazyCANSparkMax extends CANSparkMax {
    public LazyCANSparkMax(int deviceID, MotorType type) {
        super(deviceID, type);
    }

    public void set(double value) {
        if (value != get()) super.set(value);
    }

    public void setInverted(boolean isInverted) {
        if (isInverted != getInverted()) super.setInverted(isInverted);
    }

    public REVLibError setClosedLoopRampRate(double seconds) {
        if (seconds != getClosedLoopRampRate()) return super.setClosedLoopRampRate(seconds);
        return REVLibError.kOk;
    }

    public REVLibError setOpenLoopRampRate(double seconds) {
        if (seconds != getOpenLoopRampRate()) return super.setOpenLoopRampRate(seconds);
        return REVLibError.kOk;
    }
}
