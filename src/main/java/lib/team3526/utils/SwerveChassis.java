package lib.team3526.utils;

import edu.wpi.first.math.geometry.Translation2d;

public class SwerveChassis {
    public static Translation2d[] sizeToModulePositions(double trackWidth, double wheelBase) {
        return new Translation2d[] {
            new Translation2d(trackWidth / 2, wheelBase / 2),
            new Translation2d(trackWidth / 2, -wheelBase / 2),
            new Translation2d(-trackWidth / 2, wheelBase / 2),
            new Translation2d(-trackWidth / 2, -wheelBase / 2)
        };
    }
}
