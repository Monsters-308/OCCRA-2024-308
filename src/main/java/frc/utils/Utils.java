package frc.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

/** A class to store useful functions that may be used across multiple files. */
public class Utils {

    /**
     * Constrains a continuous angle to a range from -180 to 180 degrees.
     * @param angle The continuous angle.
     * @return The angle from -180 to 180 degrees.
     */
    public static Rotation2d constrainAngle(Rotation2d angle) {
        double degrees = angle.getDegrees();

        return Rotation2d.fromDegrees(
            MathUtil.inputModulus(degrees, -180, 180)
        );
    }
    
}
