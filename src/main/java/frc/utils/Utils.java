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
    
    /**
     * Controls the sensitivity of the joystick inputs.
     * @param rawSpeed The raw speed
     * @param sensitivity The senstivity of the joystick. Goes from 0-1 with 0.5 being linear.
     * @param deadband The deadband of the joystick to prevent stick drift. Any value lower than this, or above for negatives, will be made zero.
     */
    public static double sensitivityFunction(double rawSpeed, double sensitivity, double deadband) {
        double speedSign = Math.signum(rawSpeed);
        double speedWithDeadband = Math.abs(rawSpeed) < deadband ? 0 : Math.abs(rawSpeed);

        return speedSign * Math.pow(speedWithDeadband, 0.5/sensitivity);
    }
}
