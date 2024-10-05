// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class MotorConstants {
    public static final int kNeoMotorMaxRPM = 5676;
  }

  public static final class DriveConstants {
    // ID for the motors.
    public static final int kLeftFrontMotorCANID = 17;
    public static final int kLeftBackMotorCANID = 17;
    public static final int kRightFrontMotorCANID = 17;
    public static final int kRightBackMotorCANID = 17;

    // config the inverted for each of the motors.
    public static final boolean kLeftFrontMotorInverted = false;
    public static final boolean kLeftBackMotorInverted = false;
    public static final boolean kRightFrontMotorInverted = false;
    public static final boolean kRightBackMotorInverted = false;

    // brake mode so the robot can't be pushed around
    public static final IdleMode kMotorIdleMode = IdleMode.kBrake;

    public static final int kSmartCurrentLimit = 30;

    // calculate the max theoretical speed in m/s (for pid)
    public static final double kWheelDiameter = Units.inchesToMeters(6);
    public static final double kWheelCircumference = kWheelDiameter * Math.PI;
    public static final double kGearRatio = 1;

    public static final double kMaxSpeedMetersPerSecond = 
      ((MotorConstants.kNeoMotorMaxRPM / 60.0) / kGearRatio) * kWheelCircumference;
    
    // PID constants for controlling wheel velocity
    public static final double kVelocityP = 0.1;
    public static final double kVelocityI = 0;
    public static final double kVelocityD = 0;
    public static final double kVelocityFF = 1 / kMaxSpeedMetersPerSecond;
    
    // converting motor rotations to distance traveled (for odometry)
    public static final double kEncoderConversionFactor = kWheelCircumference / kGearRatio;
  }

  public static final class IntakeConstants {
    // Channel for Intake
    public static final int kIntakeMotorChannel = 17;

     // make intake invert (positive = intake, negative = outtake) 
    public static final boolean kIntakeInverted = false;
  }

  public static final class IndexConstants {
    // Channel for Index
    public static final int kIndexMotorChannel = 17;
    public static final int kBallSensorPort = 1;
    
     // make index invert (positive = forwards, negative = backwards) 
    public static final boolean kIndexInverted = false;
  }

  public static final class ShooterConstants {
    public static final int kShooterTopMotorCANID = 17;
    public static final int kShooterBottomMotorCANID = 17;

    public static final IdleMode kShooterMotorIdleMode = IdleMode.kBrake;

    public static final int kSmartCurrentLimit = 30;

    public static final boolean kShooterMotorInverted = false; // make intake invert (positive = intake, negative = outtake)

    public static final double kWheelDiamter = 6;
    public static final double kWheelCircumference = Math.PI * kWheelDiamter;
    public static final double kGearRatio = 1;
    public static final double kMaxMetersPerSecond = ((MotorConstants.kNeoMotorMaxRPM / 60) * kWheelCircumference) / kGearRatio;

    public static final double kVelocityP = 1;
    public static final double kVelocityI = 1;
    public static final double kVelocityD = 1;
    public static final double kVelocityFF = 1 / kMaxMetersPerSecond;
    
    public static final double kTurningEncoderPositionFactor = kWheelCircumference / kGearRatio;
    public static final double kTurningEncoderVelocityFactor = (kWheelCircumference / kGearRatio) / 60;

    public static final double backSpin = 0;
  }

  public static final class ClimbConstants {
    public static final int kClimbMotorID = 17;

    public static final boolean kClimbInverted = false;
   
  }

  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCoDriverControllerPort = 1;
  }
}
