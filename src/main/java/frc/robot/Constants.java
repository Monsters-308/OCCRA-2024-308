// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
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
    public static final int kLeftFrontMotorCANID = 2;
    public static final int kLeftBackMotorCANID = 3;
    public static final int kRightFrontMotorCANID = 4;
    public static final int kRightBackMotorCANID = 5;

    // config the inverted for each of the motors.
    public static final boolean kLeftFrontMotorInverted = true;
    public static final boolean kLeftBackMotorInverted = true;
    public static final boolean kRightFrontMotorInverted = false;
    public static final boolean kRightBackMotorInverted = false;

    // brake mode so the robot can't be pushed around
    public static final IdleMode kMotorIdleMode = IdleMode.kBrake;

    public static final int kSmartCurrentLimit = 30;

    // calculate the max theoretical speed in m/s (for pid)
    public static final double kWheelDiameter = Units.inchesToMeters(6);
    public static final double kWheelCircumference = kWheelDiameter * Math.PI;
    public static final double kGearRatio = 8.45;

    public static final double kMaxSpeedMetersPerSecond = 
      ((MotorConstants.kNeoMotorMaxRPM / 60.0) / kGearRatio) * kWheelCircumference;
    
    // PID constants for controlling wheel velocity
    public static final double kVelocityP = 0.01;
    public static final double kVelocityI = 0;
    public static final double kVelocityD = 0;
    public static final double kVelocityFF = 1 / kMaxSpeedMetersPerSecond;
    
    // converting motor rotations to distance traveled (for odometry)
    public static final double kEncoderConversionFactor = kWheelCircumference / kGearRatio;

    // Forwards should be positive for the encoders
    public static final boolean kInvertEncoders = true;

    // kinematics

    // Track width: this is the distance between the wheels
    public static final double kTrackWidthMeters = Units.inchesToMeters(26);
    // This may be larger due to scrubbing effects
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters); 

    // the theoretical max rotational speed of the robot (in radians per second)
    public static final double kMaxAngularSpeed = kDriveKinematics.toChassisSpeeds(
      new DifferentialDriveWheelSpeeds(-kMaxSpeedMetersPerSecond, kMaxSpeedMetersPerSecond)
    ).omegaRadiansPerSecond;

    // Determines which direction is positive for the gyro
    // Pathplanner uses CCW positive
    public static final boolean kInvertGyro = true;

    // Dead band of joysticks. If the joystick is this distance from the center, the speed will register as zero. This prevents stick drift.
    // Edit this to control how much you have to move the stick in order for it to register as an input. It is a decimal that goes from 0-1.
    public static final double kDeadBand = 0.05;

    // Controls the sensitivity of the joysticks.
    // These will be edited by the driver for prefrence
    public static final double kDriverSensitvity = 0.5;
    public static final double kRotationalSensitivity = 0.2;

    // Limits the rate of change of the driver speed and rotation, respectively.
    public static final double kSpeedSlewRateLimit = 1.0; // Units per second
    public static final double kRotationalSlewRateLimit = 2.0; // Units per second
  }

  public static final class IntakeConstants {
    // Channel for Intake
    public static final int kIntakeMotorChannel = 8;

     // make intake invert (positive = intake, negative = outtake) 
    public static final boolean kIntakeInverted = true;
    public static final NeutralModeValue kMotorIdleMode = NeutralModeValue.Coast;

    public static final double kIntakeSpeed = 0.3;
  }

  public static final class IndexConstants {
    // Channel for Index
    public static final int kIndexMotorChannel = 9;
    public static final int kBallSensorPort = 0;
    
     // make index invert (positive = forwards, negative = backwards) 
    public static final boolean kIndexInverted = true;
    public static final NeutralModeValue kMotorIdleMode = NeutralModeValue.Brake;

    public static final double kIndexSpeed = 0.3;
  }

  public static final class ShooterConstants {
    public static final int kShooterTopMotorCANID = 25;
    public static final int kShooterBottomMotorCANID = 30;

    public static final IdleMode kShooterMotorIdleMode = IdleMode.kCoast;

    public static final int kSmartCurrentLimit = 30;

    public static final boolean kTopShooterMotorInverted = false; // makes top shooter wheel invert (positive = outwards, negative = inwards)
    public static final boolean kBottomShooterMotorInverted = true; // makes bottom shooter wheel invert (positive = outwards, negative = inwards)

    public static final double kWheelDiamter = Units.inchesToMeters(4);
    public static final double kWheelCircumference = Math.PI * kWheelDiamter;
    public static final double kGearRatio = 1.6;
    public static final double kMaxMetersPerSecond = ((MotorConstants.kNeoMotorMaxRPM / 60) * kWheelCircumference) / kGearRatio;

    public static final double kVelocityP = 1;
    public static final double kVelocityI = 0;
    public static final double kVelocityD = 0;
    public static final double kVelocityFF = 1 / kMaxMetersPerSecond;
    
    public static final double kTurningEncoderPositionFactor = kWheelCircumference / kGearRatio;
    public static final double kTurningEncoderVelocityFactor = (kWheelCircumference / kGearRatio) / 60;

    public static final double kBackSpin = 0;
  }

  public static final class ClimbConstants {
    public static final int kClimbMotorID = 8;

    public static final boolean kClimbInverted = false;
   
  }

  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCoDriverControllerPort = 1;
  }

  public static final class AutonomousConstants{
    public static final double kBallLaunchTimeout = 1;

    public static final double kShooterSpeed = 0.5;
  }

  public static final class LEDConstants {
    public static final int maxLEDIndex = 5;
  }
}
