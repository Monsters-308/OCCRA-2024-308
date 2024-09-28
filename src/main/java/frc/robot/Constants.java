// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

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

    public static final IdleMode kMotorIdleMode = IdleMode.kBrake;

    public static final int kSmartCurrentLimit = 30;
  }

  public static final class IntakeConstants {
    // ID for Intake
    public static final int kIntakeMotorCANID = 17;
    // idle mode
    public static final IdleMode kIntakeMotorIdleMode = IdleMode.kBrake;
    // inverted
    public static final boolean kIntakeMotorInverted = false;
    // "smartlimit"
    public static final int kSmartCurrentLimitIntake = 30;
  }

  public static final class IndexConstants {
    public static final int kMotorCANID = 17;
  }

  public static final class ShooterConstants {
    public static final int kLeftMotorCANID = 17;
    public static final int kRightMotorCANID = 17;

    public static final int kSmartCurrentLimit = 30;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCoDriverControllerPort = 1;
  }
}
