// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  private final CANSparkMax leftFrontMotor = new CANSparkMax(DriveConstants.kLeftFrontMotorCANID, MotorType.kBrushless);
  private final CANSparkMax leftBackMotor = new CANSparkMax(DriveConstants.kLeftBackMotorCANID, MotorType.kBrushless);
  private final CANSparkMax rightFrontMotor = new CANSparkMax(DriveConstants.kRightFrontMotorCANID, MotorType.kBrushless);
  private final CANSparkMax rightBackMotor = new CANSparkMax(DriveConstants.kRightBackMotorCANID, MotorType.kBrushless);

  // encoders
  private final RelativeEncoder leftFrontEncoder = leftFrontMotor.getEncoder();
  private final RelativeEncoder leftBackEncoder = leftBackMotor.getEncoder();
  private final RelativeEncoder rightFrontEncoder = rightFrontMotor.getEncoder();
  private final RelativeEncoder rightBackEncoder = rightBackMotor.getEncoder();

  private final RelativeEncoder[] Encoders = new RelativeEncoder[]{leftFrontEncoder, leftBackEncoder, rightFrontEncoder, rightBackEncoder};

  // PID controllers
  private final SparkPIDController leftFrontPID = leftFrontMotor.getPIDController();
  private final SparkPIDController leftBackPID = leftBackMotor.getPIDController();
  private final SparkPIDController rightFrontPID = rightFrontMotor.getPIDController();
  private final SparkPIDController rightBackPID = rightBackMotor.getPIDController();

  private final SparkPIDController[] PIDControllers = new SparkPIDController[]{leftFrontPID, leftBackPID, rightFrontPID, rightBackPID};

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    
    applyAllMotors(motor -> {
      // Reset factory defaults
      motor.restoreFactoryDefaults();
      // "smart limit"
      motor.setSmartCurrentLimit(DriveConstants.kSmartCurrentLimit);
      // brake gota go slow 3:
      motor.setIdleMode(DriveConstants.kMotorIdleMode);
    });
   
    // invert settings AAAAAAAAAAAAAAAAH
    leftFrontMotor.setInverted(DriveConstants.kLeftFrontMotorInverted);
    rightFrontMotor.setInverted(DriveConstants.kRightFrontMotorInverted);
    leftBackMotor.setInverted(DriveConstants.kLeftBackMotorInverted);
    rightBackMotor.setInverted(DriveConstants.kRightBackMotorInverted);

    // set encoder feedback
    leftFrontPID.setFeedbackDevice(leftFrontEncoder);
    leftBackPID.setFeedbackDevice(leftBackEncoder);
    rightFrontPID.setFeedbackDevice(rightFrontEncoder);
    rightBackPID.setFeedbackDevice(rightBackEncoder);

    for (RelativeEncoder encoder : Encoders) {
      encoder.setPositionConversionFactor(DriveConstants.kEncoderConversionFactor); // meters
      encoder.setVelocityConversionFactor(DriveConstants.kEncoderConversionFactor / 60.0); // meters per second
    }

    for (SparkPIDController PIDController : PIDControllers) {
      PIDController.setP(DriveConstants.kVelocityP);
      PIDController.setI(DriveConstants.kVelocityI);
      PIDController.setD(DriveConstants.kVelocityD);
      PIDController.setFF(DriveConstants.kVelocityFF);
    }

    applyAllMotors(motor -> motor.burnFlash());
  }

  /**
   * Drives the robot.
   * @param forwardSpeed How fast to move forwards/backwards. -1 is full reverse; 1 is full forwards.
   * @param turningSpeed How fast it turns. -1 is full left; 1 is full right.
   */
  public void drive(double forwardSpeed, double turningSpeed) {
    // calculate speeds
    double leftSpeed = forwardSpeed + turningSpeed;
    double rightSpeed = forwardSpeed - turningSpeed;

    // normalize speeds
    double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
    if (max > 1) {
      leftSpeed /= max;
      rightSpeed /= max;
    }

    // write speeds to motors
    leftFrontPID.setReference(DriveConstants.kMaxSpeedMetersPerSecond * leftSpeed, ControlType.kVelocity);
    leftBackPID.setReference(DriveConstants.kMaxSpeedMetersPerSecond * leftSpeed, ControlType.kVelocity);
    rightFrontPID.setReference(DriveConstants.kMaxSpeedMetersPerSecond * rightSpeed, ControlType.kVelocity);
    rightBackPID.setReference(DriveConstants.kMaxSpeedMetersPerSecond * rightSpeed, ControlType.kVelocity);
  }

  /**
   * Stops the robot.
   */
  public void stopDrive() {
    applyAllMotors((motor) -> motor.set(0));
  }

  /**
   * Sets the drive motors to a specified percentage.
   * THIS IS FOR TESTING PURPOSES.
   * @param speed The drivetrain speed from -1 to 1.
   */
  public void setPercent(double speed) {
    applyAllMotors((motor) -> motor.set(speed));
  }

  /**
   * Applies a function to every drive motor.
   * @param function This function is called for every motor. It passes one CANSparkMax object into the function.
   */
  private void applyAllMotors(Consumer<CANSparkMax> function) {
    CANSparkMax[] motors = {leftFrontMotor, leftBackMotor, rightBackMotor, leftFrontMotor};

    for (int i = 0; i < motors.length; i++) {
      function.accept(motors[i]);
    }
  }
  
  /**
   * This method runs automatically every scheduler run. It shouldn't be run manually.
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
