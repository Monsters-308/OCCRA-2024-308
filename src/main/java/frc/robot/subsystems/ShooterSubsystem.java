// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;
import frc.utils.SparkSendablePID;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax topShootMotor = new CANSparkMax(ShooterConstants.kShooterTopMotorCANID, MotorType.kBrushless);
  private final CANSparkMax bottomShootMotor = new CANSparkMax(ShooterConstants.kShooterBottomMotorCANID, MotorType.kBrushless);

  private final SparkPIDController topMotorPIDController = topShootMotor.getPIDController();
  private final SparkPIDController bottomMotorPIDController = bottomShootMotor.getPIDController();

  private final RelativeEncoder topMotorEncoder = topShootMotor.getEncoder();
  private final RelativeEncoder bottomMotorEncoder = bottomShootMotor.getEncoder();

  /** This creates a new shooter subsystem, which manages the speed of the shooter wheels, so the ball can move out at the correct speed. */
  public ShooterSubsystem() {
    topMotorPIDController.setP(ShooterConstants.kVelocityP);
    topMotorPIDController.setI(ShooterConstants.kVelocityI);
    topMotorPIDController.setD(ShooterConstants.kVelocityD);
    topMotorPIDController.setFF(ShooterConstants.kVelocityFF);

    topMotorEncoder.setPositionConversionFactor(ShooterConstants.kTurningEncoderPositionFactor);
    topMotorEncoder.setVelocityConversionFactor(ShooterConstants.kTurningEncoderVelocityFactor);

    topShootMotor.restoreFactoryDefaults();
    topShootMotor.setIdleMode(ShooterConstants.kShooterMotorIdleMode);
    topShootMotor.setInverted(ShooterConstants.kTopShooterMotorInverted);
    topShootMotor.burnFlash();

    bottomMotorPIDController.setP(ShooterConstants.kVelocityP);
    bottomMotorPIDController.setI(ShooterConstants.kVelocityI);
    bottomMotorPIDController.setD(ShooterConstants.kVelocityD);
    bottomMotorPIDController.setFF(ShooterConstants.kVelocityFF);

    bottomMotorEncoder.setPositionConversionFactor(ShooterConstants.kTurningEncoderPositionFactor);
    bottomMotorEncoder.setVelocityConversionFactor(ShooterConstants.kTurningEncoderVelocityFactor);
    
    bottomShootMotor.restoreFactoryDefaults();
    bottomShootMotor.setIdleMode(ShooterConstants.kShooterMotorIdleMode);
    bottomShootMotor.setInverted(ShooterConstants.kBottomShooterMotorInverted);
    bottomShootMotor.burnFlash();

    Shuffleboard.getTab("Shooter").addDouble("Top Motor Speed", topMotorEncoder::getVelocity);
    Shuffleboard.getTab("Shooter").addDouble("Bottom Motor Speed", bottomMotorEncoder::getVelocity);

    Shuffleboard.getTab("Shooter").add("I AM SPEED", 
      new RepeatCommand(new InstantCommand(() -> setPercent(1, 1), this)).finallyDo(() -> stopShooter()));

    Shuffleboard.getTab("Shooter").add("Shooter PID controller", new SparkSendablePID(topMotorPIDController, ControlType.kVelocity));
  } 
  
  /** Starts the shooter with a specified top and bottom speed, in order to achieve backspin
   * @param topSpeed the speed you want the top shooter wheel to move at (-1 - 1)
   * @param bottomSpeed the speed you want the bottom shooter wheel to move at (-1 - 1)
   * **/
  public void startShooter(double topSpeed, double bottomSpeed) {
    topMotorPIDController.setReference(topSpeed * ShooterConstants.kMaxMetersPerSecond, ControlType.kVelocity);
    bottomMotorPIDController.setReference(bottomSpeed * ShooterConstants.kMaxMetersPerSecond, ControlType.kVelocity);
  }
  
  /** Starts the shooter with the same speed for the top and bottom
   * @param speed the speed you want the shooter wheels to move at (-1 - 1)
   */
  public void startShooter(double speed) {
    topMotorPIDController.setReference(speed * ShooterConstants.kMaxMetersPerSecond, ControlType.kVelocity);
    bottomMotorPIDController.setReference(speed * ShooterConstants.kMaxMetersPerSecond, ControlType.kVelocity);
  }

  /** Sets the motors to a specific percentage. This is for testing purposes. */
  public void setPercent(double topSpeed, double bottomSpeed) {
    topShootMotor.set(topSpeed);
    bottomShootMotor.set(bottomSpeed);
  }
  
  /** Stops the motor and sets them both to 0 */
  public void stopShooter() {
    topShootMotor.set(0);
    bottomShootMotor.set(0);
  }
}
