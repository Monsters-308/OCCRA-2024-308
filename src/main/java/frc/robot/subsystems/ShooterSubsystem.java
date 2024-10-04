// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax topShootMotor = new CANSparkMax(ShooterConstants.kShooterTopMotorCANID, MotorType.kBrushless);
  private final CANSparkMax bottomShootMotor = new CANSparkMax(ShooterConstants.kShooterBottomMotorCANID, MotorType.kBrushless);

  private final SparkPIDController topMotorPIDController = topShootMotor.getPIDController();
  private final SparkPIDController bottomMotorPIDController = bottomShootMotor.getPIDController();

  private final RelativeEncoder topMotorEncoder = topShootMotor.getEncoder();
  private final RelativeEncoder bottomMotorEncoder = bottomShootMotor.getEncoder();

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    topMotorPIDController.setP(ShooterConstants.kVelocityP);
    topMotorPIDController.setI(ShooterConstants.kVelocityI);
    topMotorPIDController.setD(ShooterConstants.kVelocityD);
    topMotorPIDController.setFF(ShooterConstants.kVelocityFF);

    topMotorEncoder.setPositionConversionFactor(ShooterConstants.kTurningEncoderPositionFactor);
    topMotorEncoder.setVelocityConversionFactor(ShooterConstants.kTurningEncoderVelocityFactor);

    topShootMotor.restoreFactoryDefaults();
    topShootMotor.setIdleMode(ShooterConstants.kShooterMotorIdleMode);
    topShootMotor.setInverted(ShooterConstants.kShooterMotorInverted);
    topShootMotor.burnFlash();

    bottomMotorPIDController.setP(ShooterConstants.kVelocityP);
    bottomMotorPIDController.setI(ShooterConstants.kVelocityI);
    bottomMotorPIDController.setD(ShooterConstants.kVelocityD);
    bottomMotorPIDController.setFF(ShooterConstants.kVelocityFF);

    bottomMotorEncoder.setPositionConversionFactor(ShooterConstants.kTurningEncoderPositionFactor);
    bottomMotorEncoder.setVelocityConversionFactor(ShooterConstants.kTurningEncoderVelocityFactor);

    bottomShootMotor.restoreFactoryDefaults();
    bottomShootMotor.setIdleMode(ShooterConstants.kShooterMotorIdleMode);
    bottomShootMotor.setInverted(ShooterConstants.kShooterMotorInverted);
    bottomShootMotor.burnFlash();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
