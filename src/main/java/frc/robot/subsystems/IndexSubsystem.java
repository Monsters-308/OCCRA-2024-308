// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IndexConstants;

public class IndexSubsystem extends SubsystemBase {
  private final CANSparkMax indexMotor = new CANSparkMax(IndexConstants.kIndexMotorCANID, MotorType.kBrushless);

  /** Creates a new IndexSubsystem. */
  public IndexSubsystem() {
    indexMotor.restoreFactoryDefaults();
    indexMotor.setSmartCurrentLimit(IndexConstants.kSmartCurrentLimitIntake);
    indexMotor.setIdleMode(IndexConstants.kIndexMotorIdleMode);
    indexMotor.burnFlash();
  }

  public void runMotor(double speed) {
    indexMotor.set(speed);
  }

  public void setInverted(boolean inverted) {
    indexMotor.setInverted(inverted);
    indexMotor.burnFlash();
  }

  public void stopMotor(double speed) {
    indexMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
