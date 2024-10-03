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
    indexMotor.setInverted(IndexConstants.kIndexInverted);
    indexMotor.burnFlash();
  }

  /**
   * Gets the current speed of the indexer.
   */
  public double getMotorSpeed() {
    return indexMotor.get();
  }

  /**
   * Starts the index.
   * @param speed How fast the index motor should spin. Goes from -1 to 1, with -1 being full reverse and 1 being full forwards.
   */
  public void setMotorSpeed(double speed) {
    indexMotor.set(speed);
  }

  /**
   * Stops the indexer. This should be done after the ball exits the indexer.
   */
  public void stopMotor() {
    indexMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
