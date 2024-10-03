// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  // getting the motor of the index
  private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorCANID, MotorType.kBrushless);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(){
      // sets the motor settings 
      intakeMotor.restoreFactoryDefaults(); // <-- restores to default
      intakeMotor.setSmartCurrentLimit(IntakeConstants.kSmartCurrentLimitIntake);// <- limit the power usage
      intakeMotor.setIdleMode(IntakeConstants.kIntakeMotorIdleMode); // sets idle mode
      intakeMotor.setInverted(IntakeConstants.kIntakeInverted);
      intakeMotor.burnFlash();
  }

  /**
   * Gets the current speed of the intake.
   */
  public double getMotorSpeed() {
    return intakeMotor.get();
  }

  /**
   * Starts the intake.
   * @param speed How fast the intake motor should spin. Goes from -1 to 1, with -1 being full reverse and 1 being full forwards.
   */
  public void setMotorSpeed(double speed) {
    intakeMotor.set(speed);
  }

  /**
   * Stops the intake. This should be done after the ball exits the indexer.
   */
  public void stopMotor() {
    intakeMotor.set(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run :3
  }

}
