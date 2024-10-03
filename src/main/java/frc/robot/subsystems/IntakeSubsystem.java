// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  // getting the motor of the intake. :3
  private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorCANID, MotorType.kBrushless);

  public IntakeSubsystem(){
      // sets the motor settings 
      intakeMotor.restoreFactoryDefaults(); // <-- restores to default
      intakeMotor.setSmartCurrentLimit(IntakeConstants.kSmartCurrentLimitIntake);// <- limit the power usage
      intakeMotor.setIdleMode(IntakeConstants.kIntakeMotorIdleMode); // sets idle mode
      intakeMotor.setInverted(false);
      intakeMotor.burnFlash();
  }

  public void setInverted(boolean inverted) {
    intakeMotor.setInverted(inverted);
    intakeMotor.burnFlash();
  }

  public void runMotor(double speed){
    intakeMotor.set(speed);
  }

  public void stopMotor(){
    intakeMotor.set(0); // or can I use IntakeMotor.stopMotor();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run :3
  }

}

// rip for my insanity and rip to the consumer layout :(


