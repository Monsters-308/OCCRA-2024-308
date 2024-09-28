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
  CANSparkMax IntakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorCANID, MotorType.kBrushless);

  public IntakeSubsystem(){
    IntakeMotor.restoreFactoryDefaults(); // <-- restores to default
    IntakeMotor.setSmartCurrentLimit(IntakeConstants.kSmartCurrentLimitIntake);// <- limit the power usage
    IntakeMotor.setIdleMode(IntakeConstants.kIntakeMotorIdleMode);
    IntakeMotor.setInverted(IntakeConstants.kIntakeMotorInverted);
    IntakeMotor.burnFlash();
  }


  public void IntakeSpin(){
    IntakeMotor.set(0.5);// <-- idk what speed and if we want a const speed.
  }

  public void IntakeUnspin(){
    IntakeMotor.set(0);

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
