// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Consumer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
private static final Object[] Intake = null;

  // getting the motor of the intake. :3
  CANSparkMax IntakeMotor1 = new CANSparkMax(IntakeConstants.kIntakeMotorCANID, MotorType.kBrushless);

  public IntakeSubsystem(){
    applyToAll((Intake) -> {
      Intake.restoreFactoryDefaults(); // <-- restores to default
      Intake.setSmartCurrentLimit(IntakeConstants.kSmartCurrentLimitIntake);// <- limit the power usage
      Intake.setIdleMode(IntakeConstants.kIntakeMotorIdleMode); // sets idle mode
      Intake.burnFlash();
    });
      IntakeMotor1.setInverted(IntakeConstants.kIntakeMotorInverted);
    


  }


  public void IntakeSpin(){
    IntakeMotor1.set(0.5);// <-- idk what speed and if we want a const speed.
  }

  public void IntakeUnspin(){
    IntakeMotor1.set(0);

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  /**
   * @param func
   */
  private void applyToAll(Consumer<CANSparkMax> func){
    CANSparkMax[] Intake = {IntakeMotor1}; // <- I just made this consumer to make it easy if we need more motors. for example {Intakemotor1, Intakemotor2}

    for (int i = 0; i < Intake.length; i++){
      func.accept(Intake[i]);

    }

  }
}




