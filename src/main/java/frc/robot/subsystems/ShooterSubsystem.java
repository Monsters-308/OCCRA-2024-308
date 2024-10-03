// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import java.util.function.Consumer;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
   private final CANSparkMax motor = new CANSparkMax(ShooterConstants.kShooterTopMotorCANID, MotorType.kBrushless);
   private final CANSparkMax shootMotorBottom = new CANSparkMax(ShooterConstants.kShooterBottomMotorCANID, MotorType.kBrushless);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    applyAllMotors(motor -> {
      motor.restoreFactoryDefaults();
      motor.setIdleMode(ShooterConstants.kIndexMotorIdleMode);
      motor.setInverted(ShooterConstants.kShooterMotorInverted);
      motor.burnFlash();
    });
  }

  /**
   * Applies a function to every drive motor.
   * @param function This function is called for every motor. It passes one CANSparkMax object into the function.
   */
  private void applyAllMotors(Consumer<CANSparkMax> function) {
    CANSparkMax[] motors = {motor, shootMotorBottom};

    for (int i = 0; i < motors.length; i++) {
      function.accept(motors[i]);
    }
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}


// ADD PID
