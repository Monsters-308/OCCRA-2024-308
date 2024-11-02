// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final Spark bobIntakeMotor = new Spark(IntakeConstants.kIntakeMotorChannel);

  /**
   * Creates a new IntakeSubsystem. which manages is what intakes the ball from the human players, or outtakes a ball or other object that should not be there.
   **/
  public IntakeSubsystem(){
    bobIntakeMotor.setInverted(IntakeConstants.kIntakeInverted);
  }

  /**
   * Gets the current speed of the intake.
   */
  public double getIntakeSpeed() {
    return bobIntakeMotor.get();
  }

  /**
   * Starts the intake.
   * @param speed How fast the intake motor should spin. Goes from -1 to 1, with -1 being full reverse and 1 being full forwards.
   */
  public void setIntakeSpeed(double speed) {
    bobIntakeMotor.set(speed);
  }

  /**
   * Stops the intake. This should be done after the ball exits the intake.
   */
  public void stopIntake() {
    bobIntakeMotor.set(0);
  }
}
