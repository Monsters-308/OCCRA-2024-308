// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class RevUpShooterCommand extends Command {
  private final ShooterSubsystem m_shooterSubsystem;
  private final DoubleSupplier getShooterSpeed;

  /**
   * This creates a new rev up shooter command. This revs up the shooter to the correct speed before the ball gets fed into the shooter.
   * @param shooterSubsystem the shooter subsystem that controls the wheels in the shooter
   * @param shooterSpeed a functional interface the had a method get that returns a double that represents the value for the speed of the shooter
   */
  public RevUpShooterCommand(ShooterSubsystem shooterSubsystem, DoubleSupplier shooterSpeed) {
    m_shooterSubsystem = shooterSubsystem;
    getShooterSpeed = shooterSpeed;
  }

  /**
   * This creates a new rev up shooter command. This revs up the shooter to the correct speed before the ball gets fed into the shooter.
   * @param shooterSubsystem the shooter subsystem that controls the wheels in the shooter
   * @param shooterSpeed a double that represents the value for the speed of the shooter
   */
  public RevUpShooterCommand(ShooterSubsystem shooterSubsystem, Double shooterSpeed) {
    m_shooterSubsystem = shooterSubsystem;
    getShooterSpeed = () -> shooterSpeed;
  }

  @Override
  public void execute() {
    m_shooterSubsystem.startShooter(getShooterSpeed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.stopShooter();
  }
}
