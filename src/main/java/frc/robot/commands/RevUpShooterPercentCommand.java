// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class RevUpShooterPercentCommand extends Command {
  private final ShooterSubsystem m_shooterSubsystem;
  private final double m_topShooterSpeed;
  private final double m_bottomShooterSpeed;

  /**
   * This creates a new rev up shooter command. This revs up the shooter to the correct speed before the ball gets fed into the shooter.
   * @param shooterSubsystem the shooter subsystem that controls the wheels in the shooter
   * @param shooterSpeed a functional interface the had a method get that returns a double that represents the value for the speed of the shooter
   */
  public RevUpShooterPercentCommand(ShooterSubsystem shooterSubsystem, double topShooterSpeed, double bottomShooterSpeed) {
    m_shooterSubsystem = shooterSubsystem;
    m_topShooterSpeed = topShooterSpeed;
    m_bottomShooterSpeed = bottomShooterSpeed;

    addRequirements(shooterSubsystem);
  }

  @Override
  public void execute() {
    m_shooterSubsystem.setPercent(m_topShooterSpeed, m_bottomShooterSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.stopShooter();
  }
}
