// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterIntakeCommand extends Command {
  private final ShooterSubsystem m_shooterSubsystem;
  private final double m_IntakeShooterSpeed;

  /**
   * This creates a shooter intake command. This allows you to intake a ball using a shooter.
   * @param shooterSubsystem the shooter subsystem that controls the wheels in the shooter
   */
  public ShooterIntakeCommand(ShooterSubsystem shooterSubsystem) {
    m_shooterSubsystem = shooterSubsystem;
    m_IntakeShooterSpeed = ShooterConstants.kBackupIntakeSpeedShooter;

    addRequirements(shooterSubsystem);
  }

  @Override
  public void execute() {
    m_shooterSubsystem.setPercent(m_IntakeShooterSpeed, m_IntakeShooterSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.stopShooter();
  }
}
