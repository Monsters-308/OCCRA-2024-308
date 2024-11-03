// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.IndexConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterIntakeCommand extends Command {
  private final ShooterSubsystem m_shooterSubsystem;
  private final IndexSubsystem m_indexSubsystem;
  private final double m_intakeShooterSpeed = ShooterConstants.kBackupIntakeSpeedShooter;

  /**
   * This creates a shooter intake command. This allows you to intake a ball using a shooter.
   * @param shooterSubsystem the shooter subsystem that controls the wheels in the shooter
   */
  public ShooterIntakeCommand(ShooterSubsystem shooterSubsystem, IndexSubsystem indexSubsystem) {
    m_shooterSubsystem = shooterSubsystem;
    m_indexSubsystem = indexSubsystem;

    addRequirements(shooterSubsystem, indexSubsystem);
  }

  @Override
  public void execute() {
    m_shooterSubsystem.setPercent(m_intakeShooterSpeed, m_intakeShooterSpeed);
    m_indexSubsystem.setIndexSpeed(-IndexConstants.kIndexSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.stopShooter();
    m_indexSubsystem.stopIndex();
  }

  @Override
  public boolean isFinished() {
    return m_indexSubsystem.isBallDetected();
  }
}
