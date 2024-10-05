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

  public RevUpShooterCommand(ShooterSubsystem shooterSubsystem, DoubleSupplier shooterSpeed) {
    m_shooterSubsystem = shooterSubsystem;
    getShooterSpeed = shooterSpeed;
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
