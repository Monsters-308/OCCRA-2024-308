// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IndexSubsystem;

import edu.wpi.first.wpilibj2.command.Command;


public class LaunchBallCommand extends Command {
  private final IndexSubsystem m_indexSubsystem;

  public LaunchBallCommand(IndexSubsystem indexSubsystem) {
    m_indexSubsystem = indexSubsystem;
  }

  @Override
  public void initialize() {
    m_indexSubsystem.setIndexSpeed(1);
  }

  @Override
  public void end(boolean interrupted) {
    m_indexSubsystem.stopIndex();
  }
}