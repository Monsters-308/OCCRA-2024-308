// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IndexSubsystem;

import edu.wpi.first.wpilibj2.command.Command;


public class IntakeCommand extends Command {
  
  private final IndexSubsystem m_subsystem;

  /**
   * Default command for controlling the drivetrain with joystick input.
   * This command does not finish on its own.
   * 
   * @param subsystem The drive subsystem.
   */
  public IndexCommand(IndexSubsystem subsystem) {
    m_subsystem = subsystem;
    
    // Use addRequirements() here to declare subsystem dependencies.
    // This makes it so that this command won't conflict with other commands
    // that use "subsystem".
    addRequirements(subsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void initialize() {
    m_subsystem.setIndexSpeed(1);
  }

  @Override
  public void execute() {
    if (m_subsystem.isBallDetected()) {
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopIndex();
  }
  
} 
