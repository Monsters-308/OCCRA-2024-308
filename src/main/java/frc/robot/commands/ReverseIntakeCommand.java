// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class ReverseIntakeCommand extends Command {
  
  private final IntakeSubsystem m_intakeSubsystem;
  private final IndexSubsystem m_indexSubsystem;

  /**
   * Reverses the intake subsystems to remove a ball or other object from the robot without shooting it
   * This command finnishes when the b button stops being held on the Co-Driver Controller
   * 
   * @param intakeSubsystem the intake subsystem
   * @param indexSubsystem the index subsystem
   */
  public ReverseIntakeCommand(IntakeSubsystem intakeSubsystem, IndexSubsystem indexSubsystem) {
    m_intakeSubsystem = intakeSubsystem;
    m_indexSubsystem = indexSubsystem;
    
    
    // Use addRequirements() here to declare subsystem dependencies.
    // This makes it so that this command won't conflict with other commands
    // that use "subsystem".
    addRequirements(intakeSubsystem, indexSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void initialize() {
    m_intakeSubsystem.setIntakeSpeed(-0.7);
    m_indexSubsystem.setIndexSpeed(-0.7);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stopIntake();
    m_indexSubsystem.stopIndex();
  }
} 
