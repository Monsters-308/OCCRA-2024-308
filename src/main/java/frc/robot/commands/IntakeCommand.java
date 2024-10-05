// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;



public class IntakeCommand extends Command {
  
  private final IntakeSubsystem m_intakeSubsystem;
  private final IndexSubsystem m_indexSubsystem;

  /**
   * Creates a new IntakeCommand.
   * This command controls how the ball get intaked into the robot.
   * This command will finnish once the ball is detected in the robot.
   * 
   * @param intakeSubsystem the intake subsystem
   * @param indexSubsystem the index subsystem
   */
  public IntakeCommand(IntakeSubsystem intakeSubsystem, IndexSubsystem indexSubsystem) {
    m_intakeSubsystem = intakeSubsystem;
    m_indexSubsystem = indexSubsystem;
    
    // Use addRequirements() here to declare subsystem dependencies.
    // This makes it so that this command won't conflict with other commands
    // that use "subsystem".
    addRequirements(intakeSubsystem, indexSubsystem);
  }

  // Called when the command starts
  @Override
  public void initialize() {
    m_intakeSubsystem.setIntakeSpeed(1);
    m_indexSubsystem.setIndexSpeed(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stopIntake();
    m_indexSubsystem.stopIndex();
  }

  @Override
  public boolean isFinished() {
    return m_indexSubsystem.isBallDetected();
  }
  
} 
