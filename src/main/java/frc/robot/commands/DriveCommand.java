// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.Utils;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class DriveCommand extends Command {
  
  private final DriveSubsystem m_subsystem;
  
  private final DoubleSupplier getLeftJoyStickInput;
  private final DoubleSupplier getRightJoyStickInput; 

  private final BooleanSupplier activateTurboMode;

  private final SlewRateLimiter speedSlewRateLimiter = new SlewRateLimiter(DriveConstants.kSpeedSlewRateLimit);
  private final SlewRateLimiter rotationalSlewRateLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRateLimit);

  /**
   * Default command for controlling the drivetrain with joystick input.
   * This command does not finish on its own.
   * 
   * @param subsystem The drive subsystem.
   * @param leftInput a supplier providing the values for the left joystick.
   * @param rightInput a supplier providing the values for the right joystick.
   */
  public DriveCommand(DriveSubsystem subsystem, DoubleSupplier leftInput, DoubleSupplier rightInput, BooleanSupplier turboMode) {
    m_subsystem = subsystem;
    getLeftJoyStickInput = leftInput;
    getRightJoyStickInput = rightInput;
    activateTurboMode = turboMode;
    
    // Use addRequirements() here to declare subsystem dependencies.
    // This makes it so that this command won't conflict with other commands
    // that use "subsystem".
    addRequirements(subsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.drive(
      // Speed
      speedSlewRateLimiter.calculate(
        Utils.sensitivityFunction(
          getLeftJoyStickInput.getAsDouble(),
          DriveConstants.kDriverSensitvity,
          DriveConstants.kDeadBand)
      ) * (activateTurboMode.getAsBoolean() ? 1 : DriveConstants.kMaxForwardSpeed),

      // Rotational 
      -rotationalSlewRateLimiter.calculate(
        Utils.sensitivityFunction(
          getRightJoyStickInput.getAsDouble(),
          DriveConstants.kRotationalSensitivity,
          DriveConstants.kDeadBand)
      ) * DriveConstants.kMaxTurningSpeed
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopDrive();
  }

}
