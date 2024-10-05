// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;

import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LaunchBallCommand;
import frc.robot.commands.ReverseIntakeCommand;
import frc.robot.commands.RevUpShooterCommand;

import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final IndexSubsystem m_indexSubsystem = new IndexSubsystem();
  private final LEDSubsystem m_ledSubsystem = new LEDSubsystem();
  private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();

  // Controllers
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_coDriverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the controller bindings
    configureBindings();
  }

  /** 
   * This method maps controller inputs to commands. 
   * This is handled in a separate function to keep things organized. 
   * */
  private void configureBindings() {
    // Configures robot to drive with joystick inputs by default
    m_driveSubsystem.setDefaultCommand(
      new DriveCommand(m_driveSubsystem, m_driverController::getLeftY, m_driverController::getRightX)
    );

    // Configures intake to start when the b button is held on the Co-Driver Controller
    m_coDriverController.b().whileTrue(new IntakeCommand(m_intakeSubsystem, m_indexSubsystem));

    // Configures the intake to reverse when the x button is held on the Co-Driver Controller
    m_coDriverController.x().whileTrue(new ReverseIntakeCommand(m_intakeSubsystem, m_indexSubsystem));

    // Configures the shooter to rev up when the left trigger is held on the Co-Driver Controller.
    // The speed is controled by the analog input of the trigger.
    m_coDriverController.leftTrigger(0.1).whileTrue(new RevUpShooterCommand(m_shooterSubsystem, m_coDriverController::getLeftTriggerAxis));

    // Configures the ball to launch when the right trigger is pressed.
    m_coDriverController.rightTrigger(0.3).whileTrue(new LaunchBallCommand(m_indexSubsystem));


    // D-PADSTUFF :3

    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new InstantCommand();
  }
}
