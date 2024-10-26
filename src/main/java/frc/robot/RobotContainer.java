// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.LEDConstants;

import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LaunchBallCommand;
import frc.robot.commands.ReverseIntakeCommand;
import frc.robot.commands.RevUpShooterCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;

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
  // private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();

  // Controllers
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_coDriverController = new CommandXboxController(OperatorConstants.kCoDriverControllerPort);

  private final SendableChooser<Command> m_autonChooser;

  private IntegerPublisher m_LEDIndexPublisher;
  private int m_LEDIndex;
  private int m_previousLEDIndex = 1;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure network tables to communicate with LEDs
    configureNetworkTables();

    // Configure the controller bindings
    configureBindings();

    // Register commands to pathplanner
    registerCommands();

    m_autonChooser = AutoBuilder.buildAutoChooser();

    Shuffleboard.getTab("Auton").add("Auton Selector", m_autonChooser);
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
    //m_coDriverController.leftTrigger(0.1).whileTrue(new RevUpShooterCommand(m_shooterSubsystem, m_coDriverController::getLeftTriggerAxis));
    m_coDriverController.leftTrigger(0.1)
      .onTrue(new InstantCommand(() -> m_shooterSubsystem.setPercent(0.7, 0.3), m_shooterSubsystem))
      .onFalse(new InstantCommand(() -> m_shooterSubsystem.stopShooter(), m_shooterSubsystem));

    // Configures the ball to launch when the right trigger is pressed.
    m_coDriverController.rightTrigger(0.3).whileTrue(new LaunchBallCommand(m_indexSubsystem));

    m_coDriverController.povUp().onTrue(new InstantCommand(() -> {
      m_previousLEDIndex = m_LEDIndex;
      m_LEDIndex = 0;
      m_LEDIndexPublisher.set(m_LEDIndex);
    }));

    m_coDriverController.povLeft().onTrue(new InstantCommand(() -> {
      m_LEDIndex = m_LEDIndex == 0 ? m_previousLEDIndex : m_LEDIndex;
      m_LEDIndex = m_LEDIndex <= 1 ? LEDConstants.maxLEDIndex : m_LEDIndex - 1;
    }));

    m_coDriverController.povRight().onTrue(new InstantCommand(() -> {
      m_LEDIndex = m_LEDIndex == 0 ? m_previousLEDIndex : m_LEDIndex;
      m_LEDIndex = m_LEDIndex >= LEDConstants.maxLEDIndex ? 1 : m_LEDIndex + 1;
    }));

    m_coDriverController.povDown().onTrue(new InstantCommand(() -> {
      m_LEDIndex = m_LEDIndex == 0 ? m_previousLEDIndex : m_LEDIndex;
      m_LEDIndexPublisher.set(m_LEDIndex);
    }));
  }

  private void configureNetworkTables() {
    NetworkTableInstance networkInstance = NetworkTableInstance.getDefault();
    NetworkTable table = networkInstance.getTable("LED Data");

    m_LEDIndexPublisher = table.getIntegerTopic("LED Image Index").publish();
  }

  /**
   * This method registers autonomous commands so that they can be used in pathplanner.
   * This is handled in a separate function to keep things organized.
   */
  private void registerCommands() {
    NamedCommands.registerCommand("Rev Up Shooter", new RepeatCommand(new InstantCommand(() -> m_shooterSubsystem.setPercent(0.7, 0.3), m_shooterSubsystem))
      .finallyDo(() -> m_shooterSubsystem.stopShooter())
    );
    NamedCommands.registerCommand("Shoot", new LaunchBallCommand(m_indexSubsystem).withTimeout(AutonomousConstants.kBallLaunchTimeout));
    NamedCommands.registerCommand("Intake", new IntakeCommand(m_intakeSubsystem, m_indexSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Run the command currently selected on shuffleboard
    return m_autonChooser.getSelected();
  }
}