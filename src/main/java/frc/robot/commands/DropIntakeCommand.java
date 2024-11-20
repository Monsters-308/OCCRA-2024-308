// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DropIntakeCommand extends SequentialCommandGroup {
  
    public DropIntakeCommand(DriveSubsystem driveSubsystem) {
        addCommands(
            new InstantCommand(() -> driveSubsystem.setPercent(1), driveSubsystem),
            new WaitCommand(0.1),
            new InstantCommand(() -> driveSubsystem.stopDrive(), driveSubsystem)
        );
    }

}
