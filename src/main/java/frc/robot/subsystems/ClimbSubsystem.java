// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
  private final BaseTalon winchMotor = new BaseTalon(ClimbConstants.kWinchMotorID, "SRX");  
  private final BaseTalon climbMotor = new BaseTalon(ClimbConstants.kClimbMotorID, "SRX");  
  

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    winchMotor.setInverted(ClimbConstants.kWinchInverted);
    climbMotor.setInverted(ClimbConstants.kClimbInverted);
  }

  public void moveClimber(double speed){
    climbMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setWinch(double speed){
    winchMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
