// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexConstants;

public class ClimbSubsystem extends SubsystemBase {
  private final BaseTalon winchMotor = new BaseTalon(IndexConstants.kIndexMotorChannel, "SRX");  
  private final DigitalInput climbMotor = new DigitalInput(IndexConstants.kBallSensorPort);

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    
  }

  public void start(){
    
  }

  public void stop(){
    
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
