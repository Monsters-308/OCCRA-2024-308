// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.IndexConstants;

public class IndexSubsystem extends SubsystemBase {
  private final TalonFX indexMotor = new TalonFX(IndexConstants.kIndexMotorChannel);  
  private final DigitalInput ballSensor = new DigitalInput(IndexConstants.kBallSensorPort);
  
  /**
   * Creates a new index subsystem.
   * It controls the mechanism that holds the ball in the robot before it gets shot.   
   **/
  public IndexSubsystem() {
    indexMotor.setInverted(IndexConstants.kIndexInverted);
    indexMotor.setNeutralMode(IndexConstants.kMotorIdleMode);

    Shuffleboard.getTab("Index").addBoolean("Is Ball Detected", this::isBallDetected);
  }

  /**
   * Gets the current speed of the indexer.
   */
  public double getIndexSpeed() {
    return indexMotor.get();
  }

  /**
   * Starts the index.
   * @param speed How fast the index motor should spin. Goes from -1 to 1, with -1 being full reverse and 1 being full forwards.
   */
  public void setIndexSpeed(double speed) {
    indexMotor.set(speed);
  }

  /**
   * Stops the indexer. This should be done after the ball is completly in the indexer.
   */
  public void stopIndex() {
    indexMotor.set(0);
  }
  
  /**
   * Detects whether the ball is over the sensor. True means it is in the sensor, and false means it is not.
   * */
  public boolean isBallDetected() {
    return !ballSensor.get();
  }
}
