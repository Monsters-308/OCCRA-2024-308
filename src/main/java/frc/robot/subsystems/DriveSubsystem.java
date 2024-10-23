// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.ControlType;

import java.util.Map;
import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.utils.Utils;

public class DriveSubsystem extends SubsystemBase {
  private final CANSparkMax leftFrontMotor = new CANSparkMax(DriveConstants.kLeftFrontMotorCANID, MotorType.kBrushless);
  private final CANSparkMax leftBackMotor = new CANSparkMax(DriveConstants.kLeftBackMotorCANID, MotorType.kBrushless);
  private final CANSparkMax rightFrontMotor = new CANSparkMax(DriveConstants.kRightFrontMotorCANID, MotorType.kBrushless);
  private final CANSparkMax rightBackMotor = new CANSparkMax(DriveConstants.kRightBackMotorCANID, MotorType.kBrushless);

  // encoders
  private final RelativeEncoder leftFrontEncoder = leftFrontMotor.getEncoder();
  private final RelativeEncoder leftBackEncoder = leftBackMotor.getEncoder();
  private final RelativeEncoder rightFrontEncoder = rightFrontMotor.getEncoder();
  private final RelativeEncoder rightBackEncoder = rightBackMotor.getEncoder();

  private final RelativeEncoder[] Encoders = new RelativeEncoder[]{leftFrontEncoder, leftBackEncoder, rightFrontEncoder, rightBackEncoder};

  // PID controllers
  private final SparkPIDController leftFrontPID = leftFrontMotor.getPIDController();
  private final SparkPIDController leftBackPID = leftBackMotor.getPIDController();
  private final SparkPIDController rightFrontPID = rightFrontMotor.getPIDController();
  private final SparkPIDController rightBackPID = rightBackMotor.getPIDController();

  private final SparkPIDController[] PIDControllers = new SparkPIDController[]{leftFrontPID, leftBackPID, rightFrontPID, rightBackPID};

  // The gyro sensor
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  // Odometry object: responsible for estimating the robot's position.
  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(
    Rotation2d.fromDegrees(getGyroAngle()), 
    getLeftPosition(), 
    getRightPosition()
  );

  private final ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");

  // Shuffleboard widget to control mirroring autons
  private final SimpleWidget mirrorAuton = driveTab.add("Mirror auton", false)
    .withWidget(BuiltInWidgets.kToggleSwitch);

  // Field widget for displaying odometry
  private final Field2d m_field = new Field2d();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    
    applyAllMotors(motor -> {
      // Reset factory defaults
      motor.restoreFactoryDefaults();
      // "smart limit"
      motor.setSmartCurrentLimit(DriveConstants.kSmartCurrentLimit);
      // brake gota go slow 3:
      motor.setIdleMode(DriveConstants.kMotorIdleMode);
    });
   
    // invert settings AAAAAAAAAAAAAAAAH
    leftFrontMotor.setInverted(DriveConstants.kLeftFrontMotorInverted);
    rightFrontMotor.setInverted(DriveConstants.kRightFrontMotorInverted);
    leftBackMotor.setInverted(DriveConstants.kLeftBackMotorInverted);
    rightBackMotor.setInverted(DriveConstants.kRightBackMotorInverted);

    // set encoder feedback
    leftFrontPID.setFeedbackDevice(leftFrontEncoder);
    leftBackPID.setFeedbackDevice(leftBackEncoder);
    rightFrontPID.setFeedbackDevice(rightFrontEncoder);
    rightBackPID.setFeedbackDevice(rightBackEncoder);

    for (RelativeEncoder encoder : Encoders) {
      encoder.setPositionConversionFactor(DriveConstants.kEncoderConversionFactor); // meters
      encoder.setVelocityConversionFactor(DriveConstants.kEncoderConversionFactor / 60.0); // meters per second
    }

    for (SparkPIDController PIDController : PIDControllers) {
      PIDController.setP(DriveConstants.kVelocityP);
      PIDController.setI(DriveConstants.kVelocityI);
      PIDController.setD(DriveConstants.kVelocityD);
      PIDController.setFF(DriveConstants.kVelocityFF);
    }

    applyAllMotors(motor -> motor.burnFlash());

    /* Shuffleboard Configuration */

    // Gyro widget
    driveTab.addDouble("Robot Heading", () -> getHeading().getDegrees())
      .withWidget(BuiltInWidgets.kGyro)
      .withSize(2, 2)
      .withProperties(Map.of(
        "Counter Clockwise", true));
    
    // Field widget for displaying odometry estimation
    driveTab.add("Field", m_field)
      .withSize(6, 3);
    
    driveTab.addDouble("X pos", () -> getPose().getX());
    driveTab.addDouble("Y pos", () -> getPose().getY());

    // For testing purposes
    driveTab.addDouble("Encoder error", () -> leftFrontEncoder.getPosition() - leftBackEncoder.getPosition());
    driveTab.addDouble("Gyro error", () -> Units.degreesToRadians(gyro.getRate()) - getChassisSpeeds().omegaRadiansPerSecond);
    driveTab.addDouble("Left Speed", this::getLeftVelocity);
    driveTab.addDouble("Right Speed", this::getRightVelocity);
    driveTab.addDouble("Rotational Speed", () -> getChassisSpeeds().omegaRadiansPerSecond);
    driveTab.add("Max Speed", DriveConstants.kMaxSpeedMetersPerSecond);
    driveTab.add("Max Rotation", DriveConstants.kMaxAngularSpeed);

    /* Pathplanner Configuration */

    AutoBuilder.configureRamsete(
      this::getPose, 
      this::resetOdometry, 
      this::getChassisSpeeds, 
      this::drive, 
      new ReplanningConfig(), 
      () -> mirrorAuton.getEntry().getBoolean(false), 
      this
    );
  }

  /**
   * Drives the robot.
   * @param forwardSpeed How fast to move forwards/backwards. -1 is full reverse; 1 is full forwards.
   * @param turningSpeed How fast it turns. -1 is full left; 1 is full right.
   */
  public void drive(double forwardSpeed, double turningSpeed) {
    // calculate speeds
    double leftSpeed = forwardSpeed + turningSpeed;
    double rightSpeed = forwardSpeed - turningSpeed;

    // normalize speeds
    double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
    if (max > 1) {
      leftSpeed /= max;
      rightSpeed /= max;
    }

    // write speeds to motors
    leftFrontPID.setReference(DriveConstants.kMaxSpeedMetersPerSecond * leftSpeed, ControlType.kVelocity);
    leftBackPID.setReference(DriveConstants.kMaxSpeedMetersPerSecond * leftSpeed, ControlType.kVelocity);
    rightFrontPID.setReference(DriveConstants.kMaxSpeedMetersPerSecond * rightSpeed, ControlType.kVelocity);
    rightBackPID.setReference(DriveConstants.kMaxSpeedMetersPerSecond * rightSpeed, ControlType.kVelocity);
  }

  /**
   * Drives the robot.
   * @param speeds The desired translational and rotation speeds.
   */
  public void drive(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = DriveConstants.kDriveKinematics.toWheelSpeeds(speeds);

    // write speeds to motors
    leftFrontPID.setReference(wheelSpeeds.leftMetersPerSecond, ControlType.kVelocity);
    leftBackPID.setReference(wheelSpeeds.leftMetersPerSecond, ControlType.kVelocity);
    rightFrontPID.setReference(wheelSpeeds.rightMetersPerSecond, ControlType.kVelocity);
    rightBackPID.setReference(wheelSpeeds.rightMetersPerSecond, ControlType.kVelocity);
  }

  /**
   * Stops the robot by setting all the motors to 0% power.
   */
  public void stopDrive() {
    applyAllMotors((motor) -> motor.set(0));
  }

  /**
   * Sets the drive motors to a specified percentage.
   * THIS IS FOR TESTING PURPOSES.
   * @param speed The drivetrain speed from -1 to 1.
   */
  public void setPercent(double speed) {
    applyAllMotors((motor) -> motor.set(speed));
  }

  /**
   * Applies a function to every drive motor.
   * @param function This function is called for every motor. It passes one CANSparkMax object into the function.
   */
  private void applyAllMotors(Consumer<CANSparkMax> function) {
    CANSparkMax[] motors = {leftFrontMotor, leftBackMotor, rightBackMotor, leftFrontMotor};

    for (int i = 0; i < motors.length; i++) {
      function.accept(motors[i]);
    }
  }

  /**
   * Calculates how far the left side of the robot has traveled
   * using the average position of the two left side encoders.
   * @return How far the left side has traveled in meters.
   */
  public double getLeftPosition() {
    return (leftFrontEncoder.getPosition() + leftBackEncoder.getPosition()) / 2.0;
  }

  /**
   * Returns the speed of the left side of the robot.
   * @return The wheel speed in meters per second.
   */
  public double getLeftVelocity(){
    return (leftFrontEncoder.getVelocity() + leftBackEncoder.getVelocity()) / 2;
  }

  /**
   * Calculates how far the right side of the robot has traveled
   * using the average position of the two right side encoders.
   * @return How far the right side has traveled in meters.
   */
  public double getRightPosition() {
    return (rightFrontEncoder.getPosition() + rightBackEncoder.getPosition()) / 2.0;
  }

  /**
   * Returns the speed of the right side of the robot.
   * @return The wheel speed in meters per second.
   */
  public double getRightVelocity() {
    return (rightFrontEncoder.getVelocity() + rightBackEncoder.getVelocity()) / 2;
  }

  /**
   * Returns the gyro's angle adjusted for inversion.
   * @apiNote This may not be the same as getHeading() and is not constrained from -180 to 180.
   * @return The angle of the gyro in degrees.
   */
  private double getGyroAngle() {
    return gyro.getAngle() * (DriveConstants.kInvertGyro ? -1.0 : 1.0);
  } 

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading, constrained from -180 to 180 degrees.
   */
  public Rotation2d getHeading() {
    return Utils.constrainAngle(
      odometry.getPoseMeters().getRotation()
    );
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose in meters.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }
  
  /**
   * Returns the translational and angular speed of the robot.
   * @return The chassis speeds.
   */
  public ChassisSpeeds getChassisSpeeds(){
    // NOTE: currently this is using the wheel speeds in order to 
    // estimate the robot's rotational speed. However, the gyro
    // can also calculate the rotational speed of the robot, and 
    // that might be more accurate than using the encoders.
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
      new DifferentialDriveWheelSpeeds(
        getLeftVelocity(),
        getRightVelocity()
      ));
  }

  /**
   * Resets the odometry to the specified pose. Note: this also resets the angle of the robot.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
      Rotation2d.fromDegrees(getGyroAngle()), 
      getLeftPosition(), 
      getRightPosition(), 
      pose
    );
  }
  
  /**
   * This method runs automatically every scheduler run. It shouldn't be run manually.
   */
  @Override
  public void periodic() {
    // Update pose estimation with odometry data
    odometry.update(
      Rotation2d.fromDegrees(getGyroAngle()), 
      getLeftPosition(), 
      getRightPosition()
    );

    // Update field widget
    m_field.setRobotPose(getPose());
  }
}
