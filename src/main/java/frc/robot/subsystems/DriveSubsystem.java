// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.RoboRio;
import frc.robot.Constants.RobotConfig;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.*;

public class DriveSubsystem extends SubsystemBase {
  
        // Drive constants
    private static final double kClosedLoopRampRate = 0.3;

    private static final double kCollisionThresholdDeltaG = 0.5;

    // Drive components
    private CANSparkMax m_leftLeader;
    private CANSparkMax m_leftFollower;  
    private CANSparkMax m_rightLeader;
    private CANSparkMax m_rightFollower;

    private DifferentialDrive m_diffDrive;

    private RelativeEncoder m_leftEncoder;
    public RelativeEncoder m_rightEncoder;

    private AHRS m_navx; // The NavX IMU (gyro)

    // Instance variable for collision detection
    private double m_lastLinearAccelY;
    
    //**Creates a drive subsystem */
    public DriveSubsystem() {
        m_leftLeader = new CANSparkMax(RoboRio.CanID.kLeftLeader, MotorType.kBrushless);
        m_leftFollower = new CANSparkMax(RoboRio.CanID.kLeftFollower, MotorType.kBrushless);
        m_rightLeader = new CANSparkMax(RoboRio.CanID.kRightLeader, MotorType.kBrushless);
        m_rightFollower = new CANSparkMax(RoboRio.CanID.kRightFollower, MotorType.kBrushless);

        m_leftFollower.follow(m_leftLeader);
        m_rightFollower.follow(m_rightLeader);

        m_rightLeader.setInverted(true);

        setClosedLoopRampRate(kClosedLoopRampRate);

        m_diffDrive = new DifferentialDrive(m_leftLeader,m_rightLeader);

        m_leftEncoder = m_leftLeader.getEncoder();
        m_rightEncoder = m_rightLeader.getEncoder();    
        
        m_leftEncoder.setPositionConversionFactor(RobotConfig.kDistancePerRotation);
        m_rightEncoder.setPositionConversionFactor(RobotConfig.kDistancePerRotation);

        try {
          m_navx = new AHRS(SPI.Port.kMXP);
          //SmartDashboard.putData(m_navx);
        }
        catch (RuntimeException ex) {
          DriverStation.reportError("Error instantiating navX MSP: " + ex.getMessage(), true);
        }
  }


  /**
   * Gets the distance traveled by the left motors, in inches.
   * @return Number of rotations of the motors, times the wheel circumference.
   */
  public double getLeftDistance() {
    return m_leftEncoder.getPosition();
  }

  /**
   * Gets the distance traveled by the right motors, in inches.
   * @return Number of rotations of the motors, times the wheel circumference.
   */
  public double getRightDistance() {
    return m_rightEncoder.getPosition();
  }

   /**
   * This command resets the left and right Encoders
   * 
   * @return a runOnce
   */
  public CommandBase resetEncoders() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
            m_leftEncoder.setPosition(0.0);
            m_rightEncoder.setPosition(0.0);
        });
  }

  /**
   * Controls the drive subsystem via arcade drive. 
   * The calculated values are squared to decrease sensitivity at low speeds. 
   * @param forwardSpeed The robot's speed along the X axis (-1.0 to 1.0). Forward is positive.
   * @param rotation The robot's rotation around the Z axis (-1.0 to 1.0). Clockwise is positive.
   */
  public void arcadeDrive(double forwardSpeed, double rotation) {

    forwardSpeed = applyLinearConstraints(forwardSpeed);
    rotation = applyAngularConstraints(rotation);

    SmartDashboard.putNumber("Rotation", rotation);

    m_diffDrive.arcadeDrive(forwardSpeed, rotation);
  }
  
  public void stop() {
    m_diffDrive.tankDrive(0, 0);
  }
  
  public void start(){
    m_diffDrive.tankDrive(0.5,0.5);
  }
  /**
   * Makes sure that the motor will run forward and backwards with the imput you have given.
   * @param forwardSpeed
   * @return the constrainted forwardSpeed
   */
  private double applyLinearConstraints(double forwardSpeed) {
    double result = forwardSpeed;

    final double kDeadzone = 0.05;
    final double kMinNeededToMove = 0.1;
    final double kSpeedLimit = 1;       // This should be a value <= 1.0

    double absSpeed = Math.abs(forwardSpeed);

    if (absSpeed < kDeadzone) {
      result = 0.0;
    }
    else if (absSpeed < kMinNeededToMove) {
      result = 0.1 * (forwardSpeed/Math.abs(forwardSpeed));
    }
    else if (absSpeed > kSpeedLimit)
    {
      result = kSpeedLimit * (forwardSpeed/Math.abs(forwardSpeed));
    }

    return result;
  }
  /**
   * Makes sure that the motor will run correctly with the imput you have given so you may rotate the robot.
   * @param forwardSpeed
   * @return the constrainted forwardSpeed
   */
  private double applyAngularConstraints(double rotation) {

    double result = rotation;

    final double kDeadzone = 0.06;
    final double kMinNeededToMove = 0.08;
    final double kSpeedLimit = 0.75;       // This should be a value <= 1.0

    double absSpeed = Math.abs(rotation);

    if (absSpeed < kDeadzone) {
      result = 0.0;
    }
    else if (absSpeed < kMinNeededToMove) {
      result = 0.1 * (rotation/Math.abs(rotation));
    }
    else if (absSpeed > kSpeedLimit)
    {
      result = kSpeedLimit * (rotation/Math.abs(rotation));
    }

    return result;
  }

  /**
   * An example method for how the gyro can be used to detect a collision along the Y axis. 
   * This could be used to stop the robot from moving if it encounters an obstacle.
   */
  public void detectCollision() {
    boolean collisionDetectedY = false;

    if (m_navx != null) {
      double currentLinearAccelY = m_navx.getWorldLinearAccelY();
      double currentJerkY = currentLinearAccelY - m_lastLinearAccelY;
      m_lastLinearAccelY = currentLinearAccelY;

      if (Math.abs(currentJerkY) > kCollisionThresholdDeltaG) {
        collisionDetectedY = true;
      }
    }

    SmartDashboard.putBoolean("Collision Detected Y", collisionDetectedY);
  }

  public void setClosedLoopRampRate(double rate) {
    this.m_leftLeader.setClosedLoopRampRate(rate);
    this.m_rightLeader.setClosedLoopRampRate(rate);
  }
  
  /**
   * Sets the idle mode for all motor controllers in the drive subsystem.
   * @param mode Idle mode (coast or brake).
   */
  public void setIdleMode(IdleMode mode) {
		m_leftLeader.setIdleMode(mode);
		m_leftFollower.setIdleMode(mode);
		m_rightLeader.setIdleMode(mode);
		m_rightFollower.setIdleMode(mode);
	}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // detectCollision();

    // SmartDashboard.putNumber("Left Encoder Distance", m_leftEncoder.getPosition());
    // SmartDashboard.putNumber("Right Encoder Distance", m_rightEncoder.getPosition());
    // SmartDashboard.putNumber("Left Drive Motor Speed", m_leftLeader.get());
    // SmartDashboard.putNumber("Right Drive Motor Speed", m_rightLeader.get());
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("photonvision/Microsoft_LifeCam_HD-3000");
    SmartDashboard.putBoolean("NT-hasTarget", table.getEntry("hasTarget").getBoolean(false));

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
