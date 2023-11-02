// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.RoboRio;

public class ArmPIDSubsystem extends ProfiledPIDSubsystem {
  /** Creates a new ArmPIDSubsystem. */
  
  //arm motors
  private CANSparkMax m_armLeader;
  private CANSparkMax m_armFollower;
  private MotorControllerGroup m_MotorControllerGroup;


  //Defineing the arm encoder
  public RelativeEncoder m_armEncoder;
  public RelativeEncoder m_armFollowerEncoder;
 
  
  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          ArmConstants.kSVolts, ArmConstants.kGVolts,
          ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);

  public ArmPIDSubsystem() {
    super(
      // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            0,
            0,
            0,
            // The motion profile constraints
          new TrapezoidProfile.Constraints(0, 0)));
    
    m_armLeader = new CANSparkMax(RoboRio.CanID.kArmLeader, MotorType.kBrushless);
    m_armFollower = new CANSparkMax(RoboRio.CanID.kArmFollower, MotorType.kBrushless);
    m_armFollower.setInverted(true);
    m_MotorControllerGroup = new MotorControllerGroup(m_armLeader, m_armFollower);

    // Need  value for Arm Encoder in Constents
    m_armEncoder = m_armLeader.getEncoder();
    m_armFollowerEncoder = m_armFollower.getEncoder();
    m_armEncoder.setPosition(0);
    m_armFollowerEncoder.setPosition(0);

  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }
}
