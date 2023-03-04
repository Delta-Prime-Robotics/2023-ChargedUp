// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeMoveCommand extends CommandBase {
  
  private static final double kForwardScaleFactor = 1.0;
 
  private final IntakeSubsystem m_IntakeSubsystem;
  private final DoubleSupplier m_forwardSpeedSupplier;


  /** Creates a new ArcadeDriveCommand. */
  public IntakeMoveCommand(IntakeSubsystem intake, DoubleSupplier forwardSpeedSupplier) {
    m_IntakeSubsystem = intake;
    m_forwardSpeedSupplier = forwardSpeedSupplier;
        
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Scale the control values so they're not as sensitive
    double scaledForwardSpeed = m_forwardSpeedSupplier.getAsDouble() * kForwardScaleFactor;
    
    m_IntakeSubsystem.IntakeGo(scaledForwardSpeed);
  }

  // Returns true when the command should end.
  @Override public boolean isFinished() {
    return false;
  }
}
