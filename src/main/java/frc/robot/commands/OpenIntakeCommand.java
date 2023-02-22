// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class OpenIntakeCommand extends CommandBase {

  private static final double kForwardScaleFactor = 0.80;

  private final IntakeSubsystem m_IntakeSubsystem;
  private final DoubleSupplier m_forwardSpeedSupplier;
  /** Creates a new OpenIntakeCommand. */
  public OpenIntakeCommand(IntakeSubsystem IntakeSubsystem, DoubleSupplier forwardSpeedSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_IntakeSubsystem = IntakeSubsystem;
    m_forwardSpeedSupplier = forwardSpeedSupplier;
    addRequirements(m_IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double scaledForwardSpeed = m_forwardSpeedSupplier.getAsDouble() * kForwardScaleFactor;
    
    m_IntakeSubsystem.IntakeGo(scaledForwardSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
