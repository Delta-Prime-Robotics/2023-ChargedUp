// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ScaledArcadeDriveCommand extends CommandBase {
  
  private static final double kForwardScaleFactor = 0.80;
  private static final double kRotationScaleFactor = 0.70;

  private final DriveSubsystem m_driveSubsystem;
  private final DoubleSupplier m_forwardSpeedSupplier;
  private final DoubleSupplier m_rotationSupplier;

  /** Creates a new ArcadeDriveCommand. */
  public ScaledArcadeDriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier forwardSpeedSupplier, DoubleSupplier rotationSupplier) {
    m_driveSubsystem = driveSubsystem;
    m_forwardSpeedSupplier = forwardSpeedSupplier;
    m_rotationSupplier = rotationSupplier;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Scale the control values so they're not as sensitive
    double scaledForwardSpeed = m_forwardSpeedSupplier.getAsDouble() * kForwardScaleFactor;
    double scaledRotation = m_rotationSupplier.getAsDouble() * kRotationScaleFactor;

    m_driveSubsystem.arcadeDrive(scaledForwardSpeed, scaledRotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
