// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import 

public final class Autos {
  private static final double kBackupSpeed = 0.5;
  private static final double kBackupDuration = 2.5; // seconds

  public static CommandBase doNothing() {
    return null;
  }

  public static CommandBase justBackup(DriveSubsystem drive) {
    // Backup 11' 5" plus half the length of the robot. Aim for ~12'
    return new ParallelDeadlineGroup(
      new WaitCommand(kBackupDuration),
      new RunCommand(() -> drive.arcadeDrive(kBackupSpeed, 0), drive)
    );
  }

  public static CommandBase justDock(DriveSubsystem drive) {
    // Use vision to determine if we're far enough?
    // Or use distance, but the wheels might slip on the charge station
    // Backup 8' 2.5" minus half the length of the robot. Approx 70"  
    return null;
  }

  public static CommandBase dropAndGo(DriveSubsystem drive, ArmSubsystem arm, IntakeSubsystem intake) {
    if (arm == null || intake == null) {
      return justBackup(drive);
    }

    // Build a sequential command
    SequentialCommandGroup sequence = new SequentialCommandGroup();

    //sequence.addCommands(arm.raiseToTier1Command());
    //sequence.andThen(intake.openCommand());

    sequence.andThen(justBackup(drive));

    return sequence;
  }

  public static CommandBase dropAndDock(DriveSubsystem drive, ArmSubsystem arm, IntakeSubsystem intake) {
    if (arm == null || intake == null) {
      return justDock(drive);
    }

    // Build a sequential command
    SequentialCommandGroup sequence = new SequentialCommandGroup();

    //sequence.addCommands(arm.raiseToMiddleRowCommand());
    //sequence.andThen(intake.openCommand());

    sequence.andThen(justDock(drive));

    return sequence;
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
