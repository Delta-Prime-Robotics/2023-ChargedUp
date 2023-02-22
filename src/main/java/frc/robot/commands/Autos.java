// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
public final class Autos {
  
  private static final double kBackupSpeed = 0.5;
  private static final double kBackupDuration = 2.5; // seconds
  private static final double kOpenIntakeSpeed = 0.5;
  private static final double kOpenIntakeDuration = 2;
 
 
 
  public static CommandBase doNothing() {
    return null;
  }

  public static CommandBase justBackup(DriveSubsystem drive) {
    // Backup 11' 5" plus half the length of the robot. Aim for ~12'
    return new ParallelDeadlineGroup(
      new WaitCommand(kBackupDuration),
      new RunCommand(() -> drive.arcadeDrive(kBackupSpeed, 0),drive)
      

    );
  }

  public static CommandBase openIntake(IntakeSubsystem intake) {
    return new ParallelDeadlineGroup(
      new WaitCommand(2),
      new RunCommand(() -> intake.IntakeGo(kOpenIntakeSpeed), intake)
      
      
    );
  }

  public static CommandBase openIntakeStop(IntakeSubsystem intake) {
    return new ParallelDeadlineGroup(
      new WaitCommand(3),
      new RunCommand(() -> intake.IntakeGo(0), intake)
    );
  }

  public static CommandBase justCharge(DriveSubsystem drive) {
    // Use vision to determine if we're far enough?
    // Or use distance, but the wheels might slip on the charge station
    // Backup 8' 2.5" minus half the length of the robot. Approx 70"  
    return null;
  }

  public static CommandBase armMove(ArmSubsystem arm) {
    return new ParallelDeadlineGroup(
      new WaitCommand(2),
      new RunCommand(() -> arm.ArmGo(0.5), arm)
    );
  }

  public static CommandBase armStop(ArmSubsystem arm) {
    return new ParallelDeadlineGroup(
      new WaitCommand(2), 
      new RunCommand(() -> arm.ArmGo(0), arm)
    );
  }

  public static CommandBase armEncoder(ArmSubsystem arm) {
    return new ParallelDeadlineGroup(
      new WaitCommand(0),
      new RunCommand(() -> arm.resetEncoders(), arm)
    );
  }
  public static CommandBase dropAndBackUp(DriveSubsystem drive, ArmSubsystem arm, IntakeSubsystem intake) {
    // if (arm == null || intake == null) {
    //   return justBackup(drive);
    // }

    // Build a sequential command
    SequentialCommandGroup sequence = new SequentialCommandGroup();
    // Raise to arm to tier 1
    // Then open intake
    //Then BackUp
    // sequence.addCommands(arm.raiseToTier1Command());
    // sequence.andThen(intake.openCommand());
    //sequence.andThen(justBackup(drive));
    sequence.addCommands(armMove(arm));
    sequence.addCommands(armStop(arm));
    sequence.addCommands(openIntake(intake));
    sequence.addCommands(openIntakeStop(intake));
    sequence.addCommands(justBackup(drive));
    

    return sequence;
  }

  

  public static CommandBase dropAndCharge(DriveSubsystem drive, ArmSubsystem arm, IntakeSubsystem intake) {
    if (arm == null || intake == null) {
      return justCharge(drive);
    }

    // Build a sequential command
    SequentialCommandGroup sequence = new SequentialCommandGroup();

    //sequence.addCommands(arm.raiseToMiddleRowCommand());
    //sequence.andThen(intake.openCommand());
    //sequence.andThen(justDock(drive));
    //sequence.addCommands(justDock(drive));

    return sequence;
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
