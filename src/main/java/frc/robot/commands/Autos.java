// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
public final class Autos {
  
  private static final double kBackupSpeed =  -0.5;
  private static final double kBackupDuration = 2; // seconds
  private static final double kOpenIntakeSpeed = 0.5;
  private static final double kOpenIntakeDuration = 2;
 
  public static CommandBase doNothing() {
    return null;
  }
  public final static Boolean driveEncoderSupplier(DriveSubsystem drive) {
    
    if (drive.m_rightEncoder.getPosition() > SmartDashboard.getNumber("Driver Encoder", 500)) {
      return true;
    }
    else {
      return false;
    }
  };

  public static CommandBase justBackup(DriveSubsystem drive, BooleanSupplier driveEncoderSupplier) {
    // Backup 11' 5" plus half the length of the robot. Aim for ~12'
    return new ParallelDeadlineGroup(
      new WaitUntilCommand(driveEncoderSupplier),
      new RunCommand(() -> drive.arcadeDrive(kBackupSpeed, 0),drive)
      

    );
  }
  public static CommandBase justCharge(DriveSubsystem drive) {
    // Use vision to determine if we're far enough?
    // Or use distance, but the wheels might slip on the charge station
    // Backup 8' 2.5" minus half the length of the robot. Approx 70"  
    return null;
  }

  private final static boolean intakeEncoderSupplier(IntakeSubsystem intake) {
    SmartDashboard.putNumber("Intake Encoder",intake.m_intakeEncoder.getPosition());
    if (intake.m_intakeEncoder.getPosition() > 100) {
      return true;
    }
    else {
      return false;
    }
  };

  public static CommandBase intakeMove(IntakeSubsystem intake, BooleanSupplier intakeEncoderSupplier) {
    return new ParallelDeadlineGroup(
      new WaitUntilCommand(intakeEncoderSupplier),
      new RunCommand(()-> intake.IntakeGo(0.5), intake)
    );
  }

  public static CommandBase intakeStop(IntakeSubsystem intake) {
    return new InstantCommand(() -> intake.IntakeGo(0), intake);
  }

 
  private final static boolean armEncoderSupplier(ArmSubsystem arm) {
    SmartDashboard.putNumber("Arm Encoder",arm.m_armEncoder.getPosition());
    if (arm.m_armEncoder.getPosition() > 100) {
      return true;
    }
    else {
      return false;
    }
  };

  public static CommandBase armMove(ArmSubsystem arm, BooleanSupplier armEncoderSupplier) {
    return new ParallelDeadlineGroup(
      new WaitUntilCommand(armEncoderSupplier),
      new RunCommand(()-> arm.ArmGo(0.5), arm)
    );
  }

  public static CommandBase armStop(ArmSubsystem arm) {
    return new InstantCommand(() -> arm.ArmGo(0), arm);
    
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
    //sequence.andThen(justBackup(drive));\
    sequence.addCommands(arm.resetEncoders());
    sequence.addCommands(intake.resetEncoders());
    sequence.addCommands(armMove(arm, () -> armEncoderSupplier(arm) ));
    sequence.addCommands(armStop(arm));
    sequence.addCommands(intakeMove(intake, () -> intakeEncoderSupplier(intake)));
    sequence.addCommands(intakeStop(intake));
    sequence.addCommands(justBackup(drive, () -> driveEncoderSupplier(drive)));
    

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
