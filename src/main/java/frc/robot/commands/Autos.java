// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.BalanceState;
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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
public final class Autos {
  
  private static final double kBackupSpeed =  -0.5;
  private static final double kBackupDuration = 3; // seconds
  private static final double kOpenIntakeSpeed = 0.5;
  private static final double kOpenIntakeDuration = 2;
  public static final double kJustBackUpEncoder = -10000;
  private static final double kArmRaiseMid = -100;
  private static final double kIntakeOpen = 100;
  private static final double kChargeDuration = 2.15;

  public static CommandBase doNothing() {
    return null;
  }
  public final static Boolean driveEncoderSupplier(DriveSubsystem drive, double encoder) {
    
    if (Math.abs(drive.m_rightEncoder.getPosition()) > 10000 ) {
      return true;
    }
    else {
      return false;
    }
  };

  public final static Boolean chargeSupplier(DriveSubsystem drive, int bState) {
    double pitch = drive.m_navx.getPitch();
    boolean result = false;
    if ((bState == BalanceState.kUp) && (pitch > 1)) {
      result = true;
    }
    else if ((bState == BalanceState.kDown) && (pitch < -1)) {
      result = true;
    }

    else if ((bState == BalanceState.kLevel) && (Math.abs(pitch) < 0.5)) {
      result = true;
    }

    return result;
  }

  public static CommandBase justBackup(DriveSubsystem drive, BooleanSupplier driveEncoderSupplier) {
    // Backup 11' 5" plus half the length of the robot. Aim for ~12'
    drive.m_leftEncoder.setPosition(0.0);
    drive.m_rightEncoder.setPosition(0.0);
    return new ParallelDeadlineGroup(
      new WaitUntilCommand(driveEncoderSupplier),
      new RunCommand(() -> drive.arcadeDrive(kBackupSpeed, 0),drive)
      

    );
  }
  public static CommandBase justChargeAuto(DriveSubsystem drive) {
    return new ParallelDeadlineGroup(
      new WaitUntilCommand(() -> chargeSupplier(drive,BalanceState.kUp)),
      new RunCommand(() -> drive.arcadeDrive(kBackupSpeed, 0),drive)
    );
  }

  public static CommandBase justCharge(DriveSubsystem drive) {
    return new ParallelDeadlineGroup(
      new WaitCommand(kChargeDuration),
      new RunCommand(() -> drive.arcadeDrive(kBackupSpeed, 0),drive)
    );
  }

  private final static boolean intakeEncoderSupplier(IntakeSubsystem intake, double encoder) {
    SmartDashboard.putNumber("Intake Encoder",intake.m_intakeEncoder.getPosition());
    if (intake.m_intakeEncoder.getPosition() > encoder) {
      return true;
    }
    else {
      return false;
    }
  };

  public static CommandBase intakeMove(IntakeSubsystem intake, BooleanSupplier intakeEncoderSupplier) {
    return new ParallelDeadlineGroup(
      new WaitCommand(0.8),
      new RunCommand(()-> intake.IntakeGo(-0.5), intake)
    );
  }

  public static CommandBase intakeBack(IntakeSubsystem intake, BooleanSupplier intakeEncoderSupplier) {
    return new ParallelDeadlineGroup(
      new WaitCommand(0.8),
      new RunCommand(()-> intake.IntakeGo(0.5), intake)
    );
  }
  public static CommandBase intakeStop(IntakeSubsystem intake) {
    return new InstantCommand(() -> intake.IntakeGo(0), intake);
  }

 
  private final static boolean armEncoderSupplier(ArmSubsystem arm, double encoder) {
    SmartDashboard.putNumber("Arm Encoder",arm.m_armEncoder.getPosition());
    if (arm.m_armEncoder.getPosition() > encoder) {
      return true;
    }
    else {
      return false;
    }
  };

  public static CommandBase armforward(ArmSubsystem arm, BooleanSupplier armEncoderSupplier) {
    return new ParallelDeadlineGroup(
      new WaitCommand(2),
      new RunCommand(()-> arm.ArmGo(-0.5), arm)
    );
  }

  public static CommandBase armback(ArmSubsystem arm, BooleanSupplier armEncoderSupplier) {
    return new ParallelDeadlineGroup(
      new WaitCommand(1.8),
      new RunCommand(()-> arm.ArmGo(0.5), arm)
    );
  }

  public static CommandBase armStop(ArmSubsystem arm) {
    return new InstantCommand(() -> arm.ArmGo(0), arm);
    
  }
  
  public static CommandBase drop(ArmSubsystem arm, IntakeSubsystem intake) {

    SequentialCommandGroup sequence = new SequentialCommandGroup();

    sequence.addCommands(arm.resetEncoders());
    sequence.addCommands(intake.resetEncoders());
    sequence.addCommands(armforward(arm, () -> armEncoderSupplier(arm, kArmRaiseMid)));
    sequence.addCommands(armStop(arm));
    sequence.addCommands(intakeMove(intake, () -> intakeEncoderSupplier(intake,kIntakeOpen)));
    sequence.addCommands(intakeStop(intake));
    sequence.addCommands(intakeBack(intake, () -> intakeEncoderSupplier(intake,kIntakeOpen)));
    sequence.addCommands(intakeStop(intake));
    sequence.addCommands(armback(arm, () -> armEncoderSupplier(arm, kArmRaiseMid)));
    sequence.addCommands(armStop(arm));
  

    return sequence;
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
    sequence.addCommands(drop(arm, intake));
    sequence.addCommands(justBackup(drive, () -> driveEncoderSupplier(drive, kJustBackUpEncoder)));
    

    return sequence;
  }

  public static CommandBase dropAndCharge(DriveSubsystem drive, ArmSubsystem arm, IntakeSubsystem intake) {
    

    // Build a sequential command
    SequentialCommandGroup sequence = new SequentialCommandGroup();

    sequence.addCommands(drop(arm, intake));
    sequence.addCommands(justCharge(drive));

    return sequence;
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
