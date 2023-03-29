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
  
  private static final double kBackupSpeed =  0.5;
  private static final double kForwardSpeed =  0.5;
  private static final double kJustBackUpTime = 3.4;
  private static final double kArmForwardSpeed =  0.5;
  private static final double kArmBackwardsSpeed =  -0.5;
  private static final double kIntakeForwardSpeed =  -0.5;
  private static final double kIntakeBackwardsSpeed =  0.5;
  private static final double kOpenAndCloseIntakeDuration = 1;
  public static final double kJustBackUpEncoder = -10000;
  private static final double kArmRaiseMid = -100;
  private static final double kArmRaiseMidTime = 2;
  private static final double kArmCloseMidTime = 1.8;
  private static final double kIntakeOpen = 100;
  private static final double kChargeDuration = 2.15;
  private static final double k180Rotation = 180;
  private static final double k90Rotation = 90;
  private static final double kPitchUp = 10;
  private static final double kPitchDown = -10;
  private static final double KPitchLevel = 3;  // this may need to be changed to something smaller? 

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

  private final static boolean intakeEncoderSupplier(IntakeSubsystem intake, double encoder) {
    SmartDashboard.putNumber("Intake Encoder",intake.m_intakeEncoder.getPosition());
    if (intake.m_intakeEncoder.getPosition() > encoder) {
      return true;
    }
    else {
      return false;
    }
  };

  public final static Boolean chargeSupplier(DriveSubsystem drive, int bState) {
    double pitch;
    boolean result = false;
    if (null != drive.m_navx) 
      pitch = drive.m_navx.getRoll();
    else 
      return false;
    
    if ((bState == BalanceState.kUp) && (pitch > kPitchUp)) {
      result = true;
    }
    else if ((bState == BalanceState.kDown) && (pitch < kPitchDown)) {
      result = true;
    }

    else if ((bState == BalanceState.kLevel) && (Math.abs(pitch) < KPitchLevel)) {
      result = true;
    }

    return result;
  }

  public final static Boolean rotationSupplier(DriveSubsystem drive, double rotation) {
    double yaw;
    boolean result = false;

    if(null != drive.m_navx)
      yaw = drive.m_navx.getYaw();
    else
      return false;

    if (yaw > rotation) {
      result = true;
    }

    return result;
  }

  public static CommandBase justBackup(DriveSubsystem drive, BooleanSupplier driveEncoderSupplier) {
    // Backup 11' 5" plus half the length of the robot. Aim for ~12'
    drive.m_leftEncoder.setPosition(0.0);
    drive.m_rightEncoder.setPosition(0.0);
    return new ParallelDeadlineGroup(
      new WaitCommand(kJustBackUpTime),
      new RunCommand(() -> drive.arcadeDrive(kBackupSpeed, 0),drive)
    );
  }

  public static CommandBase rotate180(DriveSubsystem drive) {
  double rotation = drive.m_navx.getYaw() + k180Rotation;
  return new ParallelDeadlineGroup(
      //new WaitUntilCommand(() -> rotationSupplier(drive, rotation)).withTimeout(2),
      new WaitCommand(1.15),
      new RunCommand(() -> drive.arcadeDrive(0, 0.5),drive)
    );
  }

  public static CommandBase rotate90(DriveSubsystem drive) {
    double rotation = drive.m_navx.getYaw() + k90Rotation;
    return new ParallelDeadlineGroup(
        //new WaitUntilCommand(() -> rotationSupplier(drive, rotation)).withTimeout(2),
        new WaitCommand(0.57),
        new RunCommand(() -> drive.arcadeDrive(0, 0.5),drive)
      );
    }
  
  public static CommandBase moblityOverChargeStation(DriveSubsystem drive) {

    SequentialCommandGroup sequence = new SequentialCommandGroup();

    ParallelDeadlineGroup goOver = new ParallelDeadlineGroup(
      new WaitUntilCommand(() -> chargeSupplier(drive,BalanceState.kUp)).withTimeout(4),
      new RunCommand(() -> drive.arcadeDrive(kBackupSpeed, 0),drive)
    );
    
    ParallelDeadlineGroup goForward = new ParallelDeadlineGroup(
      new WaitCommand(1),
      new RunCommand(() -> drive.arcadeDrive(kBackupSpeed, 0),drive)
    );

    
    sequence.addCommands(goOver);
    sequence.addCommands(goForward);
    return sequence;
  }

  public static CommandBase chargeAfterMoblity(DriveSubsystem drive) {

    SequentialCommandGroup sequence = new SequentialCommandGroup();
    
// we are now pointing towards the charge station. We want to move forward until we are pointed
// "up" which is actually down based on the mobilityOverChargeStation method above ::
// chargeSupplier(drive,BalanceState.kDown)   <--- We need to change to this

// original code that does not stop (probably because we are going backwards?) *** DO NOT USE ***
    // ParallelDeadlineGroup goBack = new ParallelDeadlineGroup(
    //     new WaitUntilCommand(() -> chargeSupplier(drive,BalanceState.kUp)).withTimeout(3),
    //     new RunCommand(() -> drive.arcadeDrive(kBackupSpeed, 0),drive)
    //   );

// proposed new code: rolls until the back is pointing down
    // ParallelDeadlineGroup goBack = new ParallelDeadlineGroup(
    //     new WaitUntilCommand(() -> chargeSupplier(drive,BalanceState.kDown)).withTimeout(3),
    //     new RunCommand(() -> drive.arcadeDrive(kBackupSpeed, 0),drive)
    //   );

// Proposed "balance" code
    // ParallelDeadlineGroup goLevel = new ParallelDeadlineGroup(
    //    new WaitUntilCommand(() -> chargeSupplier(drive,BalanceState.kLevel)).withTimeout(2.55),
    //    new RunCommand(() -> drive.arcadeDrive(kBackupSpeed, 0),drive)

    // Current code to get to the middle of the charge station with a timer
    // **** COMMENT THIS OUT (or remove)
      ParallelDeadlineGroup goLevel = new ParallelDeadlineGroup(
        //new WaitUntilCommand(() -> chargeSupplier(drive,BalanceState.kLevel)).withTimeout(3),
        // new WaitCommand(2.57),
        new WaitCommand(2.72),
        new RunCommand(() -> drive.arcadeDrive(kBackupSpeed, 0),drive)
      );

      //sequence.addCommands(goBack);
      sequence.addCommands(goLevel);
      return sequence;
  }

  public static CommandBase moblityTurnAndCharge(DriveSubsystem drive) {
    SequentialCommandGroup sequence = new SequentialCommandGroup();
    
    sequence.addCommands(moblityOverChargeStation(drive));
    sequence.addCommands(rotate180(drive));
    sequence.addCommands(chargeAfterMoblity(drive));
    return sequence;
  }

  public static CommandBase testMoblityAndCharge(DriveSubsystem drive) {
    SequentialCommandGroup sequence = new SequentialCommandGroup();
    
    sequence.addCommands(moblityOverChargeStation(drive));
    sequence.addCommands(testChargeAfterMoblity(drive));
    sequence.addCommands(rotate90(drive));
    return sequence;
  }

  public static CommandBase justCharge(DriveSubsystem drive) {
    return new ParallelDeadlineGroup(
      new WaitCommand(kChargeDuration),
      new RunCommand(() -> drive.arcadeDrive(kBackupSpeed, 0),drive)
    );
  }


  public static CommandBase intakeMove(IntakeSubsystem intake, Double time, Double speed) {
    return new ParallelDeadlineGroup(
      new WaitCommand(time),
      new RunCommand(()-> intake.IntakeGo(speed), intake));
    
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

  public static CommandBase armMove(ArmSubsystem arm, BooleanSupplier armEncoderSupplier, Double time, Double speed) {
    return new ParallelDeadlineGroup(
      new WaitCommand(time),
      new RunCommand(()-> arm.ArmGo(speed), arm)
    );
  }

  public static CommandBase armStop(ArmSubsystem arm) {
    return new InstantCommand(() -> arm.ArmGo(0), arm);
    
  }
  
  public static CommandBase blueDropMoblityTurnAndCharge(ArmSubsystem arm, IntakeSubsystem intake, DriveSubsystem drive)
  {
    SequentialCommandGroup sequence = new SequentialCommandGroup();

    sequence.addCommands(drop(arm, intake));
    sequence.addCommands(moblityTurnAndCharge(drive));

    return sequence;

  }

  public static CommandBase redDropMoblityTurnAndCharge(ArmSubsystem arm, IntakeSubsystem intake, DriveSubsystem drive)
  {
    SequentialCommandGroup sequence = new SequentialCommandGroup();

    sequence.addCommands(drop(arm, intake));
    sequence.addCommands(moblityTurnAndCharge(drive));

    return sequence;

  }

  public static CommandBase drop(ArmSubsystem arm, IntakeSubsystem intake) {

    SequentialCommandGroup sequence = new SequentialCommandGroup();

    ParallelCommandGroup closeAll = new ParallelCommandGroup(
    (intakeMove(intake, kOpenAndCloseIntakeDuration, kIntakeBackwardsSpeed)),
    //(intakeStop(intake));
    (armMove(arm, () -> armEncoderSupplier(arm, kArmRaiseMid),kArmCloseMidTime, kArmBackwardsSpeed))
    //(armStop(arm));
    );

    sequence.addCommands(arm.resetEncoders());
    sequence.addCommands(intake.resetEncoders());
    sequence.addCommands(armMove(arm, () -> armEncoderSupplier(arm, kArmRaiseMid), kArmRaiseMidTime, kArmForwardSpeed));
    sequence.addCommands(armStop(arm));
    sequence.addCommands(intakeMove(intake, kOpenAndCloseIntakeDuration, kIntakeForwardSpeed));
    sequence.addCommands(intakeStop(intake));
    sequence.addCommands(closeAll);
    
  

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
    //sequence.addCommands(rotate180(drive));
    return sequence;
  }

  public static CommandBase dropAndCharge(DriveSubsystem drive, ArmSubsystem arm, IntakeSubsystem intake) {
    

    // Build a sequential command
    SequentialCommandGroup sequence = new SequentialCommandGroup();

    sequence.addCommands(drop(arm, intake));
    sequence.addCommands(justCharge(drive));

    return sequence;
  }

  public static CommandBase testChargeAfterMoblity(DriveSubsystem drive) {

    SequentialCommandGroup sequence = new SequentialCommandGroup();
    
// we are now pointing towards the charge station. We want to move forward until we are pointed
// "up" which is actually down based on the mobilityOverChargeStation method above ::
// chargeSupplier(drive,BalanceState.kDown)   <--- We need to change to this

// original code that does not stop (probably because we are going backwards?) *** DO NOT USE ***
    // ParallelDeadlineGroup goBack = new ParallelDeadlineGroup(
    //     new WaitUntilCommand(() -> chargeSupplier(drive,BalanceState.kUp)).withTimeout(3),
    //     new RunCommand(() -> drive.arcadeDrive(kBackupSpeed, 0),drive)
    //   );


// proposed new code: rolls until the back is pointing down
    ParallelDeadlineGroup goBack = new ParallelDeadlineGroup(
        new WaitUntilCommand(() -> chargeSupplier(drive,BalanceState.kUp)).withTimeout(3),
        new RunCommand(() -> drive.arcadeDrive(-kBackupSpeed, 0),drive)
      );

// Proposed "balance" code
    ParallelDeadlineGroup goLevel = new ParallelDeadlineGroup(
       new WaitUntilCommand(() -> chargeSupplier(drive,BalanceState.kLevel)).withTimeout(4),
       new RunCommand(() -> drive.arcadeDrive(-kBackupSpeed* 0.7, 0), drive));

    // // Current code to get to the middle of the charge station with a timer
    // // **** COMMENT THIS OUT (or remove)
    //   ParallelDeadlineGroup goLevel = new ParallelDeadlineGroup(
    //     //new WaitUntilCommand(() -> chargeSupplier(drive,BalanceState.kLevel)).withTimeout(3),
    //     new WaitCommand(2.53),
    //     new RunCommand(() -> drive.arcadeDrive(kBackupSpeed, 0),drive)
    //   );
    sequence.addCommands(new WaitCommand(0.5));
    sequence.addCommands(goBack);
    sequence.addCommands(goLevel);
    return sequence;
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
