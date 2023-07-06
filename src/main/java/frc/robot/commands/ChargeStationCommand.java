// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Sensors;

public class ChargeStationCommand extends CommandBase {
  DriveSubsystem drive;
  Sensors sensors;

  double pitchDiff = 2;
  double halfChargeAngle = 32;
  Boolean result = false;
  int commandValue = 0; 
  double pitch = sensors.m_navx.getRoll();
  double delayedPitch = sensors.m_navx.getRoll() + pitchDiff;

  public Commands Charge(BooleanSupplier halfWayThere) {
     if (null != sensors.m_navx) {
      // if (pitch < delayedPitch) {
      // System.out.print("Yes");
      // }
      
      if(pitch <=  halfChargeAngle){
        new RunCommand(() -> drive.arcadeDrive(0.05, 0));
        new WaitUntilCommand(halfWayThere);
        
        
      }
      

    }

    else {
      throw new Error("Navx is not working");
    }
    return null;
  }
  
  private final Boolean halfWayThere(){
    if(pitch >= delayedPitch && pitch <= drive.m_lastPitchValue +3){
      return true;
    }
    else {
    return false;
    }
  }
  


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!interrupted) {
      drive.setIdleMode(IdleMode.kBrake);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
