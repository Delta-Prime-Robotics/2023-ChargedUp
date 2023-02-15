package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.*;

public class IntakeSubsystem extends SubsystemBase {
    //Defining the Intake motor
    private CANSparkMax m_intakeMotor;

    //Defiding the intake encoder
    private RelativeEncoder m_intakeEncoder;

    //Intake speed scale factor
    private final double kScaleFactor = 0.5;


    public IntakeSubsystem() {
        m_intakeMotor = new CANSparkMax(RoboRio.CanID.kIntakeControler, MotorType.kBrushless);
        
        m_intakeEncoder = m_intakeMotor.getEncoder();
        // Need  value for Intake Encoder in Constents
        m_intakeEncoder.setPositionConversionFactor(1);

        //m_intakeMotor.setSmartCurrentLimit(30, 90, 10);
    }

    /**
   * This command resets the intake encoders
   * 
   * @return a runOnce
   */
  public CommandBase resetEncoders() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
            m_intakeEncoder.setPosition(0.0);
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Motor Speed", m_intakeMotor.get());
  }


  public CommandBase IntakeGo(double speed) {
    return run(
    () -> {
      double intakeSpeed = speed;
      if (intakeSpeed > -1) {
        intakeSpeed = -1;
      }
      else if (intakeSpeed > 1){
        intakeSpeed = 1;
      }
      m_intakeMotor.set(speed * kScaleFactor);

    });
  }
}
