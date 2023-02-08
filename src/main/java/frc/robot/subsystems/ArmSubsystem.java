package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ArmSubsystem extends SubsystemBase {
    //defining the spark max arm controler
    private CANSparkMax m_armMotor;
    
    //Defineing the arm encoder
    private RelativeEncoder m_armEncoder;
    
    public ArmSubsystem() {
        m_armMotor = new CANSparkMax(RoboRio.CanID.kArmControler, MotorType.kBrushless);
        //Need  value for Arm Encoder in Constents
        // m_armEncoder = m_armControler.getEncoder();
        
        // m_armEncoder.setPositionConversionFactor(0)

        m_armMotor.setSmartCurrentLimit(30, 90, 10);
    }
    
     /**
   * This command resets the arm Encoders
   * 
   * @return a runOnce
   */
    public CommandBase resetEncoders() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> {
                m_armEncoder.setPosition(0.0);
            });
      }
    
      @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Motor Speed", m_armMotor.get());
  }

  public void stop() {
    m_armMotor.set(0);
  }
}
